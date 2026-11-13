/*
 * Moonlight for HarmonyOS
 * Copyright (C) 2024-2025 Moonlight/AlkaidLab
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

/**
 * DS5 LED 颜色 → 雷蛇骑仕 Ultra Chroma 灯光同步。
 *
 * 数据侧：主机 0x5502 Set RGB LED → BridgeClSetControllerLED →
 * StreamingSession.setControllerLED(cn,r,g,b) → 本类 onHostLed()。
 *
 * 硬件侧（已真机确证，2026-11-13 实测）：
 *  - 命令号：class 0x0F / command_id 0x03（即公开 Chroma 协议的 SetChromaFrame）。
 *    真机逐帧 GET_REPORT 读回 status=0x02 SUCCESS、tx 递增匹配，已锁定。
 *  - 负载：SetChromaFrame = [00 00 00 00 03] + RGB×4（17B），
 *    即 Profile=VOLATILE / Region=ALL / Effect=NONE / Start=0 / End=3。
 *  - 传输：Protocol2.5 90B feature report，HID SET_REPORT 0x21/0x09/0x0300。
 *
 * 隔离约束（踩过的坑，勿改）：
 *  1. 只往 id3 / Protocol2.5 接口发，绝不碰 id4 触感接口。
 *  2. 不 claim 接口：force claim 会踢掉内核 HID 驱动、破坏 0x0E 保底链路。
 *  3. 与控制器共用同一条 pipe，UsbDriverService 三处清理路径同步 detach。
 *  4. 所有发送串行化 + 颜色去重 + 节流，不与触感 SET_REPORT 兜底抢 pipe。
 *  5. 默认关闭。关闭时 0x0E 触感链路逐字节不变。
 */

import { usbManager } from '@kit.BasicServicesKit';
import { Logger } from '../../common/Logger';
import { PreferencesUtil } from '../../utils/PreferencesUtil';
import { SettingsKeys } from '../SettingsService';

const TAG = 'RazerChroma';

/** 雷蛇 VID */
const RAZER_VENDOR_ID = 0x1532;
/** HID 类 */
const USB_CLASS_HID = 0x03;
/** interrupt 传输类型 */
const INTERRUPT_TRANSFER_TYPE = 3;
/** Protocol2.5 feature report 固定长度 */
const REPORT25_LENGTH = 90;
/** HID SET_REPORT / GET_REPORT 控制传输 */
const SET_REPORT_BM_REQUEST_TYPE = 0x21;
const SET_REPORT_B_REQUEST = 0x09;
const GET_REPORT_BM_REQUEST_TYPE = 0xA1;
const GET_REPORT_B_REQUEST = 0x01;
const SET_REPORT_FEATURE_W_VALUE = 0x0300;
const SET_REPORT_OUTPUT_W_VALUE = 0x0202;
const CONTROL_TIMEOUT_MS = 120;
const RESPONSE_TIMEOUT_MS = 100;
const BULK_TIMEOUT_MS = 100;
/** Protocol2.5 是命令-应答式：写完等一小会儿再读响应 */
const RESPONSE_DELAY_MS = 12;
/** 颜色变化后的最小发送间隔：主机 LED 包高频，必须去抖 */
const MIN_SEND_INTERVAL_MS = 120;
/** 自定义帧需要周期刷新才不会被设备自身灯效覆盖 */
const KEEPALIVE_MS = 5000;
/** DS5 玩家 1 默认灯色：暗蓝。首次开启且还没收到主机包时用它点亮 */
const DEFAULT_RGB = 0x000040;
/** 手动测色时每帧保持的最小时长（仅用于节流判断） */

/** 是否把主机色的亮度归一化到设备满量程（保持色相，只提亮度） */
const NORMALIZE_BRIGHTNESS = true;

/** 一条 Protocol2.5 命令 */
interface ChromaCommand {
  commandClass: number;
  commandId: number;
  payload: number[];
}

/** 一条命令的执行结果：传输成功 ≠ 设备接受 */
interface CommandResult {
  sent: boolean;
  status: number;      // 响应首字节；-1 = 没读到响应
  accepted: boolean;   // status == 0x02
}

/**
 * 已确证的灯效命令（2026-11-13 真机锁定）。
 * 不再做命令号扫描：扫描只在 v73 调试期用于定位，确认后固化。
 */
const LOCKED_COMMAND: ChromaCommand = { commandClass: 0x0F, commandId: 0x03, payload: [] };

/** 打包后的颜色：0x00RRGGBB */
function packRgb(r: number, g: number, b: number): number {
  const rr = Math.max(0, Math.min(255, Math.round(r))) & 0xFF;
  const gg = Math.max(0, Math.min(255, Math.round(g))) & 0xFF;
  const bb = Math.max(0, Math.min(255, Math.round(b))) & 0xFF;
  return (rr << 16) | (gg << 8) | bb;
}

function rgbText(packed: number): string {
  return `(${(packed >> 16) & 0xFF},${(packed >> 8) & 0xFF},${packed & 0xFF})`;
}

function rgbHex(packed: number): string {
  const s = '000000' + packed.toString(16);
  return '#' + s.substring(s.length - 6).toUpperCase();
}

function hex2(n: number): string {
  return '0x' + ('0' + (n & 0xFF).toString(16)).slice(-2);
}

/**
 * 主机下发的 DS5 灯条色是相对亮度（例如玩家 1 只有 (0,0,64)）。
 * 保持 RGB 比例把最大分量提到 255，避免「匹配上了但暗到看不见」。
 * 全黑返回原值。
 */
function normalizeBrightness(packed: number): number {
  if (!NORMALIZE_BRIGHTNESS) return packed;
  const r = (packed >> 16) & 0xFF;
  const g = (packed >> 8) & 0xFF;
  const b = packed & 0xFF;
  const m = Math.max(r, g, b);
  if (m === 0 || m >= 255) return packed;
  const k = 255 / m;
  return packRgb(r * k, g * k, b * k);
}

/** Protocol2.5 响应首字节含义（公开协议定义） */
function statusText(status: number): string {
  switch (status) {
    case 0x00: return 'NEW/未处理';
    case 0x01: return 'BUSY';
    case 0x02: return 'SUCCESS/设备已接受';
    case 0x03: return 'FAILURE';
    case 0x04: return 'TIMEOUT';
    case 0x05: return 'UNSUPPORTED/命令号不被支持';
    case -1: return '无响应（GET_REPORT 不可读）';
    default: return `未知(${status})`;
  }
}

/** SetChromaFrame 负载：5B 头 + 4 个 zone 的 RGB */
function colorFramePayload(packed: number): number[] {
  const r = (packed >> 16) & 0xFF;
  const g = (packed >> 8) & 0xFF;
  const b = packed & 0xFF;
  const payload: number[] = [0x00, 0x00, 0x00, 0x00, 0x03];
  for (let i = 0; i < 4; i++) {
    payload.push(r, g, b);
  }
  return payload;
}

/**
 * 雷蛇 Chroma 灯光同步驱动（单例）。
 * UsbDriverService 识别到雷蛇设备后 attach(device, pipe)，
 * StreamingSession.setControllerLED 每次收到 0x5502 都调 onHostLed()。
 */
export class RazerKishiChroma {
  private static instance: RazerKishiChroma | null = null;

  static getInstance(): RazerKishiChroma {
    if (RazerKishiChroma.instance === null) {
      RazerKishiChroma.instance = new RazerKishiChroma();
    }
    return RazerKishiChroma.instance;
  }

  // ---- 传输状态 ----
  private pipe: usbManager.USBDevicePipe | null = null;
  private protocolInterfaceId: number = -1;
  private outEndpoint: usbManager.USBEndpoint | null = null;
  private attached: boolean = false;
  private deviceKey: string = '';

  // ---- 开关与颜色 ----
  private enabled: boolean = false;
  /** 最近一次主机下发的**非黑**颜色（0x00RRGGBB），持久化 */
  private lastRgb: number = DEFAULT_RGB;
  private lastSentRgb: number = -1;
  private hostPacketCount: number = 0;
  private blackPacketCount: number = 0;
  private lastBlackLogMs: number = 0;
  private lastHostMs: number = 0;

  // ---- 发送队列 ----
  private pendingRgb: number = -1;
  private sending: boolean = false;
  private lastSendMs: number = 0;
  private retryTimer: number = -1;
  private keepaliveTimer: number = -1;
  private transactionId: number = 0x1F;

  // ---- 诊断状态（供设置页显示） ----
  private lastResponseStatus: number = -1;
  private lastSendDetail: string = '尚未发送';

  private constructor() {
    this.loadPersisted();
  }

  /** 异步恢复开关与上次颜色；不阻塞单例创建 */
  private loadPersisted(): void {
    PreferencesUtil.get<boolean>(SettingsKeys.RAZER_DS5_LED_SYNC, false)
      .then((on: boolean): void => {
        this.enabled = on === true;
        // USB 设备可能比设置先加载完（覆盖安装后重进串流就是这种情况）：
        // 此时 attach() 里那次「开关已开立即接管」会落空，这里补一次。
        if (this.enabled && this.attached) {
          Logger.info(TAG, '持久化开关恢复完成，补一次灯效接管');
          this.startKeepalive();
          this.requestPush(true);
        }
      })
      .catch((err: Error): void => {
        Logger.warn(TAG, '读取灯效开关失败:', err);
      });
    PreferencesUtil.get<number>(SettingsKeys.RAZER_DS5_LED_RGB, DEFAULT_RGB)
      .then((rgb: number): void => {
        if (typeof rgb === 'number' && Number.isFinite(rgb) && rgb !== 0) {
          this.lastRgb = rgb & 0xFFFFFF;
        }
        // 设置可能在 USB 设备之后才加载完：补一次接管
        if (this.enabled && this.attached) this.requestPush(true);
      })
      .catch((err: Error): void => {
        Logger.warn(TAG, '读取上次灯色失败:', err);
      });
  }

  // ==================== 设备识别与挂载 ====================

  /**
   * 定位 id3 / Razer Protocol2.5 接口。
   * 真机拓扑：id3 HID keyboard OUT0x02/16B（Protocol2.5）、
   * id4 HID Haptics IN0x85/64B OUT0x05/64B（0x0E 触感）。
   * 明确排除 64B 触感接口：往它发设置类命令会让设备停止播放帧直到重新插拔。
   */
  attach(device: usbManager.USBDevice, pipe: usbManager.USBDevicePipe): boolean {
    if (device.vendorId !== RAZER_VENDOR_ID || !device.configs) return false;
    if (device.productId === 0x0037) {
      Logger.info(TAG, 'XInput 模式无 Protocol2.5 接口，跳过灯效挂载');
      return false;
    }
    let bestIface: usbManager.USBInterface | null = null;
    let bestEp: usbManager.USBEndpoint | null = null;
    let bestScore = -1000;
    for (let ci = 0; ci < device.configs.length; ci++) {
      const config = device.configs[ci];
      if (!config.interfaces) continue;
      for (let ii = 0; ii < config.interfaces.length; ii++) {
        const iface = config.interfaces[ii];
        if (iface.clazz !== USB_CLASS_HID) continue;
        const eps = iface.endpoints;
        if (!eps) continue;
        for (let ei = 0; ei < eps.length; ei++) {
          const ep = eps[ei];
          if ((ep.direction & 0x80) !== 0) continue;  // 只要 OUT
          if (ep.maxPacketSize >= 64) continue;       // 排除 64B 触感接口
          const xfer = ep.attributes & 0x03;
          if (xfer !== INTERRUPT_TRANSFER_TYPE && ep.type !== INTERRUPT_TRANSFER_TYPE) continue;
          let score = 10;
          if (ep.address === 0x02) score += 8;
          if (ep.maxPacketSize === 16) score += 4;
          if (iface.subClass === 0) score += 1;
          if (score > bestScore) {
            bestScore = score;
            bestIface = iface;
            bestEp = ep;
          }
        }
      }
    }
    if (bestIface === null || bestEp === null) {
      Logger.warn(TAG, '未找到 Protocol2.5 接口（HID 类 + 16B interrupt OUT），灯效不可用');
      return false;
    }

    const incomingKey = `${device.vendorId}:${device.productId}:${device.busNum}:${device.devAddress}`;
    this.pipe = pipe;
    this.protocolInterfaceId = bestIface.id;
    this.outEndpoint = RazerKishiChroma.buildEndpoint(bestEp, bestIface.id);
    this.deviceKey = incomingKey;
    this.attached = true;
    this.lastSentRgb = -1;
    Logger.info(TAG,
      `Protocol2.5 接口就绪: id=${bestIface.id} OUT=0x${bestEp.address.toString(16)}/` +
      `${bestEp.maxPacketSize}B key=${incomingKey} enabled=${this.enabled} 命令=${hex2(LOCKED_COMMAND.commandClass)}/${hex2(LOCKED_COMMAND.commandId)}`);

    // 开关已打开：设备一挂上就立刻用最近一次主机色接管灯效。
    if (this.enabled) {
      Logger.info(TAG, `开关已开，立即接管灯效并套用主机颜色 rgb=${rgbText(this.lastRgb)}`);
      this.startKeepalive();
      this.requestPush(true);
    }
    return true;
  }

  /** OHOS bulkTransfer 依赖端点里的 interfaceId/number/type，显式重建 */
  private static buildEndpoint(ep: usbManager.USBEndpoint,
    interfaceId: number): usbManager.USBEndpoint {
    return {
      address: ep.address,
      attributes: INTERRUPT_TRANSFER_TYPE,
      interval: ep.interval,
      maxPacketSize: ep.maxPacketSize,
      direction: 0x00 as usbManager.USBRequestDirection,
      interfaceId: interfaceId,
      number: ep.address,
      type: INTERRUPT_TRANSFER_TYPE
    };
  }

  /** 设备拔出 / pipe 即将关闭时调用；必须由 UsbDriverService 三处清理路径同步调用 */
  detach(): void {
    if (!this.attached) return;
    this.stopKeepalive();
    if (this.retryTimer !== -1) {
      clearTimeout(this.retryTimer);
      this.retryTimer = -1;
    }
    this.attached = false;
    this.pipe = null;
    this.outEndpoint = null;
    this.protocolInterfaceId = -1;
    this.deviceKey = '';
    this.pendingRgb = -1;
    this.lastSentRgb = -1;
    Logger.info(TAG, 'Protocol2.5 接口已释放（不关闭共用 pipe，不 claim 过的接口无需 release）');
  }

  get isAttached(): boolean {
    return this.attached;
  }

  // ==================== 开关 ====================

  get isEnabled(): boolean {
    return this.enabled;
  }

  /**
   * 打开：立刻用最近一次主机 RGB 发一帧（不等下一次 0x5502）+ 启动 keepalive。
   * 关闭：停止转发并尽力把灯交还设备自管（见 restore()）。
   */
  setEnabled(on: boolean): void {
    if (this.enabled === on) return;
    this.enabled = on;
    PreferencesUtil.put(SettingsKeys.RAZER_DS5_LED_SYNC, on).catch((err: Error): void => {
      Logger.warn(TAG, '保存灯效开关失败:', err);
    });
    if (on) {
      this.lastSentRgb = -1;
      this.startKeepalive();
      Logger.info(TAG, `开关打开：立即接管灯效，当前主机色 rgb=${rgbText(this.lastRgb)}`);
      this.requestPush(true);
    } else {
      this.stopKeepalive();
      this.restore();
      Logger.info(TAG, '开关关闭：停止转发主机灯色');
    }
  }

  /**
   * 手动测色：直接发任意 RGB，绕过主机心跳，用于验证灯效链路。
   * 会更新 lastRgb（持久化），因此关闭开关前灯会保持该色；
   * 若之后主机下发非黑颜色，会被新颜色覆盖（符合「跟随主机」语义）。
   */
  testColor(r: number, g: number, b: number): void {
    if (!this.attached) {
      Logger.warn(TAG, '手动测色：设备未连接');
      return;
    }
    const packed = packRgb(r, g, b);
    this.lastRgb = packed;
    PreferencesUtil.put(SettingsKeys.RAZER_DS5_LED_RGB, packed).catch((err: Error): void => {
      Logger.warn(TAG, '保存手动测色失败:', err);
    });
    if (this.enabled) {
      this.requestPush(true);
      Logger.info(TAG, `手动测色 rgb=${rgbText(packed)} 已发送`);
    } else {
      Logger.info(TAG, `手动测色 rgb=${rgbText(packed)}：开关未开，仅记录不发送（打开开关即生效）`);
    }
  }

  // ==================== 主机 LED 输入 ====================

  /**
   * StreamingSession.setControllerLED 的落点。
   *
   * ⚠️ Sunshine/Ds5Sidecar 每 ~0.5s 下发一次 (0,0,0) 心跳（实测 96% 的包都是它），
   * 这是「主机没有灯效数据」的占位值，**不是灭灯指令**：真实 PS5 玩家 1
   * 灯条是常亮暗蓝的。若把它当灭灯，开关打开不到一秒灯就被刷黑了。
   * 因此全黑包只计数、不覆盖 lastRgb、不触发转发。
   */
  onHostLed(controllerNumber: number, r: number, g: number, b: number): void {
    const packed = packRgb(r, g, b);
    this.hostPacketCount++;
    this.lastHostMs = Date.now();
    if (packed === 0) {
      this.blackPacketCount++;
      if (this.lastHostMs - this.lastBlackLogMs > 10000) {
        this.lastBlackLogMs = this.lastHostMs;
        Logger.info(TAG, `主机下发全黑占位包（第 ${this.blackPacketCount} 个），保持当前灯色 ` +
          `${rgbHex(this.lastRgb)} 不变；如需真正灭灯请关闭本开关`);
      }
      return;
    }
    if (packed !== this.lastRgb) {
      this.lastRgb = packed;
      PreferencesUtil.put(SettingsKeys.RAZER_DS5_LED_RGB, packed).catch((err: Error): void => {
        Logger.warn(TAG, '保存主机灯色失败:', err);
      });
      Logger.info(TAG, `主机灯色变化 cn=${controllerNumber} rgb=${rgbText(packed)} ` +
        `第 ${this.hostPacketCount} 包 enabled=${this.enabled}`);
    }
    if (this.enabled) this.requestPush(false);
  }

  /** 供设置页显示实时状态（纯读，不触发任何 USB 操作） */
  getStatusText(): string {
    const host = `主机${rgbHex(this.lastRgb)} → 输出${rgbHex(normalizeBrightness(this.lastRgb))}`;
    const link = this.attached
      ? `已连接 iface=${this.protocolInterfaceId}`
      : '未检测到 Protocol2.5 接口';
    const cmd = `已锁定 ${hex2(LOCKED_COMMAND.commandClass)}/${hex2(LOCKED_COMMAND.commandId)}`;
    const resp = `响应=${statusText(this.lastResponseStatus)}`;
    return `${host} · 收包 ${this.hostPacketCount}（黑 ${this.blackPacketCount}）· ${link} · ${cmd} · ${resp}`;
  }

  /** 页面色块显示的是**实际写进手柄**的颜色（亮度归一化后） */
  getPreviewColor(): string {
    return rgbHex(normalizeBrightness(this.lastRgb));
  }

  // ==================== 发送调度 ====================

  /** 颜色去重 + 节流；force 用于开关打开的「立刻接管」或手动测色 */
  private requestPush(force: boolean): void {
    if (!this.enabled || !this.attached || this.pipe === null) return;
    const now = Date.now();
    if (!force && this.lastRgb === this.lastSentRgb) return;
    if (!force && now - this.lastSendMs < MIN_SEND_INTERVAL_MS) {
      if (this.retryTimer === -1) {
        this.retryTimer = setTimeout((): void => {
          this.retryTimer = -1;
          this.requestPush(false);
        }, MIN_SEND_INTERVAL_MS);
      }
      return;
    }
    this.pendingRgb = this.lastRgb;
    this.drain();
  }

  /** 串行发送，保证控制传输不与自身/触感兜底抢 pipe */
  private drain(): void {
    if (this.sending) return;
    this.sending = true;
    this.drainStep().then((): void => {
      this.sending = false;
      if (this.pendingRgb >= 0 && this.enabled && this.attached) this.drain();
    }).catch((err: Error): void => {
      this.sending = false;
      Logger.warn(TAG, '发送队列异常:', err);
    });
  }

  private async drainStep(): Promise<void> {
    const packed = this.pendingRgb;
    this.pendingRgb = -1;
    if (packed < 0) return;
    const ok = await this.applyColor(packed);
    this.lastSendMs = Date.now();
    if (ok) {
      this.lastSentRgb = packed;
      this.lastSendDetail = `已写入 ${rgbHex(normalizeBrightness(packed))}`;
    } else {
      this.lastSendDetail = '命令未被设备接受（见 hilog RazerChroma 逐条 status）';
    }
  }

  /**
   * 把颜色写进设备：用已锁定的命令 0x0F/0x03 发一条 SetChromaFrame，
   * 写完用 GET_REPORT 读回 status（0x02=成功）；响应通道不可读时退化为
   * 「传输成功」。命令号不再扫描，已真机确证。
   */
  private async applyColor(packed: number): Promise<boolean> {
    const outRgb = normalizeBrightness(packed);
    const cmd: ChromaCommand = {
      commandClass: LOCKED_COMMAND.commandClass,
      commandId: LOCKED_COMMAND.commandId,
      payload: colorFramePayload(outRgb)
    };
    const res = await this.sendCommand(cmd, `帧色 ${hex2(cmd.commandClass)}/${hex2(cmd.commandId)}`);
    this.lastResponseStatus = res.status;
    if (res.accepted || (res.sent && res.status < 0)) {
      return true;  // 设备接受；或响应通道不可读时退化为「传输成功」
    }
    Logger.warn(TAG, `灯效命令本次被拒 status=${statusText(res.status)}（命令已锁定，正常不应发生）`);
    return false;
  }

  /** 构造 Protocol2.5 90B feature report */
  private buildReport(cmd: ChromaCommand): Uint8Array {
    const r = new Uint8Array(REPORT25_LENGTH);
    this.transactionId = (this.transactionId + 1) & 0xFF;
    if (this.transactionId === 0) this.transactionId = 0x1F;
    r[0] = 0x00;                    // status: NEW command
    r[1] = this.transactionId;
    // r[2..3] remaining packets = 0, r[4] protocol type = 0
    const payload = cmd.payload;
    const len = Math.min(payload.length, 80);
    r[5] = len;
    r[6] = cmd.commandClass;
    r[7] = cmd.commandId;
    for (let i = 0; i < len; i++) {
      r[8 + i] = payload[i] & 0xFF;
    }
    let crc = 0;
    for (let i = 2; i < 88; i++) {
      crc ^= r[i];
    }
    r[88] = crc & 0xFF;
    return r;
  }

  /**
   * 读回 Protocol2.5 响应：HID GET_REPORT (0xA1/0x01/0x0300)。
   * 首字节是命令状态，0x02 = 设备已执行；0x05 = 命令号不支持。
   */
  private async readResponse(expectedTx: number, label: string): Promise<number> {
    if (!this.attached || this.pipe === null) return -1;
    const buf = new Uint8Array(REPORT25_LENGTH);
    const params: usbManager.USBDeviceRequestParams = {
      bmRequestType: GET_REPORT_BM_REQUEST_TYPE,
      bRequest: GET_REPORT_B_REQUEST,
      wValue: SET_REPORT_FEATURE_W_VALUE,
      wIndex: this.protocolInterfaceId,
      wLength: REPORT25_LENGTH,
      data: buf
    };
    try {
      const ret = await usbManager.usbControlTransfer(this.pipe, params, RESPONSE_TIMEOUT_MS);
      if (typeof ret !== 'number' || ret <= 0) {
        Logger.info(TAG, `${label} GET_REPORT ret=${ret} → 响应不可读`);
        return -1;
      }
      // 成功路径的逐帧响应日志已移除：实测游戏持续变色时每帧回 0x02 SUCCESS，
      // 会刷爆 hilog 缓冲、冲掉其它排障信息。仅在解析异常时由下方 catch 打 WARN。
      return buf[0];
    } catch (err) {
      Logger.warn(TAG, `${label} GET_REPORT 异常: ${err}`);
      return -1;
    }
  }

  /**
   * 发送一条命令并读回设备响应。
   * 通道依次降级：SET_REPORT Feature(0x0300) → Output(0x0202) → interrupt OUT。
   * 只有响应 status=0x02 才算「设备接受」；传输成功但 unsupported 会被拒。
   */
  private async sendCommand(cmd: ChromaCommand, label: string): Promise<CommandResult> {
    if (!this.attached || this.pipe === null) {
      return { sent: false, status: -1, accepted: false };
    }
    const report = this.buildReport(cmd);
    const head = `${label} len=${cmd.payload.length}`;
    const wValues: number[] = [SET_REPORT_FEATURE_W_VALUE, SET_REPORT_OUTPUT_W_VALUE];
    let anySent = false;
    let bestStatus = -1;
    let responseReadable = false;

    for (let i = 0; i < wValues.length; i++) {
      const wValue = wValues[i];
      const params: usbManager.USBDeviceRequestParams = {
        bmRequestType: SET_REPORT_BM_REQUEST_TYPE,
        bRequest: SET_REPORT_B_REQUEST,
        wValue: wValue,
        wIndex: this.protocolInterfaceId,
        wLength: REPORT25_LENGTH,
        data: report
      };
      try {
        const ret = await usbManager.usbControlTransfer(this.pipe, params, CONTROL_TIMEOUT_MS);
        if (typeof ret !== 'number' || ret <= 0) {
          // 该通道写失败：通常意味着设备不接受此 wValue，转下一条通道降级
          Logger.info(TAG, `${head} SET_REPORT(0x${wValue.toString(16)}) 未发送(ret=${ret})，尝试下一通道`);
          continue;
        }
        anySent = true;
        // 发送成功不打日志：实测每次 0x02 SUCCESS，高频会刷爆缓冲；失败/被拒才在 applyColor 打 WARN
        await RazerKishiChroma.delay(RESPONSE_DELAY_MS);
        const st = await this.readResponse(report[1], head);
        if (st >= 0) {
          responseReadable = true;
          if (st === 0x02) return { sent: true, status: st, accepted: true };
          if (bestStatus < 0 || st < bestStatus) bestStatus = st;
        }
      } catch (err) {
        Logger.warn(TAG, `${head} SET_REPORT(0x${wValue.toString(16)}) 异常: ${err}`);
      }
    }

    const ep = this.outEndpoint;
    if (ep !== null && !anySent) {
      try {
        const ret = await usbManager.bulkTransfer(this.pipe, ep, report, BULK_TIMEOUT_MS);
        const ok = typeof ret === 'number' && ret > 0;
        Logger.info(TAG, `${head} interrupt-OUT ret=${ret}`);
        if (ok) anySent = true;
      } catch (err) {
        Logger.warn(TAG, `${head} interrupt-OUT 异常: ${err}`);
      }
    }

    // 响应通道完全不可读时退化为「传输成功」，并在日志里明确标注，不当成设备确认
    const accepted = !responseReadable && anySent;
    return { sent: anySent, status: bestStatus, accepted: accepted };
  }

  /** 自定义帧需要周期刷新，否则会被设备自身灯效覆盖 */
  private startKeepalive(): void {
    if (this.keepaliveTimer !== -1) return;
    this.keepaliveTimer = setInterval((): void => {
      if (!this.enabled || !this.attached) return;
      if (Date.now() - this.lastSendMs < KEEPALIVE_MS - 500) return;
      this.requestPush(true);
    }, KEEPALIVE_MS);
  }

  private stopKeepalive(): void {
    if (this.keepaliveTimer !== -1) {
      clearInterval(this.keepaliveTimer);
      this.keepaliveTimer = -1;
    }
  }

  /**
   * 关闭开关时释放接管：写一条全黑帧，然后彻底停发。
   * 设备自身灯效恢复时机由固件决定；若灯仍停在主机色，重新插拔即可回原厂模式。
   */
  private restore(): void {
    if (!this.attached || this.pipe === null) return;
    const frame: ChromaCommand = {
      commandClass: LOCKED_COMMAND.commandClass,
      commandId: LOCKED_COMMAND.commandId,
      payload: colorFramePayload(0x000000)
    };
    this.sendCommand(frame, '释放接管/熄灯').then((res: CommandResult): void => {
      Logger.info(TAG, `释放接管: ${res.sent ? '已发熄灯帧' : '发送未通过'}；` +
        '若灯仍停在主机色，重新插拔手柄即可恢复原厂灯效');
    }).catch((err: Error): void => {
      Logger.warn(TAG, '释放接管异常:', err);
    });
    this.lastSentRgb = -1;
  }

  private static delay(ms: number): Promise<void> {
    return new Promise<void>(resolve => setTimeout(resolve, ms));
  }
}
