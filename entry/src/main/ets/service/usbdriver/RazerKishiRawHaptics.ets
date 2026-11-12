/*
 * Moonlight for HarmonyOS
 * Copyright (C) 2024-2025 Moonlight/AlkaidLab
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

/*
 * Razer Kishi Ultra / V3 Pro —— raw-PCM 波形直灌驱动
 *
 * v23 方向：PeaSyo（Geocld/PeaSyo, RazerKishiController.java）证实 Kishi 触感
 * 接口本质是「原始波形播放器」：PS5 3kHz stereo PCM → 4kHz 重采样 → 64B 帧
 * （头 55 AA 00 00 00 00 00 30 FE 79 + 48B payload + XOR[2..57] 校验 + 5B 尾零）
 * 经 INTERRUPT OUT 端点 bulkTransfer 直发执行器，全保真。
 *
 * v24 修正（2026-11-11 实测日志）：
 *  - PeaSyo 的 "Interface 3" 是 **Android 枚举索引**；鸿蒙枚举多出一个接口，
 *    真机 HID 模式(PID 0x71a)拓扑为：
 *      id=0 HID GamePad IN 0x81/32 | id=1 IN 0x82/32 | id=2 IN 0x83/16
 *      id=3 HID keyboard OUT 0x02/16   <- v23 误选，16B 装不下 64B 帧
 *      id=4 HID Haptics IN 0x85/64 OUT 0x05/64  <- 真正的触感接口
 *    所以本驱动改为按「HID 类 + maxPacketSize>=64 的 INTERRUPT OUT 端点」定位，
 *    不再硬编码接口索引。0x0E 驱动用的也是这个接口的 OUT 0x05。
 *  - 不独占 claim 接口：与 0x0E 驱动共用同一 pipe/端点，直接写端点；
 *    claimInterface(force) 会把内核 HID 驱动踢掉并干扰 0x0E 保底路径。
 *  - 握手改为尽力而为：Feature report(0x0300) 失败时再试 Output report(0x0202)，
 *    全失败也不放弃直发 —— 0x0E 路径从不握手也能震动，说明设备默认已启用。
 *  - 让路条件从「已挂载」收紧为「确实在成功发帧」(isActive)，
 *    raw 无帧流时 0x0E 照常工作，普通 rumble / 非 DS5 主机不会静音。
 *
 * v31 重大更正（2026-11-11 21:12，貔貅鸿蒙版串流 PS5 的 DEBUG 日志实证）：
 *  v30「鸿蒙 USB 栈不支持 raw-PCM」的结论是错的 —— 同一台平板、同一个 Kishi、
 *  同样是 @ohos.usbManager，貔貅 com.lijiahao.peasyooh 能正常震。
 *  真正差异是帧协议版本：貔貅鸿蒙版走 Razer Denise / HD Haptics，
 *    - 64B 帧首字节固定 0x51，前 15B 是 RazerEncoder 输出，其后 49B 补零
 *    - 没有 55 AA 帧头、没有 XOR 校验、没有 48B PCM 载荷
 *    - 不发任何 HID SET_REPORT 握手
 *    - 挂载时 claimInterface(iface3 + iface4, force=true) 均返回 0
 *    - 端点 iface4 OUT 0x05/64B，bulkTransfer 返回 64 成功
 *    成功帧 51 1e b0 f5 87 ac 3d 62 35 61 ab 0d 58 6a c0，
 *    静音帧 51 00 00 00 00 00 00 02 00 00 00 00 00 00 00。
 *  v31 保留 55AA / 0x0E 全部代码，新增 Denise 帧格式与 4 段协议试音；
 *  串流时 RAW_PCM_ENABLED 仍为 false，0x0E 保底不受影响。
 *
 * v33 收官结论（2026-11-11 21:46，用户回报「都不震」）：
 *  五轮试音（v29/v31/v32）已把端点、帧头(0x51)、claim、握手、帧率、
 *  「每帧重开 USB 管道」全部对齐，设备依然不播；且 v32 的重开实验会抢走
 *  0x0E 驱动的接口声明，导致连对照组都不震（重插拔后才恢复）。
 *  貔貅 DEBUG 日志给出真正的差异：
 *    [PeaSyo][RazerEncoder] startup smoke loaded=true symbolsReady=true
 *      setupRet=0 updateRet=0 encodeRet=0 getPayloadRet=0 payloadSize=15
 *      payloadPreview=a1 fd b7 ed bf 6d fb 63 fb 6f db 7e db f6 c0
 *  即貔貅加载了一个**私有编码器动态库**（HAP 内 libs/arm64），提供
 *  setup/update/encode/getPayload 四个符号，把 48kHz 触觉 PCM 实时编码成
 *  15B payload。设备只接受编码器产出的合法动态码流；我们灌的固定抓帧
 *  （重复同一帧）与伪造 PCM 都不是合法编码，因此 bulkTransfer=64 也不播。
 *  结论：协议复刻缺少的是这个 .so，不是传输层参数。HAP / .so 被 DAC 拒绝读取
 *  （Permission denied），需用户提供貔貅安装包才能继续。
 *  v33 撤销重开实验、移除会破坏链路的试音按钮，恢复 0x0E 独占保底。
 */

import { usbManager } from '@kit.BasicServicesKit';
import { Logger } from '../../common/Logger';

const TAG = 'RazerKishiRawHaptics';

/** 雷蛇 USB VID */
const RAZER_VENDOR_ID = 0x1532;
/** 端点传输类型：3 = interrupt */
const INTERRUPT_TRANSFER_TYPE = 3;
/** 挂载延迟：避开控制器异步校准窗口（与 0x0E 驱动一致） */
const ATTACH_DELAY_MS = 900;
/** INTERRUPT OUT 发送超时（毫秒）。v28 批量提交多帧需要更宽窗口 */
const RAW_OUT_TIMEOUT_MS = 100;
/** 连续这么多次发送失败则认为端点不可用，停止让路（回退 0x0E） */
const RAW_FAILURE_THRESHOLD = 20;
/** raw 帧流「活跃」判定窗口：超过这个时间没成功发出帧就交还 0x0E */
const RAW_ACTIVE_WINDOW_MS = 250;
/** v28: 队列上限。超出丢最旧帧，避免试音结束后残留十几秒的过期波形 */
const RAW_QUEUE_MAX = 32;
/**
 * v29: 严格逐帧 64B。v28 曾把 4 帧拼成 256B 一次 bulkTransfer，
 * 超过 interrupt 端点 maxPacketSize(64B)，固件根本不会当作 4 个独立帧解析。
 * PeaSyo 的 sender 也是逐帧 bulkTransfer(frame, 64, 100)。
 */
/** PeaSyo 语义：左右声道两条启用命令都成功，才认为执行器已被启用 */
const RAW_HANDSHAKE_MIN_OK = 2;
/**
 * v30: raw-PCM 直灌总开关，默认关闭。
 * ⚠️ v31 更正：v30「鸿蒙上判定不可用」是错误结论（貔貅鸿蒙版实测能震），
 * 根因是帧协议版本不同。此开关暂时保持 false，是为了在 0x51 协议验证通过前
 * 不改变串流行为，让已验证可用的 0x0E 独占端点。
 * 2026-11-11 20:46 四段协议试音实证（用户回报「只有 #1 震」）：
 *   #1 0x0E 对照组：report=02 13 00 01 0e + 64B 应答，正常震动 ✅
 *   #2 idx3 OUT 0x02/16B 手动分包：bulkTransfer 连 16B 都返回失败 → 该端点在鸿蒙完全不可写
 *   #3 idx4 OUT 0x05/64B 逐帧无握手：64B 写成功，设备不播（固件只认 0x0E 帧）
 *   #4 同上 + PeaSyo 握手：4 条 SET_REPORT 全 ret=-1，设备不播
 * 且设备注释明写「查询/设置类命令会让它停止播放帧直到重新插拔」。
 * 因此串流时 raw 必须完全静默，让 0x0E 独占端点；诊断试音仍可临时打开发帧。
 */
const RAW_PCM_ENABLED = false;

/**
 * v35: 官方编码器实时直灌总开关。
 * ★v57 体感对照实验（20:22）：**关闭**。
 *   背景：v52~v56 反复调整编码器增益/DSP 后，用户判定"粗、没有细腻感"，
 *   且整体回退到 v52 归一化基线后**依然不细腻** ⇒ 问题不在 DSP 参数，
 *   而在"编码器链路 vs 0x0E 合成链路"这条**路线本身**。
 *   本版把编码器整条关掉（RAW_PCM_ENABLED 也是 false ⇒ onRawHapticFrames 直接 return，
 *   armed 永不点亮 ⇒ isActive 恒 false ⇒ 0x0E 独占端点、全程工作），
 *   用于一次性判定"细腻感到底来自哪条链路"。
 *   - 若细腻感回来 ⇒ 用户基准就是 0x0E，编码器路线应降级/放弃；
 *   - 若仍不细腻 ⇒ 0x0E 的数据源（AudioVibrationService）也有问题，转查那条。
 */
const LIVE_ENCODER_ENABLED = false;

/** v31: 帧格式。0 = Android PeaSyo 55AA 帧；1 = 貔貅鸿蒙版 Razer Denise 0x51 帧 */
const RAW_FORMAT_LEGACY_55AA = 0;
const RAW_FORMAT_DENISE_51 = 1;

/**
 * v52: HD 触觉帧 —— 与「已验证能驱动马达」的 0x0E 帧**完全同构**的 20B Razer 包络。
 * 依据（RazerKishiHaptics.buildFrameReport 实测结构 + 真机日志）：
 *   [0x02][0x13][0x00][0x01][cmd][15B 位流]   ← report[1] = 4 + 15 = 0x13
 * 其中第 5 字节才是 command（0x0E 能震、0x51 是我们一直用的），
 * 所以 0x51 从来不是「外层帧头」，把 payload 塞进 64B 裸帧设备一律丢弃。
 */
const RAZER_HD_PACKET_BYTES = 20;
const RAZER_HD_PAYLOAD_BYTES = 15;
/**
 * v53: 固化用的 command 字节。v52 实测 0x0E / 0x51 两种都能驱动马达；
 * 默认用 0x0E（与已验证保底通路同款，行为最可预期）。想换 0x51 只改这一行。
 */
const RAZER_HD_CMD = 0x0E;

/** 单帧字节数（PeaSyo FRAME_SIZE=64） */
const RAW_FRAME_BYTES = 64;
/** 帧头字节数（PeaSyo HEADER_SIZE=10） */
const RAW_HEADER_SIZE = 10;
/** 载荷字节数（PeaSyo PAYLOAD_SIZE=48 = 24 个 int16 = 12 立体声样本） */
const RAW_PAYLOAD_SIZE = 48;
/** 载荷样本数（short 数） */
const RAW_PAYLOAD_SAMPLES = 24;
/** 校验和计算区间起点（PeaSyo: XOR from index 2） */
const RAW_CHECKSUM_START = 2;
/** 校验和存放位置 = HEADER_SIZE + PAYLOAD_SIZE = 58 */
const RAW_CHECKSUM_POS = RAW_HEADER_SIZE + RAW_PAYLOAD_SIZE;

/** HID 握手控制传输参数（Host->Device | Class | Interface, SET_REPORT） */
const RAW_HANDSHAKE_BM_REQUEST_TYPE = 0x21;
const RAW_HANDSHAKE_B_REQUEST = 0x09;
/** Feature report, report ID 0（PeaSyo 原样；v29 起不再尝试其他变体） */
const RAW_HANDSHAKE_W_VALUE_FEATURE = 0x0300;

/** v28: 端点候选。PeaSyo 硬编码 Android 的 interface 3，鸿蒙枚举拓扑不同，
 * 因此保留全部 INTERRUPT OUT 端点，由「端点扫描试音」逐个实测定死。 */
interface RawEndpointCandidate {
  iface: usbManager.USBInterface;
  ep: usbManager.USBEndpoint;
  /** config.interfaces 里的枚举序号（Android 的 getInterface(3) 用的就是它） */
  ifaceIndex: number;
  label: string;
}

/**
 * 雷蛇骑仕 raw-PCM 触感驱动（与 0x0E 共用的 64B INTERRUPT OUT 端点直灌）。
 *
 * 生命周期：UsbDriverService 在识别到雷蛇设备后调用 tryAttach()；
 * native 的 razerRawHaptic 回调把 64B raw 帧交给 onRawHapticFrames() 直发。
 * 设备拔出或链路失效时由 UsbDriverService 调用 detach()。
 */
export class RazerKishiRawHaptics {
  private static instance: RazerKishiRawHaptics | null = null;

  static getInstance(): RazerKishiRawHaptics {
    if (RazerKishiRawHaptics.instance === null) {
      RazerKishiRawHaptics.instance = new RazerKishiRawHaptics();
    }
    return RazerKishiRawHaptics.instance;
  }

  // ---- 传输状态 ----
  private pipe: usbManager.USBDevicePipe | null = null;
  private outEndpoint: usbManager.USBEndpoint | null = null;
  private interfaceId: number = -1;
  private attached: boolean = false;
  private claiming: boolean = false;
  private attachTimer: number = -1;
  private pendingDevice: usbManager.USBDevice | null = null;
  private pendingPipe: usbManager.USBDevicePipe | null = null;
  private pendingCandidate: RawEndpointCandidate | null = null;
  private consecutiveFailures: number = 0;
  private lastFrameLogMs: number = 0;
  /** 最近一次成功写出帧的时间戳；0 表示从未成功 */
  private lastSuccessMs: number = 0;
  /** 懒握手：挂载阶段不发任何控制传输，等第一帧 raw 数据到达时再启用声道，
   * 避免打扰 0x0E 保底链路（设备可能在设置类命令后停止播放帧）。 */
  private handshakeDone: boolean = false;
  /** v57: 对照实验期间 raw 静默日志节流（每 5 秒最多一条，用于确认开关真的生效） */
  private rawOffLogMs: number = 0;

  // ---- 用户设置 ----
  private enabled: boolean = true;
  /** 振幅增益：native 已重采样，这里按用户震动强度等比缩放再重算校验和 */
  private gain: number = 1.0;

  // ---- 帧队列（已拷贝，避免引用 native 释放的 buffer） ----
  private queue: Uint8Array[] = [];
  private sending: boolean = false;

  // ---- v28: 端点扫描 ----
  private candidates: RawEndpointCandidate[] = [];
  private activeIndex: number = 0;
  private handshakeOkCount: number = 0;
  private sentFrames: number = 0;
  /**
   * v29: 诊断/试音模式。此时 raw 仍会真实发帧供实验，但 isActive 恒为 false，
   * 绝不抢占 0x0E 保底 —— v28 的教训是「bulkTransfer 写成功 != 设备在播放」，
   * 靠写成功就让路会把已验证可用的震动链路掐掉。
   */
  private diagnosticMode: boolean = false;
  /**
   * v35: 官方编码器实时直灌激活标志。收到第一帧合法 0x51 帧即点亮，
   * 此后 isActive 据此让 0x0E 合成让路（无需 SET_REPORT 握手，
   * 因鸿蒙 USB 栈拒绝 SET_REPORT 且设备默认已启用执行器，与貔貅同款行为）。
   * 编码器停发超过 RAW_ACTIVE_WINDOW_MS 后自然过期，0x0E 自动接管作保底。
   */
  private liveEncoderArmed: boolean = false;
  /** v29: 试音用。关掉后首帧不发任何控制传输，用于验证「无握手直发」 */
  private handshakeEnabled: boolean = true;
  /** v31: 帧格式（默认 Denise 0x51；55AA 仅保留作对照） */
  private frameFormat: number = RAW_FORMAT_DENISE_51;
  /** v32: 诊断用 —— 貔貅式「每帧重开 USB 管道」模式 */
  private reacquireMode: boolean = false;
  private immediateBusy: boolean = false;
  private immediateOkFrames: number = 0;
  private immediateFailFrames: number = 0;
  /** v32: 保存设备对象，供每帧重新 getDevices / connectDevice 使用 */
  private device: usbManager.USBDevice | null = null;

  /** raw 触感接口是否已挂载（诊断用；不等于可以震动） */
  get isAttached(): boolean {
    return this.attached;
  }

  /**
   * 是否「确实在成功发帧」——0x0E 驱动据此让路。
   * v29 收紧为三重条件：已挂载 + 握手确实成功（PeaSyo: 握手失败就不启动 sender）
   * + 最近 RAW_ACTIVE_WINDOW_MS 内真实写出过帧。
   * 鸿蒙实测所有 HID SET_REPORT 都返回 -1，因此 raw 正常不会激活，
   * 震动始终由已验证可用的 0x0E 保底承担，不会再出现「写成功但不震还挡路」。
   */
  get isActive(): boolean {
    const recentlyOk = this.lastSuccessMs > 0 &&
      (Date.now() - this.lastSuccessMs) < RAW_ACTIVE_WINDOW_MS;
    // v35: 官方编码器直灌——只要已点亮且最近确实在发帧，即让 0x0E 让路，
    // 不依赖 SET_REPORT 握手（鸿蒙上全失败且会使设备停播）。
    if (LIVE_ENCODER_ENABLED && this.liveEncoderArmed) {
      return this.attached && this.enabled && !this.diagnosticMode && recentlyOk;
    }
    // 旧 55AA 路径：仍要求 SET_REPORT 握手成功（鸿蒙上恒失败，故不会激活）。
    return this.attached && this.enabled && !this.diagnosticMode &&
      this.handshakeOkCount >= RAW_HANDSHAKE_MIN_OK && recentlyOk;
  }

  /** v29: 试音期间开启，raw 发帧但不抢占 0x0E */
  setDiagnosticMode(on: boolean): void {
    this.diagnosticMode = on;
    Logger.info(TAG, `raw 诊断模式=${on}（${on ? '发帧但不让路' : '按握手结果决定是否让路'}）`);
  }

  /** v29: 试音用，控制是否执行 PeaSyo 握手 */
  setHandshakeEnabled(on: boolean): void {
    this.handshakeEnabled = on;
    this.handshakeDone = false;
    this.handshakeOkCount = 0;
  }

  /** v31: 切换帧格式；Denise 帧不能按 24 个 PCM 样本缩放 / 重算 XOR 校验 */
  setFrameFormat(format: number): void {
    this.frameFormat = format === RAW_FORMAT_LEGACY_55AA ?
      RAW_FORMAT_LEGACY_55AA : RAW_FORMAT_DENISE_51;
    Logger.info(TAG, `raw 帧格式=${this.frameFormat === RAW_FORMAT_DENISE_51 ? 'Denise 0x51' : '55AA 旧版'}`);
  }

  /**
   * v32: 诊断试音开关：开启后每帧重开 USB 管道（貔貅鸿蒙版实测做法）。
   * v33 已强制禁用：21:46 实测该路径 force claim + closePipe 会撤销 0x0E 驱动
   * 的接口声明，使 #3/#4 段所有 interrupt-OUT 与 SET_REPORT 均返回 -1，
   * 必须重新插拔手柄才恢复。代码保留供将来在有官方编码器库时参考，
   * 但任何调用都不会再启用，确保 0x0E 保底链路始终可用。
   */
  setReacquireMode(on: boolean): void {
    this.reacquireMode = false;
    this.immediateBusy = false;
    this.immediateOkFrames = 0;
    this.immediateFailFrames = 0;
    Logger.info(TAG, `raw 每帧重开模式=false（v33 已禁用实验，请求值=${on}）`);
  }

  get immediateSentFrames(): number { return this.immediateOkFrames; }

  /**
   * v32: 严格复刻貔貅鸿蒙版 RazerHapticSink 的单帧发送序列。
   * 21:12:25 DEBUG 日志每 16~20ms 一轮：
   *   ReleaseInterface(iface3/4) → ClosePipe → GetDevices → HasRight → OpenDevice
   *   → ClaimInterface(iface3)=0 → ClaimInterface(iface4)=0 → bulkTransfer(64B)=64
   * v23–v31 一直是「一次 connectDevice 长期复用 + 20ms 塞 3 帧 + 队列积压」，
   * 虽然 bulkTransfer 返回 64 但设备不播；这是最后一个没复刻的差异。
   * 关闭重开模式时走原有队列路径，用作对照段。
   */
  sendDeniseFrameImmediate(frame: Uint8Array): void {
    if (!this.attached || !this.diagnosticMode || frame.length !== RAW_FRAME_BYTES) return;
    if (!this.reacquireMode) {
      this.onRawHapticFrames(frame);
      return;
    }
    if (this.immediateBusy) return;  // 不排队：宁可丢帧也要保住 20ms 节奏
    this.immediateBusy = true;
    this.sendReacquiredFrame(frame);
  }

  private async sendReacquiredFrame(frame: Uint8Array): Promise<void> {
    const saved = this.device;
    let p: usbManager.USBDevicePipe | null = null;
    let ok = false;
    try {
      if (saved === null) return;
      const devs = usbManager.getDevices();
      let dev: usbManager.USBDevice = saved;
      if (devs !== undefined) {
        for (let i = 0; i < devs.length; i++) {
          const d = devs[i];
          if (d !== undefined && d.vendorId === RAZER_VENDOR_ID &&
            d.productId === saved.productId) {
            dev = d;
            break;
          }
        }
      }
      if (!usbManager.hasRight(dev.name)) {
        this.immediateFailFrames++;
        return;
      }
      p = usbManager.connectDevice(dev);
      let ifaceSmall: usbManager.USBInterface | null = null;
      let ifaceHaptic: usbManager.USBInterface | null = null;
      for (let i = 0; i < this.candidates.length; i++) {
        const c = this.candidates[i];
        if (c === undefined) continue;
        if (c.ep.maxPacketSize >= RAW_FRAME_BYTES) ifaceHaptic = c.iface;
        else ifaceSmall = c.iface;
      }
      if (ifaceSmall !== null) usbManager.claimInterface(p, ifaceSmall, true);
      if (ifaceHaptic !== null) usbManager.claimInterface(p, ifaceHaptic, true);
      const ep = this.outEndpoint;
      if (ep !== null) {
        const written = await usbManager.bulkTransfer(p, ep, frame, RAW_OUT_TIMEOUT_MS);
        ok = typeof written === 'number' && written >= RAW_FRAME_BYTES;
      }
      if (ifaceSmall !== null) usbManager.releaseInterface(p, ifaceSmall);
      if (ifaceHaptic !== null) usbManager.releaseInterface(p, ifaceHaptic);
    } catch (err) {
      if (this.immediateFailFrames < 3) {
        Logger.warn(TAG, `每帧重开发送异常: ${err}`);
      }
    } finally {
      if (p !== null) {
        try { usbManager.closePipe(p); } catch (_) { /* 管道已关闭 */ }
      }
      this.immediateBusy = false;
      if (ok) {
        this.immediateOkFrames++;
        this.sentFrames++;
        this.lastSuccessMs = Date.now();
      } else {
        this.immediateFailFrames++;
      }
      const nowMs = Date.now();
      if (nowMs - this.lastFrameLogMs >= 1000) {
        this.lastFrameLogMs = nowMs;
        Logger.info(TAG, `貔貅序列发帧: ok=${this.immediateOkFrames} fail=${this.immediateFailFrames} ` +
          `端点=${this.scanLabel}`);
      }
    }
  }

  /** v27: 试音自检用的可读状态，直接显示在手柄测试页上 */
  get diagnostics(): string {
    const ageMs = this.lastSuccessMs > 0 ? Date.now() - this.lastSuccessMs : -1;
    const ep = this.outEndpoint;
    return `attached=${this.attached} active=${this.isActive} ` +
      `iface=${this.interfaceId} ` +
      `out=0x${ep !== null ? ep.address.toString(16) : '--'}/${ep !== null ? ep.maxPacketSize : 0}B ` +
      `queue=${this.queue.length} fails=${this.consecutiveFailures} ` +
      `握手=${this.handshakeOkCount}/4 已发=${this.sentFrames} lastOk=${ageMs}ms ` +
      `格式=${this.frameFormat === RAW_FORMAT_DENISE_51 ? '0x51' : '55AA'}`;
  }

  // ==================== 设备识别与挂载 ====================

  // ==================== 端点扫描（v28） ====================

  get candidateCount(): number { return this.candidates.length; }

  get activeCandidateIndex(): number { return this.activeIndex; }

  /** 最近一次握手成功的命令条数（0 表示控制传输全失败，设备可能默认已启用） */
  get handshakeOk(): number { return this.handshakeOkCount; }

  get sentFrameCount(): number { return this.sentFrames; }

  /** 当前正在试的端点描述，显示在测试页 */
  get scanLabel(): string {
    const c = this.candidates[this.activeIndex];
    return c !== undefined ? c.label : '无端点';
  }

  /**
   * 切换到第 index 个候选端点：重建 endpoint、清队列、重置握手并尝试声明接口。
   * 每次切换都会重新走一遍懒握手（首帧到达时），便于逐端点验证。
   */
  selectCandidate(index: number): boolean {
    if (!this.attached || this.pipe === null) return false;
    const c = this.candidates[index];
    if (c === undefined) return false;
    this.activeIndex = index;
    this.outEndpoint = RazerKishiRawHaptics.buildEndpoint(c.ep, c.iface.id);
    this.interfaceId = c.iface.id;
    this.queue = [];
    this.consecutiveFailures = 0;
    this.lastSuccessMs = 0;
    this.handshakeDone = false;
    this.handshakeOkCount = 0;
    // v31: 不在这里 claim —— claim 是要单独验证的变量（貔貅会 claim，
    // 而 v24 的 0x0E 保底依赖内核 HID 驱动）。由 claimAllForDiagnosis() 显式控制。
    Logger.info(TAG, `扫描切换 #${index + 1}/${this.candidates.length}: ${c.label}（未 claim）`);
    return true;
  }

  /** v31: 固定选貔貅实测的触感端点（HID + mps>=64 的 INTERRUPT OUT，即 OUT 0x05/64B） */
  selectHapticEndpoint(): boolean {
    for (let i = 0; i < this.candidates.length; i++) {
      const c = this.candidates[i];
      if (c !== undefined && c.iface.clazz === 0x03 && c.ep.maxPacketSize >= RAW_FRAME_BYTES) {
        return this.selectCandidate(i);
      }
    }
    return false;
  }

  /**
   * v31: 复刻貔貅鸿蒙版挂载行为 —— transportProbe 里 iface3(Razer Protocol2.5)
   * 与 iface4(Razer HD Haptics Specifications) 都是 claimResult=0。
   * 仅在诊断试音中调用，串流时不碰，避免踢掉内核 HID 驱动影响 0x0E 保底。
   */
  claimAllForDiagnosis(): void {
    if (this.pipe === null) return;
    for (let i = 0; i < this.candidates.length; i++) {
      const c = this.candidates[i];
      if (c === undefined) continue;
      try {
        const ret = usbManager.claimInterface(this.pipe, c.iface, true);
        Logger.info(TAG, `诊断 claim idx${c.ifaceIndex} ${c.label}: ret=${ret}`);
      } catch (err) {
        Logger.warn(TAG, `诊断 claim idx${c.ifaceIndex} 异常: ${err}`);
      }
    }
  }

  /** 立刻丢弃积压帧（试音结束/切换端点/震动关闭时调用，避免过期波形残留） */
  clearQueue(): void {
    this.queue = [];
  }

  /**
   * 尝试挂载 raw-PCM 触感端点。返回 true 只代表「已排队延迟挂载」，
   * 不保证最终可用；0x0E 保底始终由 UsbDriverService 独立挂载。
   */
  tryAttach(device: usbManager.USBDevice, pipe: usbManager.USBDevicePipe): boolean {
    if (this.attached || this.claiming || this.attachTimer !== -1) return this.attached;
    if (device.vendorId !== RAZER_VENDOR_ID || !device.configs) return false;
    // XInput 模式（PID 0x0037）没有雷蛇 HID 触感接口
    if (device.productId === 0x0037) {
      Logger.info(TAG, 'XInput 模式无 raw-PCM 触感接口，跳过');
      return false;
    }
    const list = this.collectCandidates(device);
    if (list.length === 0) {
      Logger.warn(TAG, '未找到任何 INTERRUPT OUT 端点，raw 路径不可用');
      return false;
    }
    this.candidates = list;
    this.activeIndex = this.pickDefaultIndex(list);
    const candidate = list[this.activeIndex];
    Logger.info(TAG, `发现 ${list.length} 个 INTERRUPT OUT 候选: ` +
      list.map((c: RawEndpointCandidate): string => c.label).join(' | '));
    this.pendingDevice = device;
    this.pendingPipe = pipe;
    this.pendingCandidate = candidate;
    this.claiming = true;
    Logger.info(TAG,
      `已定位 raw-PCM 触感端点 #${this.activeIndex + 1}/${list.length}: ${candidate.label}; ` +
      `${ATTACH_DELAY_MS}ms 后就绪`);
    this.attachTimer = setTimeout((): void => {
      this.attachTimer = -1;
      this.finishAttach();
    }, ATTACH_DELAY_MS);
    return true;
  }

  /**
   * v28: 收集设备上所有 INTERRUPT OUT 端点，按 config.interfaces 枚举顺序编号。
   * PeaSyo 用的是 Android 的 getInterface(3)（枚举索引），鸿蒙拓扑未必一致；
   * 逐个实测比按 maxPacketSize 猜更可靠，因此全部保留为扫描候选。
   */
  private collectCandidates(device: usbManager.USBDevice): RawEndpointCandidate[] {
    const out: RawEndpointCandidate[] = [];
    for (let ci = 0; ci < device.configs.length; ci++) {
      const config = device.configs[ci];
      if (!config.interfaces) continue;
      for (let ii = 0; ii < config.interfaces.length; ii++) {
        const iface = config.interfaces[ii];
        if (!iface.endpoints) continue;
        const isHid: boolean = iface.clazz === 0x03;
        for (let ei = 0; ei < iface.endpoints.length; ei++) {
          const ep = iface.endpoints[ei];
          const xfer = ep.attributes & 0x03;
          const isInterrupt: boolean =
            xfer === INTERRUPT_TRANSFER_TYPE || ep.type === INTERRUPT_TRANSFER_TYPE;
          const isOut: boolean = (ep.direction & 0x80) === 0;
          if (!isInterrupt || !isOut) continue;
          const label = `iface=${iface.id}(idx${ii}) ` +
            `OUT=0x${ep.address.toString(16)}/${ep.maxPacketSize}B HID=${isHid ? 1 : 0}`;
          out.push({ iface: iface, ep: ep, ifaceIndex: ii, label: label });
        }
      }
    }
    return out;
  }

  /** 默认仍选 HID + >=64B（v24 行为），扫描时会遍历全部候选 */
  private pickDefaultIndex(list: RawEndpointCandidate[]): number {
    for (let i = 0; i < list.length; i++) {
      const c = list[i];
      if (c.iface.clazz === 0x03 && c.ep.maxPacketSize >= RAW_FRAME_BYTES) return i;
    }
    return 0;
  }

  private async finishAttach(): Promise<void> {
    const device = this.pendingDevice;
    const pipe = this.pendingPipe;
    const candidate = this.pendingCandidate;
    this.pendingDevice = null;
    this.pendingPipe = null;
    this.pendingCandidate = null;
    this.claiming = false;
    if (device === null || pipe === null || candidate === null) return;
    try {
      // 与 0x0E 一致：不独占 claim，直接写端点（claim force 会踢掉内核 HID 驱动）
      this.pipe = pipe;
      this.device = device;  // v32: 供每帧重开模式重新 connectDevice
      this.interfaceId = candidate.iface.id;
      this.outEndpoint = RazerKishiRawHaptics.buildEndpoint(candidate.ep, candidate.iface.id);
      this.attached = true;
      this.consecutiveFailures = 0;
      this.lastSuccessMs = 0;
      this.handshakeDone = false;
      this.handshakeOkCount = 0;
      this.sentFrames = 0;
      Logger.info(TAG,
        `raw-PCM 端点已就绪 #${this.activeIndex + 1}/${this.candidates.length}: ${candidate.label}; ` +
        `首帧到达时才握手 VID=0x${device.vendorId.toString(16)} PID=0x${device.productId.toString(16)}`);
    } catch (err) {
      Logger.warn(TAG, '挂载 raw-PCM 端点异常:', err);
    }
  }

  /**
   * v29: 严格复刻 PeaSyo 的 startHaptics() —— 只发 4 条 Feature report(0x0300)，
   * wIndex 用目标接口 id，不做任何变体尝试。
   *
   * v28 为「广撒网」对每个端点发了 20 条 SET_REPORT（0x0300/0x0202/0x0200 ×
   * 两种 wIndex），鸿蒙上全部 ret=-1；而 0x0E 驱动注释明确记着「查询/设置类
   * 命令会让设备停止播放帧，直到重新插拔」。这些无效命令很可能把设备弄进了
   * 不播放状态，导致整轮扫描全静音。现在回到最少、最可信的 4 条。
   *
   * 关键语义：PeaSyo 是「sendHapticStateControl 失败就 return false，不启动 sender」。
   * 我们也据 handshakeOkCount 决定 isActive，握手全失败时绝不抢占 0x0E。
   */
  private async startHaptics(): Promise<void> {
    // v30: 非诊断模式绝不发控制传输 —— 设备上这些命令「会停止播放帧直到重新插拔」。
    const liveAllowed = LIVE_ENCODER_ENABLED && this.liveEncoderArmed;
    if (!RAW_PCM_ENABLED && !this.diagnosticMode && !liveAllowed) {
      this.handshakeOkCount = 0;
      return;
    }
    // v35: 官方编码器路径不做任何 SET_REPORT（鸿蒙上全失败且会使设备停播），
    // 设备默认已启用执行器（貔貅同款行为），直接置握手成功让 isActive 通过。
    if (LIVE_ENCODER_ENABLED && this.liveEncoderArmed) {
      this.handshakeOkCount = RAW_HANDSHAKE_MIN_OK;
      Logger.info(TAG, 'live 编码器跳过 SET_REPORT 握手，按设备默认启用状态接管端点');
      return;
    }
    if (!this.handshakeEnabled) {
      this.handshakeOkCount = 0;
      Logger.info(TAG, 'raw 握手已按试音要求关闭，直接直发帧');
      return;
    }
    const cmds: Uint8Array[] = [
      new Uint8Array([0x00, 0x01, 0x01]), // 左声道开
      new Uint8Array([0x00, 0x02, 0x01]), // 右声道开
      new Uint8Array([0x00, 0x01, 0x64]), // 左最高强度
      new Uint8Array([0x00, 0x02, 0x64])  // 右最高强度
    ];
    const c = this.candidates[this.activeIndex];
    if (c !== undefined && this.pipe !== null) {
      try {
        const claimRet = usbManager.claimInterface(this.pipe, c.iface, true);
        Logger.info(TAG, `raw 握手前声明接口 idx${c.ifaceIndex}: ret=${claimRet}`);
      } catch (err) {
        Logger.warn(TAG, `raw 握手前声明接口异常: ${err}`);
      }
    }
    let okCount = 0;
    for (let i = 0; i < cmds.length; i++) {
      if (await this.sendControl(cmds[i], RAW_HANDSHAKE_W_VALUE_FEATURE,
        this.interfaceId, i + 1)) {
        okCount++;
      }
    }
    this.handshakeOkCount = okCount;
    Logger.info(TAG, `raw 握手完成: ${okCount}/${cmds.length} 条成功` +
      (okCount >= RAW_HANDSHAKE_MIN_OK ? '；执行器已启用' :
        '；控制传输不被鸿蒙 USB 栈接受，raw 不激活，震动交还 0x0E'));
  }

  private async sendControl(data: Uint8Array, wValue: number, wIndex: number,
    attempt: number): Promise<boolean> {
    if (this.pipe === null) return false;
    const params: usbManager.USBDeviceRequestParams = {
      bmRequestType: RAW_HANDSHAKE_BM_REQUEST_TYPE,
      bRequest: RAW_HANDSHAKE_B_REQUEST,
      wValue: wValue,
      wIndex: wIndex,
      wLength: data.length,
      data: data
    };
    try {
      const ret = await usbManager.usbControlTransfer(this.pipe, params, 1000);
      const ok: boolean = typeof ret === 'number' && ret > 0;
      Logger.info(TAG,
        `raw 握手 #${attempt} wValue=0x${wValue.toString(16)} wIndex=${wIndex} ` +
        `[${Array.from(data).map((b: number): string => b.toString(16)).join(' ')}]: ` +
        `ret=${ret} ok=${ok}`);
      return ok;
    } catch (err) {
      Logger.warn(TAG, `raw 握手 #${attempt} 异常: ${err}`);
      return false;
    }
  }

  // ==================== 帧直发 ====================

  /**
   * 收到主机下发的原始 64B raw 帧（native 已重采样打包，PeaSyo 格式）。
   * 按 64B 拆分、拷贝、按增益缩放并重算校验和后入队，由 drain 顺序直发。
   */
  async onRawHapticFrames(frames: Uint8Array): Promise<void> {
    // v36 修正：入口【不能】要求 liveEncoderArmed 已为真 —— 该标志正是本函数内部
    // 依据首帧 0x51 点亮的，原来的 !liveAllowed 早退会形成死锁（永远进不来 → 永远
    // 点不亮），实测表现为编码器已 setup 成功但 raw 帧一帧没发出去。
    // 现在只要总开关开就放行，由下方逐帧校验帧头 0x51 后才真正点亮并让路。
    if (!RAW_PCM_ENABLED && !this.diagnosticMode && !LIVE_ENCODER_ENABLED) {
      // v57 对照实验：编码器整条关闭 ⇒ raw 帧一帧不发，armed 永不点亮，
      // isActive 恒 false ⇒ 0x0E 合成独占端点、全程工作。
      const nowMs = Date.now();
      if (nowMs - this.rawOffLogMs > 5000) {
        this.rawOffLogMs = nowMs;
        Logger.info(TAG, 'v57 对照实验生效：LIVE_ENCODER_ENABLED=false，raw 帧全部丢弃，0x0E 独占端点');
      }
      return;
    }
    if (!this.attached || !this.enabled || this.pipe === null || this.outEndpoint === null) return;
    if (frames.length < RAW_FRAME_BYTES) return;
    const count = Math.floor(frames.length / RAW_FRAME_BYTES);
    for (let i = 0; i < count; i++) {
      const off = i * RAW_FRAME_BYTES;
      // v52: 帧首不再是 0x51（那是 command 字节，现在由包络承载），
      // 只要收到 Denise 帧就点亮 armed，不再依赖帧首字节。
      if (!this.liveEncoderArmed &&
          LIVE_ENCODER_ENABLED &&
          this.frameFormat === RAW_FORMAT_DENISE_51) {
        this.liveEncoderArmed = true;
        Logger.info(TAG, `live HD 触觉已接管（20B 包络 cmd=0x${RAZER_HD_CMD.toString(16)}` +
          ' 直发 OUT 0x05）：0x0E 合成让路');
      }
      if (this.frameFormat === RAW_FORMAT_DENISE_51) {
        this.queue.push(this.buildHdPacket(frames, off));
      } else {
        this.queue.push(this.buildScaledFrame(frames, off));
      }
    }
    // v28: 队列上限。v27 实测积压 1260 帧（≈4 秒延迟），试音结束 12 秒后
    // 还在发过期波形，且一直把 0x0E 掐着。超出就丢最旧帧，保证实时。
    if (this.queue.length > RAW_QUEUE_MAX) {
      this.queue.splice(0, this.queue.length - RAW_QUEUE_MAX);
    }
    if (!this.sending) {
      this.sending = true;
      this.drain();
    }
  }

  /**
   * v53（固化）: 把 native 送来的 payload（帧首起最多 15B，不足补零）装进 **20B Razer 包络**：
   *   [0x02][0x13][0x00][0x01][cmd][15B 位流]
   * v52 实测（19:59:45~53）三段实验全部成功驱动马达 —— 指令 0x0E/0x51、我方/貔貅 payload
   * 都能震，反证此前十几轮失败的唯一原因就是「64B 裸帧」这个错误的封装。
   */
  private buildHdPacket(src: Uint8Array, offset: number): Uint8Array {
    const pkt = new Uint8Array(RAZER_HD_PACKET_BYTES);
    pkt[0] = 0x02;
    pkt[1] = 4 + RAZER_HD_PAYLOAD_BYTES;  // 0x13 = 19
    pkt[2] = 0x00;
    pkt[3] = 0x01;                        // fragment = 1
    pkt[4] = RAZER_HD_CMD;
    for (let i = 0; i < RAZER_HD_PAYLOAD_BYTES; i++) {
      const idx = offset + i;
      pkt[5 + i] = idx < src.length ? src[idx] : 0;
    }
    return pkt;
  }

  /** 拷贝一帧，按增益缩放 24 个 int16 样本并重算 XOR[2..57] 校验和 */
  private buildScaledFrame(src: Uint8Array, offset: number): Uint8Array {
    const f = new Uint8Array(RAW_FRAME_BYTES);
    f.set(src.subarray(offset, offset + RAW_FRAME_BYTES));
    // v31: Denise 0x51 帧前 15B 是编码 payload，不能按 55AA 的 24 个 PCM 样本
    // 缩放，也不能重算 XOR 校验（该格式根本没有校验字节）。
    if (this.frameFormat === RAW_FORMAT_DENISE_51) return f;
    if (this.gain !== 1.0) {
      for (let s = 0; s < RAW_PAYLOAD_SAMPLES; s++) {
        const p = RAW_HEADER_SIZE + s * 2;
        let v = f[p] | (f[p + 1] << 8);
        if (v >= 0x8000) v -= 0x10000;
        v = Math.max(-32768, Math.min(32767, Math.round(v * this.gain)));
        f[p] = v & 0xFF;
        f[p + 1] = (v >> 8) & 0xFF;
      }
      let cs = 0;
      for (let i = RAW_CHECKSUM_START; i < RAW_HEADER_SIZE + RAW_PAYLOAD_SIZE; i++) cs ^= f[i];
      f[RAW_CHECKSUM_POS] = cs;
    }
    return f;
  }

  /**
   * v29: 严格逐帧 64B（PeaSyo sender 原样），并针对 maxPacketSize < 64 的端点
   * 手动拆包。真机日志证据：iface=3 OUT=0x02/16B 上 bulkTransfer(64B) 一律
   * 返回 -1 —— 鸿蒙 USB 栈不像 Android 内核那样自动把大于 mps 的 interrupt
   * 传输拆成多个 packet。这里改成 4 次 16B 顺序提交，让固件有机会重组。
   */
  private static splitFrame(frame: Uint8Array, mps: number): Uint8Array[] {
    if (mps <= 0 || mps >= RAW_FRAME_BYTES) return [frame];
    const parts: Uint8Array[] = [];
    for (let off = 0; off < RAW_FRAME_BYTES; off += mps) {
      const len = Math.min(mps, RAW_FRAME_BYTES - off);
      const part = new Uint8Array(len);
      for (let i = 0; i < len; i++) part[i] = frame[off + i];
      parts.push(part);
    }
    return parts;
  }

  private async drain(): Promise<void> {
    while (this.queue.length > 0) {
      if (this.pipe === null || this.outEndpoint === null) break;
      const pipe = this.pipe;
      const ep = this.outEndpoint;
      // 首帧到达才握手：普通 rumble / 非 DS5 主机永远不会触发控制传输。
      if (!this.handshakeDone) {
        this.handshakeDone = true;
        await this.startHaptics();
      }
      const frame = this.queue.shift();
      if (frame === undefined) break;
      const chunks = RazerKishiRawHaptics.splitFrame(frame, ep.maxPacketSize);
      try {
        let written = 0;
        let ok = true;
        for (let ci = 0; ci < chunks.length; ci++) {
          const w = await usbManager.bulkTransfer(pipe, ep, chunks[ci], RAW_OUT_TIMEOUT_MS);
          if (!(typeof w === 'number' && w > 0)) {
            ok = false;
            break;
          }
          written += w;
        }
        if (ok) {
          this.consecutiveFailures = 0;
          this.lastSuccessMs = Date.now();
          this.sentFrames++;
          const nowMs = Date.now();
          if (nowMs - this.lastFrameLogMs >= 1000) {
            this.lastFrameLogMs = nowMs;
            Logger.info(TAG,
              `raw 触感帧直发: ${written}B/1帧(${chunks.length}包), 队列余 ${this.queue.length}, ` +
              `端点=${this.scanLabel}, 握手=${this.handshakeOkCount}/4, gain=${this.gain.toFixed(2)}`);
          }
        } else {
          this.onFrameFailure(`bulkTransfer 返回失败 (mps=${ep.maxPacketSize}, 共${chunks.length}包)`);
        }
      } catch (err) {
        this.onFrameFailure(`异常 ${err}`);
      }
      if (this.consecutiveFailures >= RAW_FAILURE_THRESHOLD) {
        this.queue = [];
        break;
      }
    }
    this.sending = false;
  }

  private onFrameFailure(reason: string): void {
    this.consecutiveFailures++;
    if (this.consecutiveFailures <= 3 || this.consecutiveFailures === RAW_FAILURE_THRESHOLD) {
      Logger.warn(TAG,
        `raw 触感帧发送失败(${this.consecutiveFailures}/${RAW_FAILURE_THRESHOLD}): ${reason}`);
    }
    if (this.consecutiveFailures >= RAW_FAILURE_THRESHOLD) {
      // 不再让路：把 lastSuccessMs 清零，0x0E 立刻接管（无需重插拔）
      this.lastSuccessMs = 0;
      Logger.warn(TAG, 'raw 端点连续失败，已交还 0x0E 保底路径');
    }
  }

  // ==================== 设置 ====================

  /** 用户震动强度（0-200）→ 振幅增益。100 为不放大 */
  setStrength(strength: number): void {
    this.gain = Math.max(0, Math.min(2.0, strength / 100.0));
    Logger.info(TAG, `raw 触感增益更新: gain=${this.gain.toFixed(2)} (strength=${strength})`);
  }

  setEnabled(enabled: boolean): void {
    this.enabled = enabled;
  }

  // ==================== 释放 ====================

  /** 设备已拔出或 pipe 即将关闭时调用。不 release 接口（未独占 claim）。 */
  detach(): void {
    if (this.attachTimer !== -1) {
      clearTimeout(this.attachTimer);
      this.attachTimer = -1;
    }
    this.pendingDevice = null;
    this.pendingPipe = null;
    this.pendingCandidate = null;
    this.claiming = false;
    this.queue = [];
    this.sending = false;
    this.handshakeDone = false;
    this.lastSuccessMs = 0;
    this.consecutiveFailures = 0;
    this.pipe = null;
    this.outEndpoint = null;
    this.interfaceId = -1;
    this.attached = false;
    this.candidates = [];
    this.activeIndex = 0;
    this.handshakeOkCount = 0;
    this.sentFrames = 0;
    this.diagnosticMode = false;
    this.handshakeEnabled = true;
    this.frameFormat = RAW_FORMAT_DENISE_51;
    this.device = null;
    this.reacquireMode = false;
    this.immediateBusy = false;
    Logger.info(TAG, 'raw-PCM 触感端点已释放（0x0E 保底不受影响）');
  }

  /** 显式重建端点对象，确保 bulkTransfer 拿到完整的 interfaceId/number/type */
  private static buildEndpoint(ep: usbManager.USBEndpoint, interfaceId: number): usbManager.USBEndpoint {
    const isIn: boolean = (ep.direction & 0x80) !== 0;
    return {
      address: ep.address,
      attributes: INTERRUPT_TRANSFER_TYPE,
      interval: ep.interval,
      maxPacketSize: ep.maxPacketSize,
      direction: (isIn ? 0x80 : 0x00) as usbManager.USBRequestDirection,
      interfaceId: interfaceId,
      number: ep.address,
      type: INTERRUPT_TRANSFER_TYPE
    };
  }
}
