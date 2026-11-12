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
 * Razer HD Haptics (Sensa HD) driver for Razer Kishi Ultra / Kishi V3 Pro.
 *
 * 雷蛇骑仕系列把触感马达放在一个独立的 HID 类接口上（"Razer HD Haptics
 * Specifications"，Usage Page 0x0E），与手柄输入接口分离。系统会把它识别为
 * "无马达手柄"，游戏的 rumble 事件默认无处可去。本驱动直接讲雷蛇的触感流
 * 协议，把 Moonlight 的双马达震动渲染成 30ms 触感帧。
 *
 * 真机拓扑（Kishi Ultra HID 模式，PID 0x071A，2026-11-10 实测）：
 *   接口 0: HID Game Pad        IN 0x81/32B             <- 手柄输入
 *   接口 1: HID Digitizer       IN 0x82/32B             <- 触摸板
 *   接口 2: HID vendor          IN 0x83/16B
 *   接口 3: HID keyboard        OUT 0x02/16B
 *   接口 4: HID Haptics         IN 0x85/64B OUT 0x05/64B <- 触感（本驱动）
 * 触感接口是 HID 类（clazz=0x03）、interrupt 端点，不是厂商自定义类/bulk 端点；
 * 它的报告描述符声明 output report 2（63 字节载荷）与 report 4，正好承载流帧。
 *
 * 协议逆向参考（MIT License, Copyright (c) 2026 Mokomis）:
 *   https://github.com/Mokomis/rumblebridge  (daemon/rumblebridged.c, firmware 2.0.0.0)
 *
 * 协议要点（在 Kishi 固件 2.0.0.0 上实测）：
 *  - 只有流帧（command 0x0E）可以在该接口上发送；查询/设置类命令（mode 0x87、
 *    gain 0x88/0x08）会让设备停止播放帧，直到重新插拔。
 *  - 每一帧设备都会在 IN 端点回答；驱动读取回答作为链路健康信号。
 *  - 空闲时绝不发送：骑仕 15 分钟无输入会自动休眠掉线。持续发保活帧会导致
 *    设备"应答但不再震动"，只能重新插拔恢复。因此停止时只发一帧静音帧，
 *    之后完全静默，设备休眠后由上层重新 attach。
 *  - 帧格式（output report 2）：[2][len][0][fragment=1][0x0E][帧位流]
 *    帧位流 MSB first：7bit 时长(ms)，随后每通道 1bit 有频带 + 4 组
 *    (6bit 幅度, 7bit 频率索引)，1bit 无更多频带，1bit 无瞬态。
 *  - 左通道为重马达、右通道为轻马达；游戏只驱动单马达时镜像到两侧。
 */

import { usbManager } from '@kit.BasicServicesKit';
import { Logger } from '../../common/Logger';
import { Ds5HapticsPcmFrame } from '../streaming/MoonBridge';
import { RazerKishiRawHaptics } from './RazerKishiRawHaptics';

const TAG = 'RazerKishiHaptics';

/** 雷蛇 USB VID */
const RAZER_VENDOR_ID = 0x1532;
/** HID 类 class code：触感接口是 HID 类，不是厂商自定义类(0xFF) */
const USB_CLASS_HID = 0x03;
/** 端点传输类型：3 = interrupt（配置描述符 bmAttributes 低 2 位） */
const INTERRUPT_TRANSFER_TYPE = 3;
/** 挂载延迟：控制器启动后会异步临时 claim/release 各接口读报告描述符，避开该窗口 */
const ATTACH_DELAY_MS = 900;
/** HID SET_REPORT(Output) 兜底通道 */
const SET_REPORT_BM_REQUEST_TYPE = 0x21;  // Host->Device | Class | Interface
const SET_REPORT_B_REQUEST = 0x09;
const SET_REPORT_W_VALUE = 0x0202;        // Output report, report ID 2
/** 触感流帧周期（毫秒）：20ms = 50Hz 时间分辨率，才能表达 HD 触觉的颗粒 */
const FRAME_MS = 20;
/**
 * 有效期按事件类型分级。v15 对所有事件统一 110ms，把 Moonlight 的
 * 「事件式 rumble」（发一次 FFFF 后等发 0）全部截成短促一嗒，强震动
 * 没有持续高幅度，也失去强弱区分。
 */
const HOLD_SUSTAINED_MS = 600;   // 持续型强震动（引擎、冰块摩擦、爆炸余波）
const HOLD_TRANSIENT_MS = 160;   // 瞬态型（枪声、敲击）
const HOLD_WEAK_MS = 150;        // 弱触感流（雨声），停发后快速静音避免拖尾
/** 输出报告长度：5 字节头 + 15 字节帧位流 */
const REPORT_LENGTH = 20;
/** IN 端点回答缓冲区长度 */
const ANSWER_LENGTH = 64;
/** 连续这么多帧失败后认为设备已离开 USB，自动 detach */
const FAILURE_DETACH_THRESHOLD = 10;
/** 输出端点传输超时（毫秒） */
const OUT_TIMEOUT_MS = 100;
/** IN 端点读取超时（毫秒）：应答通常 4ms 内到达；20ms 帧流下不能等太久 */
const IN_TIMEOUT_MS = 6;
/**
 * v67: 帧流看门狗（毫秒）。
 *
 * streamTick 头一行 `if (this.streaming) return;` 是防止慢链路重入的必要保护，
 * 但它没有任何脱身机制：只要某次 `await usbManager.bulkTransfer(...)` 的 Promise
 * 永不 settle（设备侧 stall / 内核 usbfs 不回收 URB），`streaming` 就永久为 true，
 * 之后每个 tick 都在第一行被吞掉 ⇒ **完全静默、零日志、定时器还在空转**。
 * 这与「震动一段时间后完全不震、几分钟后又自己恢复」的现象高度吻合，
 * 因此加阈值兜底。正常一帧的上限约 = OUT 超时 100ms + IN 读 6ms，
 * 1500ms 只有在真卡死时才会触发，正常抖动碰不到。
 */
const STREAM_STUCK_MS = 1500;
/** 链路心跳周期（毫秒）：完全静默时也要能看出究竟断在哪一层 */
const HEARTBEAT_MS = 3000;
/** PCM 断流告警门槛（毫秒）：正常 20ms 一帧，超过这个时长没到就是真断了 */
const PCM_GAP_WARN_MS = 1500;
/**
 * 默认感知增强（%）。参考 PeaSyo 的模式：弱触感信号不砍、抬到可感底。
 * v14 的 110% 加上噪声门让剑星下雨这类细密弱信号完全消失，这里补回来。
 */
const DEFAULT_BOOST_PERCENT = 150;
/**
 * 软膝（0-63）：弱的非零请求抬到这个可感底，而不是像 v14 那样直接归零。
 * 真正的 0 输入仍返回 0，事件之间的绝对静音不会被破坏。
 */
const SOFT_KNEE_LEVEL = 6;
/** 事件包络衰减时间常数（毫秒） */
const ENVELOPE_DECAY_MS = 55;
/**
 * 包络按强度自适应：弱触感流（雨声、脚步）几乎不衰减，保持连续颗粒感；
 * 强瞬态（枪声、碰撞）快速收下去，制造「一下」的干脆感。
 */
const ENVELOPE_SUSTAIN_WEAK = 0.90;
const ENVELOPE_SUSTAIN_STRONG = 0.45;
/** 达到这个幅度（单通道 0-63）视为强事件，包络按强事件处理 */
const ENVELOPE_STRONG_PEAK = 40;
/** 只有足够强的事件才插 10ms 锐利起音短帧，弱触感流不被切成碎片 */
const ATTACK_MIN_SUM = 24;
/** 幅度跃升多少（0-63 之和）才算一次新的触感事件，用于重置包络 */
const NEW_EVENT_DELTA = 8;
/**
 * HD 波形的 4 个频点（每通道一个 band 内的 4 组幅度/频率，同时合成）。
 * 只取 rumblebridge 实测有效的频点：index 5 ≈ 45Hz 低频基底、
 * 10 ≈ 59Hz 主峰（最强）、35 ≈ 132Hz 次峰（清脆细节）。
 * v13 摊到 18/26/30/42 这些 14 以上响应急降的无效索引，才会更糊。
 */
const HD_HEAVY_FREQS: number[] = [5, 10, 10, 35];
const HD_LIGHT_FREQS: number[] = [10, 35, 35, 35];
// v20: 把 DS5 真实频谱的 4 子带权重映射到协议 4 谐波分量。频率索引沿用已知
// 有效区间（索引越大=越高频，见 heavyFreq=10 / lightFreq=35）。重马达偏低频、
// 轻马达偏高频，与各自通道的频段包络一致。索引->Hz 官方未公开，用经验值，
// 实测偏了可在本处微调。
// v22: 频率索引适中偏高（Kishi Ultra 标榜宽带）。
// 子带 [50,180,550,1300]Hz 映射到这些协议频率索引；真实 Hz 由设备 freqMax 决定，未知→经验值。
// 配合 callbacks.cpp 的 PCM_BAND_PREEMP 预加重，让高频段即便在弱响应区也能被感觉到。
const PCM_SPECTRAL_HEAVY: number[] = [5, 16, 34, 60];
const PCM_SPECTRAL_LIGHT: number[] = [10, 26, 44, 80];
/** 高频带逐帧随机调制的深度，制造冰块/沙砾这类不规则颗粒感 */
const TEXTURE_JITTER = 0.40;
/** 瞬态起音帧时长（毫秒）：短帧让敲击/枪声这类瞬态收得干脆 */
const ATTACK_FRAME_MS = 10;
/** 起音帧幅度放大系数，制造锐利的 attack 而不是平顶嗡鸣 */
const ATTACK_GAIN = 1.25;
/** 频率索引默认值（约 30Hz + 2.9Hz/步）。10 ≈ 59Hz 主峰，35 ≈ 132Hz 次峰 */
const DEFAULT_HEAVY_FREQ = 10;
const DEFAULT_LIGHT_FREQ = 35;
/** PCM 波形帧的刷新窗口：主机停发后很快静音，不再猜 600ms 的 hold（v19 收紧到 90ms 让收尾更干脆） */
const PCM_HOLD_MS = 90;
/**
 * v58: 瞬态峰值锁定（解决「连续跳偶尔丢落地」+「落地不够明显」）。
 *
 * 成因（时序竞争，不是灵敏度问题）：
 *   native `BridgeClDs5HapticsPcm` 每 20ms 出一帧并**直接覆盖** ArkTS 的 `this.current`；
 *   `streamTick` 也是独立的 20ms 定时器去读 `this.current`。两个定时器不同源、会漂移，
 *   叠加 `streamTick` 里 `if (this.streaming) return`（上一帧 USB 写未回则整拍跳过），
 *   落地那一帧的峰值有概率**在被打包发送前就被下一帧雨滴覆盖** ⇒ 落地震感丢失。
 *
 * v59 实测修正（20:34 日志，1135 次触发全部是雨滴）：
 *   原来只看 rise≥8，而雨滴自身波动就有 8~19 档 ⇒ 1135 次误触发，
 *   真正的落地（满档 63）只出现 12 次，被淹没在雨滴里，所以既不明显也像"丢"。
 *   实测档位分布非常干净：
 *     雨滴常见 27~48，49~62 **完全为空**，落地直接到 63（共 12 次 ≈ 跳跃次数）。
 *   ⇒ 改用**绝对档位**做主判据（PCM_TRANSIENT_LEVEL=55），上升沿只作辅助（提到 20）。
 *   ⇒ 保持时长 60ms → 120ms，并做线性衰减：落地从 63 渐降回雨滴电平，
 *     形成"一下"的余震感 —— 幅度已到上限无法再加，**只能靠时间维度把冲击做明显**。
 */
const PCM_TRANSIENT_LEVEL = 55;
/** v66: **rise 辅助判据已彻底删除**。v65 日志实证它仍在误触发（72/361 是雨滴），
 *  且会让锁定被雨滴持续刷新 ⇒ 轻马达反复拉满 ⇒ 马达饱和 ⇒ 雨滴真空期。
 *  现在只认绝对档位 55，落地实测直接 63，干净分界。 */
/** 瞬态峰值保持时长（毫秒）：真实落地冲击的余震量级
 *  v65: 120 → 170。v62/v64 实测「落地还是不够」时，幅度(63 顶格)与频谱(4 子带已铺开)
 *  两个维度都已到顶，**唯一还没用过的维度是时间**：120ms 只有 6 拍，
 *  物理马达刚起速就回落，主观上"像抖了一下"而不是"砸了一下"。
 *  170ms ≈ 8~9 拍，覆盖真实落地的余震长度（真人落地 100~200ms）。
 *  只在冲击帧生效（PCM_TRANSIENT_LEVEL=55），雨滴帧不会进入此分支。 */
const PCM_PEAK_HOLD_MS = 170;

/** 双马达幅度（0-65535，与 Moonlight rumble 协议一致） */
interface MotorPair {
  low: number;
  high: number;
}

/** 触感接口上挑出的 interrupt 输出/输入端点 */
interface HapticsEndpoints {
  output: usbManager.USBEndpoint | null;
  input: usbManager.USBEndpoint | null;
}

/**
 * 雷蛇骑仕触感驱动。
 *
 * 生命周期：UsbDriverService 在识别到雷蛇设备后调用 tryAttach()，
 * GamepadVibrationService 在每次渲染混合输出后调用 render()。
 * 设备拔出或链路失效时自动 detach，插回后由 UsbDriverService 重新 attach。
 */
export class RazerKishiHaptics {
  private static instance: RazerKishiHaptics | null = null;

  static getInstance(): RazerKishiHaptics {
    if (RazerKishiHaptics.instance === null) {
      RazerKishiHaptics.instance = new RazerKishiHaptics();
    }
    return RazerKishiHaptics.instance;
  }

  // ---- 传输状态 ----
  private pipe: usbManager.USBDevicePipe | null = null;
  private usbInterface: usbManager.USBInterface | null = null;
  private outEndpoint: usbManager.USBEndpoint | null = null;
  private inEndpoint: usbManager.USBEndpoint | null = null;
  private deviceKey: string = '';
  private attached: boolean = false;
  private claiming: boolean = false;
  /** 是否已 claim 触感接口：优先不声明直接写，失败后才声明，避免影响手柄输入接口 */
  private claimed: boolean = false;
  private interfaceId: number = -1;
  /** 延迟挂载：先定位接口，等控制器描述符校准结束后再正式占用 */
  private attachTimer: number = -1;
  private pendingDevice: usbManager.USBDevice | null = null;
  private pendingPipe: usbManager.USBDevicePipe | null = null;
  private pendingInterface: usbManager.USBInterface | null = null;
  private pendingEndpoints: HapticsEndpoints | null = null;
  /** 写入通道诊断计数器（限流用） */
  private renderLogCount: number = 0;
  private frameLogCount: number = 0;
  /** 帧流日志节流（原先按固定次数，长会话后半段完全没有日志可查） */
  private lastRenderLogMs: number = 0;
  private lastFrameLogMs: number = 0;

  // ---- 触感流状态 ----
  private frameTimer: number = -1;
  /** 当前触感事件的峰值幅度（0-63）；每帧由包络缩放后再发出 */
  private current: MotorPair = { low: 0, high: 0 };
  private playing: boolean = false;
  private effectUntilMs: number = 0;
  /** 当前事件起点，用于计算 attack-decay 包络 */
  private eventStartMs: number = 0;
  /** 挂载自检期间跳过包络，保持稳定的双通道自检震动 */
  private probeMode: boolean = false;
  private consecutiveFailures: number = 0;
  private streaming: boolean = false;

  // ---- v67 诊断：定位「震动一段时间后完全静默」 ----
  /** 本次 streaming 置位的时间戳；0 表示未置位 */
  private streamingSinceMs: number = 0;
  /** 最近一次收到非静音 PCM 帧的时间戳（0 = 从未收到） */
  private lastPcmRecvMs: number = 0;
  /** 最近一次 OUT 写成功的时间戳（0 = 从未成功） */
  private lastSendOkMs: number = 0;
  /** 链路心跳定时器；attach 后常驻，保证完全静默时也有日志可查 */
  private heartbeatTimer: number = -1;

  // ---- 用户设置 ----
  /** 总强度 0-100 */
  private strength: number = 100;
  /** 响应曲线 ×100：100 为线性，越低越抬升弱效果（让雨声/脚步可感） */
  private curve100: number = 65;
  /** 高保真音色：tone=高频占比、attack=瞬态强度、texture=颗粒度，来自 DS5 IR / 本地音频触觉分析 */
  private tone: number = 0;
  private attack: number = 0;
  private texture: number = 0;
  /** 持续型（引擎/环境音）vs 瞬态型（枪声/敲击）；纯 rumble 默认持续 */
  private sustained: boolean = true;
  /** 当前事件是否按持续型处理 */
  private sustainedEvent: boolean = true;
  /** 确定性伪随机序列，用于高频带逐帧颗粒调制 */
  private frameSerial: number = 1;
  /** 下一帧使用短时长强起音帧 */
  private pendingAttack: boolean = false;
  // v58: 瞬态峰值锁定状态（见 PCM_TRANSIENT_LEVEL 注释）
  private pcmHoldLow: number = 0;
  private pcmHoldHigh: number = 0;
  private pcmHoldUntilMs: number = 0;
  private pcmHoldStartMs: number = 0;
  /** 统计：被峰值锁定救回来的瞬态次数（日志用于确认修复真的在起作用） */
  private pcmHoldRescued: number = 0;
  /** 主机原始触觉波形流是否接管了帧流 */
  private pcmMode: boolean = false;
  /** 波形帧诊断日志节流 */
  private lastPcmLogMs: number = 0;
  /** 额外感知增益（%），由 GamepadVibrationService 按用户震动强度联动 */
  private boostPercent: number = DEFAULT_BOOST_PERCENT;
  /** 当前事件是否为强触感（决定包络与起音策略） */
  private strongEvent: boolean = false;
  /** 重/轻马达各自缩放 0-100 */
  private heavyScale: number = 100;
  private lightScale: number = 100;
  /** 帧频率索引（7bit，0-127） */
  private heavyFreq: number = DEFAULT_HEAVY_FREQ;
  private lightFreq: number = DEFAULT_LIGHT_FREQ;
  // v20: 最近一帧的 4 子带频谱权重（0..1），由 DS5 PCM 真实波形算出。
  private bandAmp: number[] = [0, 0, 0, 0];
  // v20: 仅当本帧来自真实 DS5 PCM 频谱时才启用 PCM_SPECTRAL 谐波映射；
  // 普通 rumble / 旧 HD 仍走各自频率（与 rumblebridge 参考实现对齐）。
  private pcmSpectral: boolean = false;
  /** 是否启用（false 时 render 直接忽略） */
  private enabled: boolean = true;

  // ==================== 设备识别与挂载 ====================

  /**
   * 判断设备是否可能是带触感接口的雷蛇骑仕。
   * 不硬编码 PID（Kishi Ultra 0x071A、Kishi V3 Pro HID 0x0724 各不相同），
   * 以「VID + HID 类双向 interrupt 接口」的接口形态识别更稳健。
   */
  static isRazerHapticsDevice(device: usbManager.USBDevice): boolean {
    if (device.vendorId !== RAZER_VENDOR_ID) return false;
    if (!device.configs) return false;
    let best = -1000;
    for (let ci = 0; ci < device.configs.length; ci++) {
      const config = device.configs[ci];
      if (!config.interfaces) continue;
      for (let ii = 0; ii < config.interfaces.length; ii++) {
        const iface = config.interfaces[ii];
        const score = RazerKishiHaptics.scoreHapticsInterface(iface);
        Logger.info(TAG,
          `候选接口 id=${iface.id}: class=0x${iface.clazz.toString(16)} sub=${iface.subClass} ` +
          `proto=${iface.protocol} eps=${iface.endpoints ? iface.endpoints.length : 0} 触感评分=${score}`);
        if (score > best) best = score;
      }
    }
    return best > 0;
  }

  /**
   * 触感接口特征：HID 类，且同时具备 interrupt IN 与 OUT 端点。
   * 真机上只有接口 4 同时满足（其它 HID 接口要么只有 IN、要么只有 OUT），
   * 因此无需 PID 白名单即可精确定位。
   */
  private static scoreHapticsInterface(iface: usbManager.USBInterface): number {
    if (iface.clazz !== USB_CLASS_HID) return -1;
    const eps = iface.endpoints;
    if (!eps || eps.length === 0) return -1;
    let inMax = 0;
    let outMax = 0;
    for (const ep of eps) {
      const xfer = ep.attributes & 0x03;
      if (xfer !== INTERRUPT_TRANSFER_TYPE && ep.type !== INTERRUPT_TRANSFER_TYPE) continue;
      if ((ep.direction & 0x80) !== 0) {
        inMax = Math.max(inMax, ep.maxPacketSize);
      } else {
        outMax = Math.max(outMax, ep.maxPacketSize);
      }
    }
    if (inMax === 0 || outMax < REPORT_LENGTH) return -1;
    let score = 10;
    if (outMax >= 64 && inMax >= 64) score += 5;
    // "Razer HD Haptics Specifications" 接口不做 boot 声明
    if (iface.subClass === 0 && iface.protocol === 0) score += 2;
    return score;
  }

  /** 从接口中挑出 interrupt OUT / IN 端点（按最大包长优先） */
  private static pickEndpoints(iface: usbManager.USBInterface): HapticsEndpoints {
    let out: usbManager.USBEndpoint | null = null;
    let inn: usbManager.USBEndpoint | null = null;
    const eps = iface.endpoints;
    if (!eps) return { output: null, input: null };
    for (const ep of eps) {
      const xfer = ep.attributes & 0x03;
      if (xfer !== INTERRUPT_TRANSFER_TYPE && ep.type !== INTERRUPT_TRANSFER_TYPE) continue;
      if ((ep.direction & 0x80) !== 0) {
        if (inn === null || ep.maxPacketSize > inn.maxPacketSize) inn = ep;
      } else {
        if (out === null || ep.maxPacketSize > out.maxPacketSize) out = ep;
      }
    }
    return { output: out, input: inn };
  }

  /**
   * OHOS 的 bulkTransfer 依赖端点对象里的 interfaceId / number / type。
   * 设备枚举返回的 USBEndpoint 未必带齐这些字段，这里显式重建，
   * 与已验证可用的手柄输入轮询保持一致。
   */
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

  /**
   * 尝试在同一 pipe 上占用触感接口（不关闭 pipe——它归控制器所有）。
   *
   * 控制器 start() 返回后，NativeHidController 仍会异步临时 claim/release 各接口
   * 去读 HID 报告描述符；若此时抢占接口 4，会被随后的 release 一起解掉。
   * 因此先定位接口，延迟到校准窗口结束后再正式 claim。
   */
  tryAttach(device: usbManager.USBDevice, pipe: usbManager.USBDevicePipe): boolean {
    // v68 关键修正：已经"挂着"但这次拿到的是**另一条 pipe 或另一台设备实例**，
    // 说明旧 pipe 已经不归我们了（控制器因 IO 错误重启 / 设备重新枚举 / USB 复位）。
    // 旧代码在这里直接 `return this.attached`（true）什么都不做，UsbDriverService
    // 还会照着返回值打印"已排队（延迟占用）"，看上去一切正常；实际上之后每一帧
    // 都在往一条已关闭的管道写，返回 -1 攒够 10 次才 detach ⇒ 干等下一次重新枚举。
    // 实测真空 2 分 47 秒（22:12:55 释放 → 22:15:42 才重新挂上）。
    // 现在：pipe 变了或设备实例变了，一律先 detach 再用新 pipe 重挂。
    const incomingKey = `${device.vendorId}:${device.productId}:${device.busNum}:${device.devAddress}`;
    if (this.attached && (this.pipe !== pipe || this.deviceKey !== incomingKey)) {
      Logger.warn(TAG, `触感接口收到新的 pipe/设备实例（旧 ${this.deviceKey} → 新 ${incomingKey}），` +
        '旧管道已失效，强制重新挂载');
      this.detach();
    }
    if (this.attached || this.claiming || this.attachTimer !== -1) return this.attached;
    if (device.vendorId !== RAZER_VENDOR_ID || !device.configs) return false;
    // XInput 模式（PID 0x0037）没有雷蛇 HID Haptics 接口，跳过挂载以免误 claim
    if (device.productId === 0x0037) {
      Logger.info(TAG, 'XInput 模式无 HID 触感接口，跳过触感挂载');
      return false;
    }

    let bestIface: usbManager.USBInterface | null = null;
    let bestScore = -1000;
    for (let ci = 0; ci < device.configs.length; ci++) {
      const config = device.configs[ci];
      if (!config.interfaces) continue;
      for (let ii = 0; ii < config.interfaces.length; ii++) {
        const iface = config.interfaces[ii];
        const score = RazerKishiHaptics.scoreHapticsInterface(iface);
        if (score > bestScore) {
          bestScore = score;
          bestIface = iface;
        }
      }
    }
    if (bestIface === null || bestScore <= 0) {
      Logger.warn(TAG, '未找到 HID 触感接口（需 HID 类 + interrupt IN/OUT 双向端点）');
      return false;
    }
    const eps = RazerKishiHaptics.pickEndpoints(bestIface);
    const outEp = eps.output;
    const inEp = eps.input;
    if (outEp === null || inEp === null) {
      Logger.warn(TAG, '触感接口端点不完整');
      return false;
    }

    this.pendingDevice = device;
    this.pendingPipe = pipe;
    this.pendingInterface = bestIface;
    this.pendingEndpoints = eps;
    this.claiming = true;
    Logger.info(TAG,
      `已定位触感接口 id=${bestIface.id} (评分=${bestScore}), ` +
      `OUT=0x${outEp.address.toString(16)}/${outEp.maxPacketSize}B, ` +
      `IN=0x${inEp.address.toString(16)}/${inEp.maxPacketSize}B; ` +
      `${ATTACH_DELAY_MS}ms 后占用（避开描述符校准）`);
    this.attachTimer = setTimeout((): void => {
      this.attachTimer = -1;
      this.finishAttach();
    }, ATTACH_DELAY_MS);
    return true;
  }

  /** 延迟窗口结束后真正声明接口并启动自检帧 */
  private finishAttach(): void {
    const device = this.pendingDevice;
    const pipe = this.pendingPipe;
    const iface = this.pendingInterface;
    const eps = this.pendingEndpoints;
    this.pendingDevice = null;
    this.pendingPipe = null;
    this.pendingInterface = null;
    this.pendingEndpoints = null;
    this.claiming = false;
    if (device === null || pipe === null || iface === null || eps === null) return;
    const outEp = eps.output;
    const inEp = eps.input;
    if (outEp === null || inEp === null) return;
    try {
      this.pipe = pipe;
      this.usbInterface = iface;
      this.interfaceId = iface.id;
      this.outEndpoint = RazerKishiHaptics.buildEndpoint(outEp, iface.id);
      this.inEndpoint = RazerKishiHaptics.buildEndpoint(inEp, iface.id);
      this.deviceKey = `${device.vendorId}:${device.productId}:${device.busNum}:${device.devAddress}`;
      this.attached = true;
      this.claimed = false;
      this.consecutiveFailures = 0;
      this.streaming = false;
      this.streamingSinceMs = 0;
      this.lastPcmRecvMs = 0;
      this.lastSendOkMs = 0;
      this.startHeartbeat();
      Logger.info(TAG,
        `雷蛇触感接口已就绪（先不声明，直接写端点）: iface=${iface.id}, ` +
        `OUT=0x${outEp.address.toString(16)}, IN=0x${inEp.address.toString(16)}, ` +
        `VID=0x${device.vendorId.toString(16)} PID=0x${device.productId.toString(16)}`);
      this.playStartupProbe();
    } catch (err) {
      Logger.warn(TAG, '占用触感接口异常:', err);
    }
  }

  /** 设备已拔出或 pipe 即将关闭时调用。释放接口并复位状态。 */
  detach(): void {
    if (this.attachTimer !== -1) {
      clearTimeout(this.attachTimer);
      this.attachTimer = -1;
    }
    this.stopHeartbeat();
    this.pendingDevice = null;
    this.pendingPipe = null;
    this.pendingInterface = null;
    this.pendingEndpoints = null;
    this.claiming = false;
    this.stopFrameTimer();
    const wasAttached = this.attached;
    this.attached = false;
    this.playing = false;
    this.current = { low: 0, high: 0 };
    const pipe = this.pipe;
    const iface = this.usbInterface;
    this.pipe = null;
    this.usbInterface = null;
    this.outEndpoint = null;
    this.inEndpoint = null;
    this.interfaceId = -1;
    this.renderLogCount = 0;
    this.frameLogCount = 0;
    if (pipe !== null && iface !== null && this.claimed) {
      try {
        usbManager.releaseInterface(pipe, iface);
      } catch (_) { /* 设备可能已消失，忽略 */ }
    }
    this.claimed = false;
    if (wasAttached) {
      const pcmAge = this.lastPcmRecvMs > 0 ? Date.now() - this.lastPcmRecvMs : -1;
      Logger.warn(TAG, `触感接口已释放（失败连击=${this.consecutiveFailures}, ` +
        `streaming=${this.streaming ? 1 : 0}, 距上帧PCM=${pcmAge}ms）`);
    }
    this.streaming = false;
    this.streamingSinceMs = 0;
  }

  get isAttached(): boolean {
    return this.attached;
  }

  // ==================== 设置 ====================

  /**
   * 更新触感设置。
   * @param strength 总强度 0-100
   * @param curve100 响应曲线 ×100（100 线性；<100 抬升弱效果，>100 压制弱效果）
   * @param heavyFreq 重马达频率索引 0-127
   * @param lightFreq 轻马达频率索引 0-127
   */
  applySettings(strength: number, curve100: number, heavyFreq: number,
    lightFreq: number, enabled: boolean): void {
    this.enabled = enabled;
    this.strength = Math.max(0, Math.min(100, Math.round(strength)));
    this.curve100 = Math.max(30, Math.min(200, Math.round(curve100)));
    this.heavyFreq = Math.max(0, Math.min(127, Math.round(heavyFreq)));
    this.lightFreq = Math.max(0, Math.min(127, Math.round(lightFreq)));
  }

  /**
   * 设置重/轻马达缩放（0-100），供震动模式（仅手柄/仅设备等）联动使用。
   */
  setMotorScales(heavyScale: number, lightScale: number): void {
    this.heavyScale = Math.max(0, Math.min(100, Math.round(heavyScale)));
    this.lightScale = Math.max(0, Math.min(100, Math.round(lightScale)));
  }

  /**
   * 设置触感总增益（%）。100 为不额外放大；应用层默认 140，
   * 并把「游戏震动强度」滑块（0-200）等比例映射进来，让用户可以自行加力。
   */
  setBoost(percent: number): void {
    this.boostPercent = Math.max(50, Math.min(300, Math.round(percent)));
    Logger.info(TAG, `触感增益更新: ${this.boostPercent}% (curve=${this.curve100})`);
  }

  get boost(): number {
    return this.boostPercent;
  }

  /**
   * 把 DS5 触觉分析帧的频谱/瞬态/持续特征送进雷蛇波形合成器。
   * 四个参数全为零且 sustained=true 时，帧格式与 rumblebridge 参考实现
   * 逐字节一致（纯 rumble 兼容模式）。
   */
  setTimbre(tone: number, attack: number, texture: number = 0,
    sustained: boolean = true): void {
    // 原始波形流接管时，IR / 本地音频分析得到的统计描述会覆盖真实
    // 波形特征，因此直接忽略。
    if (this.pcmMode && this.effectUntilMs > Date.now()) return;
    this.tone = clampUnit(tone);
    this.attack = clampUnit(attack);
    this.texture = clampUnit(texture);
    this.sustained = sustained;
    if (this.attack >= 0.35 && !sustained) this.pendingAttack = true;
  }

  /** 主机原始波形流是否正在驱动帧流（供 mixer 避免重复下发 rumble） */
  get isPcmActive(): boolean {
    return this.pcmMode && this.effectUntilMs > Date.now();
  }

  /**
   * 用主机原始 DualSense 触觉波形（48kHz 立体声）测得的真实包络直接渲染。
   *
   * v18 起 native 把 DS5 双声道重采样为频段 lane：leftRms = 低频带包络
   * （重马达），rightRms = 高频带包络（轻马达）。沙砾/金属等材质颗粒
   * 落在轻马达，撞击器/引擎落在重马达，不再依赖单一 tex 参数。
   *
   * 这是与「参考 DS5 有线插 PC 的震感」最接近的路径：强弱、持续长度、
   * 频段与颗粒都来自波形本身，不再经 level() 的曲线/boost/软膝，也不再
   * 用 HOLD_SUSTAINED_MS 猜事件长短。主机连续发帧就连续震，停发即静音。
   */
  renderPcmFrame(frame: Ds5HapticsPcmFrame): void {
    if (!this.enabled) return;
    // v24: 仅当 raw-PCM 确实在成功发帧时让路。raw 挂载但无帧流/发送失败时
    // 0x0E 必须继续兜底，否则一旦 raw 静默就整条触感链路没声音。
    if (RazerKishiRawHaptics.getInstance().isActive) return;
    const nowMs = Date.now();
    if (frame.silent !== 0) {
      this.pcmMode = false;
      this.current = { low: 0, high: 0 };
      this.effectUntilMs = 0;
      // v58: 流结束/断流时清掉瞬态锁定，避免残留峰值让下一段开头假震一下
      this.pcmHoldLow = 0;
      this.pcmHoldHigh = 0;
      this.pcmHoldUntilMs = 0;
      return;
    }
    // v67: 诊断——记录 PCM 帧真实到达时刻，并在断流恢复时告警。
    // 完全静默时，心跳日志里的「距上帧PCM」能立刻区分是上游断流还是发送侧故障。
    if (this.lastPcmRecvMs > 0 && nowMs - this.lastPcmRecvMs > PCM_GAP_WARN_MS) {
      Logger.warn(TAG, `PCM 断流已恢复: 中断 ${nowMs - this.lastPcmRecvMs}ms`);
    }
    this.lastPcmRecvMs = nowMs;
    // 真实频段包络 -> 0..63。native v19 已做噪声门（消除持续微嗡底噪）+
    // 6x 感知扩展（抬升中低档纹理、彼此仍可分档）+ 软膝上限（强帧保序）。
    // 这里只乘用户总强度滑块，保持游戏原本的强弱与材质层次。
    const gain = this.strength / 100;
    let low = Math.max(0, Math.min(63, Math.round(frame.leftRms * 63 * gain)));
    let high = Math.max(0, Math.min(63, Math.round(frame.rightRms * 63 * gain)));
    // 单侧输入时给另一侧补底，避免只有一只手有触感（沿用既有策略）。
    if (low === 0 && high > 0) low = Math.max(1, Math.round(high * 0.6));
    if (high === 0 && low > 0) high = Math.max(1, Math.round(low * 0.6));

    // v58/v59: 瞬态峰值锁定 + 余震衰减（落地/撞击既明显又不丢）。
    // v59 关键修正：主判据改为绝对档位（>=55），因为实测雨滴峰值最高 48、
    // 49~62 完全为空、落地直接 63 —— 这是干净的分界；单看 rise 会被雨滴淹没。
    // v66 关键修正（消除「雨滴真空期」）：
    //   ① **删除 rise 辅助判据**。v65 日志实证它仍在误触发：361 次锁定里有 72 次
    //      是 `rise=36~50 满档=0`、`L=36~52` 的**雨滴帧**。雨滴本身的档位波动就有
    //      30~50，rise 永远区分不了它和落地。
    //   ② **余震只给重马达做长尾**，轻马达改为 k² 快速回落。v65 把 170ms 余震
    //      同时作用在两路上：雨滴密集时锁定被连续刷新，轻马达被反复拉满，
    //      马达饱和后短暂停震 ⇒ 用户感知「雨滴真空期」。落地本来就是低频冲击的
    //      长尾，高频碎裂声本来就短促，不该被拖长。
    {
      const peakNow = Math.max(low, high);
      if (peakNow >= PCM_TRANSIENT_LEVEL) {
        // 落地/撞击：锁满峰值，并从此开始 PCM_PEAK_HOLD_MS 的线性衰减
        this.pcmHoldLow = low;
        this.pcmHoldHigh = high;
        this.pcmHoldStartMs = nowMs;
        this.pcmHoldUntilMs = nowMs + PCM_PEAK_HOLD_MS;
        Logger.info(TAG, `瞬态锁定: L=${low} H=${high} ` +
          `满档=1 余震低=${PCM_PEAK_HOLD_MS}ms 高频倍率=1/3`);
      } else if (nowMs < this.pcmHoldUntilMs) {
        // 保持期：低频线性衰减做长尾；高频用 k² 在前 1/3 时间就回落到雨滴电平
        const span = this.pcmHoldUntilMs - this.pcmHoldStartMs;
        const k: number = span > 0 ? (this.pcmHoldUntilMs - nowMs) / span : 0;
        const envLow = Math.round(low + (this.pcmHoldLow - low) * k);
        const envHigh = Math.round(high + (this.pcmHoldHigh - high) * k * k);
        if (envLow > low || envHigh > high) this.pcmHoldRescued++;
        low = Math.max(low, envLow);
        high = Math.max(high, envHigh);
      } else {
        this.pcmHoldLow = low;
        this.pcmHoldHigh = high;
      }
    }

    // 真实频段占比 / 过零率 / 瞬态驱动 4 频带合成：不同地面材质
    // （沙地、草地、金属）的波形差异在这里被保留下来。
    const lowRatio = (frame.leftLowRatio + frame.rightLowRatio) * 0.5;
    this.tone = 1 - clampUnit(lowRatio);
    this.texture = clampUnit(frame.texture);
    this.attack = clampUnit(frame.attack);
    // v20: 用 DS5 真实频谱子带权重驱动 4 谐波分量（替代纯 tone 标量权重）。
    // 只有 native 真的下发了 4 子带带宽才进入频谱路径；否则回退 rumble/HD。
    const incoming = (frame.bandAmp && frame.bandAmp.length === 4) ? frame.bandAmp : null;
    this.pcmSpectral = incoming !== null;
    this.bandAmp = incoming
      ? [incoming[0], incoming[1], incoming[2], incoming[3]]
      : [0, 0, 0, 0];
    this.sustained = true;
    this.sustainedEvent = true;
    this.pendingAttack = false;
    this.probeMode = false;
    this.pcmMode = true;
    this.current = { low: low, high: high };
    this.effectUntilMs = nowMs + PCM_HOLD_MS;

    if (nowMs - this.lastPcmLogMs >= 1000) {
      this.lastPcmLogMs = nowMs;
      Logger.info(TAG,
        `触感 PCM: L=${low} H=${high} band=${frame.leftRms.toFixed(3)}/` +
        `${frame.rightRms.toFixed(3)} lowRatio=${lowRatio.toFixed(2)} ` +
        `tex=${this.texture.toFixed(2)} atk=${this.attack.toFixed(2)} ` +
        `frames=${frame.frameCount} 保住瞬态=${this.pcmHoldRescued}`);
      this.pcmHoldRescued = 0;
    }
    if (!this.playing && this.attached) {
      this.playing = true;
      this.startFrameTimer();
    }
  }

  // ==================== 渲染入口 ====================

  /**
   * 渲染一次混合后的双马达输出（0-65535，与 Moonlight rumble 一致）。
   * 由 GamepadVibrationService 在每次 USB 输出渲染时调用。
   * 内部把请求转成 0-63 触感幅度，并以 30ms 帧流式播放直到效果到期。
   */
  render(lowFreqMotor: number, highFreqMotor: number): void {
    if (!this.enabled) return;
    // v24: 同 renderPcmFrame，仅 raw 真实活跃时让路。
    if (RazerKishiRawHaptics.getInstance().isActive) return;
    // 原始波形流驱动时，rumble 统计值不能覆盖由真实 PCM 测得的包络。
    // pcmMode 由 streamTick 在效果过期时清理，这里不主动翻转，避免
    // mixer 的 400ms 活跃窗口与帧流状态不一致。
    if (this.pcmMode && this.effectUntilMs > Date.now()) return;
    const nowMs = Date.now();
    if (nowMs - this.lastRenderLogMs >= 500) {
      this.lastRenderLogMs = nowMs;
      Logger.info(TAG,
        `触感 render: low=${lowFreqMotor} high=${highFreqMotor} ` +
        `boost=${this.boostPercent}% curve=${this.curve100} attached=${this.attached}`);
    }
    let low = this.level(lowFreqMotor, this.heavyScale);
    let high = this.level(highFreqMotor, this.lightScale);

    // 音色只改变「重/低频」与「轻/高频」两个实测共振峰之间的能量分配，
    // 不再把幅度摊薄到马达几乎不响应的频率索引上（v13 的宽频拆分实测更糊）。
    if (low > 0 && high > 0 && this.tone > 0.01) {
      const shift = this.tone * 0.30;
      low = Math.max(1, Math.round(low * (1 - shift)));
      high = Math.min(63, Math.round(high * (1 + shift)));
    }

    if (low === 0 && high === 0) {
      // 噪声门关上 = 事件之间真正静音，帧循环会立刻发一帧静音帧收尾。
      this.current = { low: 0, high: 0 };
      this.effectUntilMs = 0;
      return;
    }

    const previousSum = this.current.low + this.current.high;
    const expired = this.effectUntilMs <= nowMs;
    const isNewEvent = expired || this.eventStartMs === 0 ||
      low + high > previousSum + NEW_EVENT_DELTA;
    if (isNewEvent) {
      this.eventStartMs = nowMs;
      this.strongEvent = low + high >= ATTACK_MIN_SUM;
      // 持续型保持满幅高幅度；只有瞬态型才插 10ms 锐利起音短帧。
      // v15 把所有事件都当成短促处理，正是用户说的「没有区分」。
      this.sustainedEvent = this.sustained || this.attack < 0.35;
      this.pendingAttack = this.strongEvent && !this.sustainedEvent;
    }
    // 峰值双向平滑跟随主机值：既能防止上一事件的峰值一直顶住，也能在主机
    // 骤然降幅时及时收下去（只升不降会让每一次震动都拖一条尾巴）。
    this.current = {
      low: isNewEvent ? low : Math.round(this.current.low * 0.75 + low * 0.25),
      high: isNewEvent ? high : Math.round(this.current.high * 0.75 + high * 0.25)
    };
    // 持续强震动给足 600ms 满幅，瞬态短促收尾，弱流停发后快速静音。
    const peak = Math.max(this.current.low, this.current.high);
    const holdMs = this.sustainedEvent
      ? (peak >= ATTACK_MIN_SUM ? HOLD_SUSTAINED_MS : HOLD_WEAK_MS)
      : HOLD_TRANSIENT_MS;
    this.effectUntilMs = nowMs + holdMs;
    this.probeMode = false;
    if (!this.playing && this.attached) {
      this.playing = true;
      this.startFrameTimer();
    }
  }

  /** 立即停止触感输出（发送一帧静音帧）。 */
  forceStop(): void {
    this.current = { low: 0, high: 0 };
    this.effectUntilMs = 0;
    this.eventStartMs = 0;
    this.probeMode = false;
    this.pcmMode = false;
    this.strongEvent = false;
    this.sustainedEvent = true;
  }

  /**
   * 事件包络：持续型不衰减（保持满幅高幅度），瞬态型按强度衰减。
   * 这样引擎/冰块摩擦是持续的强震，枪声是干脆的一下，雨声是细密连续流。
   */
  private envelope(ageMs: number, peak: number, sustained: boolean): number {
    if (sustained || ageMs <= 0) return 1;
    const strength = Math.max(0, Math.min(1, peak / ENVELOPE_STRONG_PEAK));
    const sustain = ENVELOPE_SUSTAIN_WEAK +
      (ENVELOPE_SUSTAIN_STRONG - ENVELOPE_SUSTAIN_WEAK) * strength;
    const decayed = sustain + (1 - sustain) * Math.exp(-ageMs / ENVELOPE_DECAY_MS);
    return Math.max(sustain, Math.min(1, decayed));
  }

  /** 确定性伪随机（小模数 LCG，避免 ArkTS 浮点位运算溢出） */
  private jitter(): number {
    this.frameSerial = (this.frameSerial * 75 + 74) % 65537;
    return this.frameSerial / 65537;
  }

  // ==================== 帧流 ====================

  /**
   * 挂载成功后播一段自检震动（约 300ms + 一帧静音收尾）。
   * 左右两个通道同时给同等幅度，便于用户明确感知「两侧都有触感」；
   * 100ms 太短，容易被误判成只有单侧在响。
   * 只发送协议允许的 command 0x0E 流帧，不发任何查询/设置命令。
   */
  private playStartupProbe(): void {
    if (!this.attached || this.playing) return;
    Logger.info(TAG, '触感链路自检：左右双通道各 45/63，持续约 300ms');
    this.current = { low: 45, high: 45 };
    this.effectUntilMs = Date.now() + 300;
    this.eventStartMs = Date.now();
    this.probeMode = true;
    this.playing = true;
    this.startFrameTimer();
  }

  private startFrameTimer(): void {
    if (this.frameTimer !== -1) return;
    this.streaming = false;
    this.frameTimer = setInterval((): void => {
      this.streamTick();
    }, FRAME_MS);
  }

  private stopFrameTimer(): void {
    if (this.frameTimer !== -1) {
      clearInterval(this.frameTimer);
      this.frameTimer = -1;
    }
    this.streaming = false;
    this.streamingSinceMs = 0;
    this.playing = false;
  }

  // ==================== v67 链路心跳 ====================

  /**
   * 常驻心跳：每 3 秒打一条链路状态，**独立于帧流**。
   *
   * 为什么必需：0x0E 链路任何一层断掉，表现出来都是「完全不震」，而现有日志
   * 全都挂在「正在发帧」这条路径上 —— 一旦不发帧就再无任何输出，排查等于摸黑。
   * 靠这条心跳在完全静默时也能一眼判断断点：
   *   距上帧PCM 大、距上次发送成功 也大  -> 主机/网络侧 PCM 断流（上游问题）
   *   距上帧PCM 小、距上次发送成功 大    -> 本驱动没发出去（streaming 卡死 / 已 detach）
   *   两者都小                          -> 帧确实在发，设备不执行（休眠 / 过热保护 / 固件）
   */
  private startHeartbeat(): void {
    if (this.heartbeatTimer !== -1) return;
    this.heartbeatTimer = setInterval((): void => {
      this.heartbeatTick();
    }, HEARTBEAT_MS);
  }

  private stopHeartbeat(): void {
    if (this.heartbeatTimer !== -1) {
      clearInterval(this.heartbeatTimer);
      this.heartbeatTimer = -1;
    }
  }

  private heartbeatTick(): void {
    const nowMs = Date.now();
    const pcmAge = this.lastPcmRecvMs > 0 ? nowMs - this.lastPcmRecvMs : -1;
    const sendAge = this.lastSendOkMs > 0 ? nowMs - this.lastSendOkMs : -1;
    Logger.info(TAG,
      `链路心跳: attached=${this.attached ? 1 : 0} playing=${this.playing ? 1 : 0} ` +
      `streaming=${this.streaming ? 1 : 0} pcmMode=${this.pcmMode ? 1 : 0} ` +
      `距上帧PCM=${pcmAge}ms 距上次发送成功=${sendAge}ms ` +
      `失败连击=${this.consecutiveFailures} L=${this.current.low} H=${this.current.high}`);
  }

  private async streamTick(): Promise<void> {
    // v67: 看门狗。若上次发送卡死（usbManager.bulkTransfer 的 Promise 不 settle），
    // streaming 会永久为 true，之后每个 tick 都在这里被吞掉 ⇒ 完全静默且零日志。
    // 超过 STREAM_STUCK_MS 就强制复位；正常一帧上限约 106ms，绝不会误触发。
    if (this.streaming) {
      const stuckMs = Date.now() - this.streamingSinceMs;
      if (stuckMs < STREAM_STUCK_MS) return;  // 上一帧尚未完成（慢链路），跳过本拍
      Logger.warn(TAG, `帧流卡死 ${stuckMs}ms（bulkTransfer 未返回），强制复位 streaming`);
      this.streaming = false;
      this.streamingSinceMs = 0;
    }
    if (!this.attached || this.pipe === null ||
        this.outEndpoint === null || this.inEndpoint === null) {
      this.stopFrameTimer();
      return;
    }
    // v24: raw-PCM 正往同一个 64B OUT 端点直发时，0x0E 帧流本拍完全不写，
    // 避免两种协议帧在同一端点混发。只跳过本拍、不停定时器，raw 帧流
    // 停止 250ms 后 0x0E 会立刻自动接管（isActive 窗口过期）。
    if (RazerKishiRawHaptics.getInstance().isActive) return;
    const nowMs = Date.now();
    const on = this.effectUntilMs > nowMs;
    if (!on && this.pcmMode) this.pcmMode = false;
    const low = on ? this.current.low : 0;
    const high = on ? this.current.high : 0;
    if (!on && !this.playing) {
      // 空闲：什么都不发，让设备按自己的节奏休眠。
      this.stopFrameTimer();
      return;
    }
    // 一帧静音帧让效果立即收尾，而不是等最后一帧自然跑完。
    this.playing = on;
    this.streaming = true;
    this.streamingSinceMs = nowMs;
    let frameLow = low;
    let frameHigh = high;
    let durationMs = FRAME_MS;
    let envelope = 1;
    // PCM 波形的每一帧都已带着真实包络，再叠一层人造衰减会把强弱
    // 变化抹平；只有 rumble / IR 回退路径才需要合成包络。
    if (on && !this.probeMode && !this.pcmMode) {
      envelope = this.envelope(nowMs - this.eventStartMs,
        Math.max(low, high), this.sustainedEvent);
      frameLow = Math.round(low * envelope);
      frameHigh = Math.round(high * envelope);
    }
    if (this.pendingAttack) {
      this.pendingAttack = false;
      if (on && !this.probeMode && this.strongEvent && !this.sustainedEvent) {
        durationMs = ATTACK_FRAME_MS;
        frameLow = Math.min(63, Math.round(low * ATTACK_GAIN));
        frameHigh = Math.min(63, Math.round(high * ATTACK_GAIN));
      }
    }
    const report = this.buildFrameReport(frameLow, frameHigh, durationMs);
    let sent = false;
    try {
      sent = await this.writeReport(report);
      if (sent) {
        this.consecutiveFailures = 0;
        this.lastSendOkMs = nowMs;
        if (nowMs - this.lastFrameLogMs >= 500) {
          this.lastFrameLogMs = nowMs;
          Logger.info(TAG,
            `触感帧: L=${frameLow} H=${frameHigh} peak=${low}/${high} ` +
            `env=${envelope.toFixed(2)} dur=${durationMs} ` +
            `sus=${this.sustainedEvent ? 1 : 0} pcm=${this.pcmMode ? 1 : 0} ` +
            `tex=${this.texture.toFixed(2)} report=${hexOf(report, 20)}`);
        }
        await this.readAnswer();
      }
    } finally {
      this.streaming = false;
      this.streamingSinceMs = 0;
    }
    if (!sent) {
      this.consecutiveFailures++;
      // 描述符校准期可能把内核驱动重新绑上，连续失败时声明接口再试
      if (this.consecutiveFailures === 3 && !this.claimed) {
        this.ensureClaimed();
      }
      if (this.consecutiveFailures >= FAILURE_DETACH_THRESHOLD) {
        Logger.warn(TAG, '触感帧连续发送失败，释放触感接口（设备可能已休眠或拔出）');
        this.detach();
      }
    }
  }

  /**
   * 发送一帧 output report 2。
   *
   * 分级尝试，尽量不打扰已经正常工作的手柄输入接口：
   *   1. 不声明触感接口，直接写 interrupt OUT 端点（与 rumblebridged 的 usbfs ioctl 等价）；
   *   2. 仍失败则走 HID 标准的 SET_REPORT(Output) 控制传输；
   *   3. 两条都不通才声明触感接口，然后整轮重试。
   * 所有通道发的都是同一个 command 0x0E 流帧，绝不发送查询/设置命令。
   */
  private async writeReport(report: Uint8Array): Promise<boolean> {
    if (await this.tryWriteOnce(report)) return true;
    if (!this.claimed && this.ensureClaimed()) {
      Logger.warn(TAG, '触感输出首轮失败，已声明接口后重试');
      if (await this.tryWriteOnce(report)) return true;
    }
    return false;
  }

  /** 一轮发送：interrupt OUT 端点 → SET_REPORT 控制传输兜底 */
  private async tryWriteOnce(report: Uint8Array): Promise<boolean> {
    if (this.pipe === null || this.outEndpoint === null) return false;
    try {
      const written = await usbManager.bulkTransfer(this.pipe, this.outEndpoint, report, OUT_TIMEOUT_MS);
      if (typeof written === 'number' && written > 0) return true;
      if (this.frameLogCount < 6) {
        Logger.warn(TAG, `触感帧 interrupt-OUT 返回 ${written}`);
      }
    } catch (err) {
      if (this.frameLogCount < 6) {
        Logger.warn(TAG, '触感帧 interrupt-OUT 异常:', err);
      }
    }
    try {
      const params: usbManager.USBDeviceRequestParams = {
        bmRequestType: SET_REPORT_BM_REQUEST_TYPE,
        bRequest: SET_REPORT_B_REQUEST,
        wValue: SET_REPORT_W_VALUE,
        wIndex: this.interfaceId,
        wLength: report.length,
        data: report
      };
      const ret = await usbManager.usbControlTransfer(this.pipe, params, OUT_TIMEOUT_MS);
      if (typeof ret === 'number' && ret > 0) {
        Logger.info(TAG, `触感帧走 SET_REPORT 成功: ret=${ret}`);
        return true;
      }
      if (this.frameLogCount < 6) {
        Logger.warn(TAG, `触感帧 SET_REPORT 返回 ${ret}`);
      }
    } catch (err) {
      if (this.frameLogCount < 6) {
        Logger.warn(TAG, '触感帧 SET_REPORT 异常:', err);
      }
    }
    return false;
  }

  /** 设备对每帧都有回答；读不到回答只作诊断，不判为发送失败。 */
  private async readAnswer(): Promise<void> {
    if (this.pipe === null || this.inEndpoint === null) return;
    try {
      const answer = new Uint8Array(ANSWER_LENGTH);
      const n = await usbManager.bulkTransfer(this.pipe, this.inEndpoint, answer, IN_TIMEOUT_MS);
      if (this.frameLogCount <= 4) {
        Logger.info(TAG, `触感帧应答: ${n}B ${hexOf(answer, 12)}`);
      }
    } catch (_) { /* 无应答或超时：不影响 OUT 发送结果 */ }
  }

  /** 内核 HID 驱动占用接口时，才声明接口（会与手柄输入接口共存）。 */
  private ensureClaimed(): boolean {
    if (this.claimed) return true;
    if (this.pipe === null || this.usbInterface === null) return false;
    try {
      const ret = usbManager.claimInterface(this.pipe, this.usbInterface, true);
      this.claimed = true;
      Logger.warn(TAG, `已声明触感接口 id=${this.interfaceId}: ret=${ret}`);
      return ret >= 0;
    } catch (err) {
      Logger.warn(TAG, '声明触感接口异常:', err);
      return false;
    }
  }

  // ==================== 协议编码 ====================

  /**
   * 游戏 16bit 马达强度 → 触感 0-63 幅度，按总强度/马达缩放/响应曲线调整。
   * curve100=100 线性；<100 抬升弱效果。默认 60，再叠加 boostPercent 增益，
   * 非零输入保底 MIN_PERCEPTIBLE_LEVEL，满幅请求仍 clamp 到 63。
   */
  private level(strength: number, motorScale: number): number {
    if (strength <= 0 || !this.enabled) return 0;
    const normalized = strength / 65535.0;
    // curve100 < 100 抬升弱效果：DS5 音频触觉与很多游戏的 rumble 只有满幅的
    // 5%-20%，线性映射后会被 6bit 量化压到几乎无感。
    const curved = Math.pow(normalized, this.curve100 / 100.0);
    const scaled = 63 * (this.strength / 100.0) * (motorScale / 100.0) *
      (this.boostPercent / 100.0) * curved;
    const rounded = Math.round(scaled);
    // 软膝：v14 的硬噪声门把 1-4/63 直接归零，剑星下雨这类细密弱信号
    // 因此完全消失。这里改为把「真实的非零弱请求」抬到可感底 —— 参考
    // PeaSyo 转发 PS5 音频触觉流的做法：不砍弱信号，只保证 0 仍是绝对静音。
    if (rounded <= 0) return 0;
    if (rounded < SOFT_KNEE_LEVEL) return SOFT_KNEE_LEVEL;
    return Math.min(63, rounded);
  }

  /**
   * 构造一帧双马达触感流（output report 2）：
   * [2][len][0][fragment=1][0x0E][7bit 时长 | 通道×(1bit 频带 + 4×(6bit 幅度,7bit 频率) + 1bit 结束 + 1bit 无瞬态)]
   * 幅度直接使用 0-63 触感值（level() 已完成 0-65535 → 0-63 的换算）。
   */
  private buildFrameReport(low: number, high: number,
    durationMs: number = FRAME_MS): Uint8Array {
    const report = new Uint8Array(REPORT_LENGTH);
    report[0] = 2;
    report[1] = 4 + 15;
    report[2] = 0;
    report[3] = 1;
    report[4] = 0x0E;

    const amps: number[] = [low, high];
    const freqs: number[] = [this.heavyFreq, this.lightFreq];
    // 单马达驱动时镜像到两侧，避免只有一只手有触感。
    // 两个独立 if 与 rumblebridged.c 的 send_frame() 严格一致。
    if (high === 0) {
      amps[1] = low;
      freqs[1] = this.heavyFreq;
    }
    if (low === 0) {
      amps[0] = high;
      freqs[0] = this.lightFreq;
    }
    // 游戏常只驱动重马达，给轻马达一个很小的非零值（例如 low=63, high=2）。
    // 上面的镜像只在「严格等于 0」时触发，这种帧右侧幅度只有 2/63，用户会
    // 感觉「只有左边在震」。给明显偏弱的一侧按较强侧的 60% 补底，既保留
    // 左右方向感，又保证两只手都能感知到。参考帧（全零/全满/对称）不受影响。
    if (amps[0] > 0 && amps[1] > 0) {
      if (amps[1] * 100 < amps[0] * 35) {
        amps[1] = Math.round(amps[0] * 0.6);
      } else if (amps[0] * 100 < amps[1] * 35) {
        amps[0] = Math.round(amps[1] * 0.6);
      }
    }

    // === DS5 HD 触觉 → 雷蛇 Sensa HD 多频带波形 ===
    // 一个 band 里的 4 组 (幅度, 频率) 是同时合成的 4 个频率分量。参考实现
    // 把它们全填成同一个值，只能播出单一频率的平顶嗡鸣；把 DS5 分析帧的
    // 频谱特征映射到这里，才有冰块/沙砾那种复杂质感。
    // 频率只取实测有效点（5/10/35），避免 v13 摊到无效索引上把能量稀释。
    const hd = this.tone > 0.01 || this.texture > 0.01 || this.attack > 0.01;
    const jitterSample = hd ? this.jitter() : 0;
    const weights: number[] = hd ? [
      1.0 - 0.25 * this.tone,                    // 低频基底
      1.0,                                       // 主峰，承载体感
      0.55 + 0.45 * this.tone,                   // 中高频过渡
      (0.25 + 0.70 * this.tone) *                // 高频细节 + 逐帧颗粒
        (1 - TEXTURE_JITTER * this.texture * jitterSample)
    ] : [1.0, 1.0, 1.0, 1.0];

    // hd=false 时与 rumblebridged.c 的 send_frame() 逐字节一致（纯 rumble 兼容）。
    // v20: 仅当本帧来自真实 DS5 PCM 频谱（pcmSpectral）时才用 PCM_SPECTRAL 谐波
    // 映射；否则保留原 rumble/HD 频点，避免破坏与 rumblebridge 参考实现的字节一致。
    const useSpectral = this.pcmSpectral && this.bandAmp.length === 4;
    const writer = new BitWriter(report, 40);  // 5 字节头之后
    writer.put(durationMs, 7);
    for (let c = 0; c < 2; c++) {
      writer.put(1, 1);  // 本通道有频带
      const bandFreqs = useSpectral
        ? (c === 0 ? PCM_SPECTRAL_HEAVY : PCM_SPECTRAL_LIGHT)
        : (hd ? (c === 0 ? HD_HEAVY_FREQS : HD_LIGHT_FREQS)
              : [freqs[c], freqs[c], freqs[c], freqs[c]]);
      for (let i = 0; i < 4; i++) {
        const w = useSpectral ? this.bandAmp[i] : (hd ? weights[i] : 1.0);
        const bandAmp = Math.max(0, Math.min(63, Math.round(amps[c] * w)));
        writer.put(bandAmp, 6);     // 0-63
        writer.put(bandFreqs[i], 7); // 0-127
      }
      writer.put(0, 1);  // 无更多频带
      writer.put(0, 1);  // 无瞬态
    }
    return report;
  }

}

function clampUnit(value: number): number {
  if (!Number.isFinite(value) || value <= 0) return 0;
  return Math.min(1, value);
}

/** 把前 length 字节转成十六进制串（诊断日志用） */
function hexOf(data: Uint8Array, length: number): string {
  const n = Math.min(length, data.length);
  const parts: string[] = new Array<string>();
  for (let i = 0; i < n; i++) {
    const s = '0' + data[i].toString(16);
    parts.push(s.substring(s.length - 2));
  }
  return parts.join(' ');
}

/** MSB-first 位流写入器（对应 rumblebridged.c 的 put_bits） */
class BitWriter {
  private buffer: Uint8Array;
  private bit: number;

  constructor(buffer: Uint8Array, startBit: number) {
    this.buffer = buffer;
    this.bit = startBit;
  }

  put(value: number, count: number): void {
    for (let i = count - 1; i >= 0; i--) {
      if (((value >> i) & 1) !== 0) {
        // ArkTS/JS 的 `/` 是浮点除法！C 参考实现里 out[*bit / 8] 是整数除法，
        // 这里若直接写 this.buffer[this.bit / 8]，非整数索引对 TypedArray 的赋值
        // 会被静默忽略，只有 bit 为 8 的倍数时才落到字节上（结果只剩 0x00/0x80），
        // 幅度和频率全丢，设备收到的是「零幅度帧」——能应答但不会震动。
        const byteIndex: number = Math.floor(this.bit / 8);
        this.buffer[byteIndex] |= 1 << (7 - (this.bit % 8));
      }
      this.bit++;
    }
  }
}
