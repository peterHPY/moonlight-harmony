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
 * 使用原生层解析的通用 HID 手柄控制器
 * 
 * 这个版本使用 C++ 原生层来解析 HID 报告
 * 支持更多手柄类型，解析更准确
 */

import { usbManager } from '@kit.BasicServicesKit';
import { AbstractController } from './AbstractController';
import { UsbDriverListener } from './UsbDriverListener';
import { ButtonFlags, UsbClass, ControllerType, ControllerCapabilities } from './ControllerConstants';
import { AXIS_MAX, TRIGGER_MAX } from '../input/GamepadTypes';
import { NativeHidParserService, NativeGamepadState, NativeGamepadType, NativeButtonFlags } from './NativeHidParserService';
import { createUsbError, UsbError } from './UsbErrorCodes';
import { Logger } from '../../common/Logger';
import {
  HidReportLayout,
  HidGamepadState,
  HidInterfaceInfo,
  parseHidReportDescriptor,
  parseUsbConfiguration,
  parseRawUsbDescriptors,
  scoreHidLayout,
  decodeHidReport,
  decodeXbox360InputReport,
  describeHidReport,
  applyKishiUltraButtonMap,
  hexDump
} from './HidReportDescriptor';

const TAG = '[USB-NativeHID]';

/**
 * 使用原生解析器的通用 HID 手柄控制器
 */
export class NativeHidController extends AbstractController {
  private device: usbManager.USBDevice;
  private pipe: usbManager.USBDevicePipe;
  private running: boolean = false;
  private inputEndpoint: number = 0;
  private outputEndpoint: number = 0;
  private inputMaxPacketSize: number = 64;
  private interfaceId: number = 0;
  private nativeParser: NativeHidParserService;
  private forceProtocolType: number = 0;  // 0=自动, 1=Xbox, 2=DS4, 3=Switch, 5=DualSense
  /** 依据 HID Report Descriptor 动态生成的输入布局；null 表示回退到原有解析路径 */
  private hidLayout: HidReportLayout | null = null;
  private hidLoggedFrames: number = 0;
  private rawLoggedFrames: number = 0;
  /** 按键诊断：只记录按钮/方向键/扳机状态发生变化的帧，避免日志淹没 */
  private lastButtonSignature: string = '';
  private buttonLogCount: number = 0;
  private otherReportLogCount: number = 0;
  /** 模拟量诊断：摇杆签名变化时打印，用于判断「扳机是否串到摇杆」。 */
  private lastStickSignature: string = '';
  private stickLogCount: number = 0;
  /** 从配置描述符解析出的候选 HID 接口 */
  private hidInterfaces: HidInterfaceInfo[] = [];
  private claimedInterfaceId: number = -1;
  /**
   * XInput 模式（0xFF/93/1 厂商接口，无 HID Report Descriptor）使用固定
   * X360 20 字节报告解析，跳过描述符校准与原生/后备 HID 解析。
   */
  private isXinputFixedReport: boolean = false;
  /** 首帧自动判定后锁定报告形态，避免 00 14 与 A+Guide 组合撞车 */
  private xinputReportHasHeader: boolean | null = null;
  private xinputLoggedFrames: number = 0;
  private xinputButtonLogCount: number = 0;
  private lastXinputButtonFlags: number = 0;

  // 最后活动时间跟踪（用于检测雷蛇等静默断开的设备）
  private lastActivityTime: number = 0;
  private consecutiveTimeouts: number = 0;
  // 连续超时阈值：如果连续超过这个次数超时且没有数据，可能已断开
  private static readonly SILENT_DISCONNECT_TIMEOUT_COUNT = 10;  // 10次超时 = 30秒（每次3秒）

  /**
   * 协议类型常量
   */
  static readonly PROTOCOL_AUTO = 0;
  static readonly PROTOCOL_XBOX = 1;
  static readonly PROTOCOL_DS4 = 2;
  static readonly PROTOCOL_SWITCH = 3;
  static readonly PROTOCOL_GENERIC = 4;
  static readonly PROTOCOL_DUALSENSE = 5;

  constructor(
    device: usbManager.USBDevice,
    pipe: usbManager.USBDevicePipe,
    deviceId: number,
    listener: UsbDriverListener
  ) {
    super(deviceId, listener, device.vendorId, device.productId);
    this.device = device;
    this.pipe = pipe;
    this.nativeParser = NativeHidParserService.getInstance();

    // 初始化原生解析器
    this.nativeParser.init();

    // 获取手柄类型和名称
    const nativeType = this.nativeParser.getGamepadType(device.vendorId, device.productId);
    this.deviceName = this.nativeParser.getGamepadName(device.vendorId, device.productId);

    // 转换类型
    switch (nativeType) {
      case NativeGamepadType.XBOX:
        this.type = ControllerType.XBOX;
        break;
      case NativeGamepadType.PLAYSTATION:
        this.type = ControllerType.PS;
        break;
      case NativeGamepadType.SWITCH:
        this.type = ControllerType.NINTENDO;
        break;
      default:
        this.type = ControllerType.XBOX; // 默认模拟 Xbox
    }

    this.capabilities = ControllerCapabilities.ANALOG_TRIGGERS | ControllerCapabilities.RUMBLE;

    // 支持的按钮
    this.supportedButtonFlags =
      ButtonFlags.A_FLAG | ButtonFlags.B_FLAG | ButtonFlags.X_FLAG | ButtonFlags.Y_FLAG |
      ButtonFlags.UP_FLAG | ButtonFlags.DOWN_FLAG | ButtonFlags.LEFT_FLAG | ButtonFlags.RIGHT_FLAG |
      ButtonFlags.LB_FLAG | ButtonFlags.RB_FLAG |
      ButtonFlags.LS_CLK_FLAG | ButtonFlags.RS_CLK_FLAG |
      ButtonFlags.BACK_FLAG | ButtonFlags.PLAY_FLAG | ButtonFlags.SPECIAL_BUTTON_FLAG;

    // Kishi Ultra 的 L4/R4 扩展键映射为触摸板按钮，需在上报能力里声明
    if (device.vendorId === 0x1532) {
      this.supportedButtonFlags |= ButtonFlags.TOUCHPAD_FLAG;
    }

    // 解析配置描述符并同步声明候选接口；报告描述符在 start() 后异步读取
    this.findEndpoints();

    Logger.info('NativeHidController', `${TAG} 创建控制器: ${this.deviceName} (type=${nativeType})`);
  }

  /**
   * 设置强制协议类型
   * 用于手柄有多种模式（如 DS4 模拟模式）时强制使用特定协议
   * @param protocolType 协议类型 (0=自动, 1=Xbox, 2=DS4, 3=Switch, 4=Generic, 5=DualSense)
   */
  setForceProtocolType(protocolType: number): void {
    this.forceProtocolType = protocolType;
    Logger.info('NativeHidController', `${TAG} 设置强制协议类型: ${protocolType}`);
  }

  /**
   * 获取当前强制协议类型
   */
  getForceProtocolType(): number {
    return this.forceProtocolType;
  }

  /**
   * HID 子类常量
   * HID 协议定义：
   * - 子类 1: Boot Interface
   * - 协议 1: 键盘
   * - 协议 2: 鼠标
   */
  private static readonly HID_SUBCLASS_BOOT = 1;
  private static readonly HID_PROTOCOL_KEYBOARD = 1;
  private static readonly HID_PROTOCOL_MOUSE = 2;

  /**
   * 检查是否可以驱动该设备
   */
  static canClaimDevice(device: usbManager.USBDevice): boolean {
    const parser = NativeHidParserService.getInstance();
    parser.init();

    // 检查原生层是否支持
    if (parser.isSupportedGamepad(device.vendorId, device.productId)) {
      Logger.info('NativeHidController', `${TAG} 原生支持的手柄: VID=0x${device.vendorId.toString(16)}, PID=0x${device.productId.toString(16)}`);
      return true;
    }

    // 查找可接管的接口：标准 HID 类，或 XInput 兼容设备的 vendor 类接口
    if (!device.configs || device.configs.length === 0) {
      return false;
    }

    const config = device.configs[0];
    if (!config.interfaces || config.interfaces.length === 0) {
      return false;
    }

    // 设备名称整体排除键鼠；复合设备不因单个 boot 接口被整体拒绝
    const nameLower = device.name.toLowerCase();
    if (nameLower.includes('keyboard') || nameLower.includes('键盘') ||
        nameLower.includes('mouse') || nameLower.includes('鼠标')) {
      Logger.info('NativeHidController', `${TAG} 通过名称跳过键鼠: ${device.name}`);
      return false;
    }

    for (let i = 0; i < config.interfaces.length; i++) {
      const iface = config.interfaces[i];
      const eps: usbManager.USBEndpoint[] = iface.endpoints ? iface.endpoints : new Array<usbManager.USBEndpoint>();
      if (eps.length === 0) {
        continue;
      }
      const isHid: boolean = iface.clazz === UsbClass.USB_CLASS_HID;
      // Razer Kishi 等 XInput 兼容设备把输入放在 0xFF / sub 93 / proto 1 接口上
      const isXinputVendor: boolean = iface.clazz === UsbClass.USB_CLASS_VENDOR_SPEC &&
        iface.subClass === 93 && iface.protocol === 1;
      if (!isHid && !isXinputVendor) {
        continue;
      }
      // Boot 键盘/鼠标只是复合设备的一个子接口，跳过它继续找手柄
      if (isHid && iface.subClass === NativeHidController.HID_SUBCLASS_BOOT &&
          (iface.protocol === NativeHidController.HID_PROTOCOL_KEYBOARD ||
           iface.protocol === NativeHidController.HID_PROTOCOL_MOUSE)) {
        Logger.info('NativeHidController', `${TAG} 跳过 boot 键鼠接口[${i}]`);
        continue;
      }
      Logger.info('NativeHidController', `${TAG} 检测到可接管接口[${i}]: VID=0x${device.vendorId.toString(16)}, PID=0x${device.productId.toString(16)}, class=0x${iface.clazz.toString(16)}, sub=${iface.subClass}, proto=${iface.protocol}`);
      return true;
    }

    return false;
  }

  /** 读取并解析配置描述符，得到候选 HID 接口清单 */
  private collectHidInterfaces(): HidInterfaceInfo[] {
    try {
      const raw: Uint8Array = usbManager.getRawDescriptor(this.pipe);
      const infos: HidInterfaceInfo[] = parseUsbConfiguration(raw);
      Logger.info('NativeHidController', `${TAG} 配置描述符解析出 ${infos.length} 个接口; raw=${hexDump(raw, 32)}`);
      for (const info of infos) {
        Logger.info('NativeHidController', `${TAG} 接口 id=${info.interfaceId}: class=0x${info.clazz.toString(16)} sub=${info.subClass} proto=${info.protocol} reportLen=${info.reportDescriptorLength} IN=0x${info.inputEndpoint.toString(16)}/${info.inputMaxPacketSize}B OUT=0x${info.outputEndpoint.toString(16)}`);
      }
      return infos;
    } catch (err) {
      Logger.warn('NativeHidController', `${TAG} 读取配置描述符失败:`, err);
      return new Array<HidInterfaceInfo>();
    }
  }

  /**
   * 同步选接口并声明。
   *
   * 复合 HID 设备（Kishi Ultra 等）会同时暴露 boot 键盘、键盘/鼠标复合、
   * 厂商通道等多个接口。含游戏手柄 Collection 的报告描述符通常明显更长，
   * 因此这里用「报告描述符长度 + 端点信息」做启发式选择，并在 start() 之后
   * 用真实的报告描述符内容做二次校准（见 initializeDescriptorsAsync）。
   */
  private findEndpoints(): void {
    this.hidInterfaces = this.collectHidInterfaces();

    // XInput 模式：Razer Kishi Ultra 切到 XInput 后是 0xFF/93/1 厂商接口，
    // 不发 HID Report Descriptor。此时不能按「报告描述符长度」给接口打分，
    // 否则会选到复合设备里的键盘/触摸板 HID 接口，读到的全是无关报告。
    let xinputIndex: number = -1;
    let hidReportInterfaceCount: number = 0;
    for (let i = 0; i < this.hidInterfaces.length; i++) {
      const info: HidInterfaceInfo = this.hidInterfaces[i];
      if (info.clazz === UsbClass.USB_CLASS_VENDOR_SPEC &&
          info.subClass === 93 && info.protocol === 1 && info.inputEndpoint !== 0) {
        xinputIndex = i;
      }
      if (info.clazz === UsbClass.USB_CLASS_HID && info.reportDescriptorLength > 0 &&
          info.inputEndpoint !== 0) {
        hidReportInterfaceCount++;
      }
    }
    if (xinputIndex >= 0 && (this.productId === 0x0037 || hidReportInterfaceCount === 0)) {
      this.isXinputFixedReport = true;
      const xif: HidInterfaceInfo = this.hidInterfaces[xinputIndex];
      Logger.info('NativeHidController', `${TAG} XInput 模式: 选用厂商接口 id=${xif.interfaceId} (class=0xFF/93/1)，启用固定 X360 报告解析`);
      this.applyInterface(xif, xinputIndex, 0);
      return;
    }

    let bestIndex: number = -1;
    let bestScore: number = -100000;
    for (let i = 0; i < this.hidInterfaces.length; i++) {
      const info: HidInterfaceInfo = this.hidInterfaces[i];
      if (info.inputEndpoint === 0) {
        continue;
      }
      const isHid: boolean = info.clazz === UsbClass.USB_CLASS_HID;
      const isXinputVendor: boolean = info.clazz === UsbClass.USB_CLASS_VENDOR_SPEC &&
        info.subClass === 93 && info.protocol === 1;
      if (!isHid && !isXinputVendor) {
        continue;
      }
      let score: number = info.reportDescriptorLength;
      // boot 键鼠接口（subClass=1，protocol 0/1/2）永远不是手柄主接口
      if (info.subClass === NativeHidController.HID_SUBCLASS_BOOT &&
          (info.protocol === 0 || info.protocol === NativeHidController.HID_PROTOCOL_KEYBOARD ||
           info.protocol === NativeHidController.HID_PROTOCOL_MOUSE)) {
        score -= 5000;
      }
      if (info.inputMaxPacketSize >= 16) {
        score += 20;
      }
      if (score > bestScore) {
        bestScore = score;
        bestIndex = i;
      }
    }

    if (bestIndex < 0) {
      Logger.warn('NativeHidController', `${TAG} 配置描述符中未找到可用 HID 输入接口，回退到设备对象枚举`);
      this.findEndpointsFromConfigs();
      return;
    }
    this.applyInterface(this.hidInterfaces[bestIndex], bestIndex, bestScore);
  }

  /** 配置描述符不可用时的最后兜底：按设备对象里的接口顺序挑一个 */
  private findEndpointsFromConfigs(): void {
    if (!this.device.configs || this.device.configs.length === 0) {
      return;
    }
    const interfaces = this.device.configs[0].interfaces;
    if (!interfaces || interfaces.length === 0) {
      return;
    }
    let bestIndex: number = -1;
    let bestScore: number = -1000;
    for (let i = 0; i < interfaces.length; i++) {
      const iface = interfaces[i];
      const eps: usbManager.USBEndpoint[] = iface.endpoints ? iface.endpoints : new Array<usbManager.USBEndpoint>();
      const isHid: boolean = iface.clazz === UsbClass.USB_CLASS_HID;
      const isXinputVendor: boolean = iface.clazz === UsbClass.USB_CLASS_VENDOR_SPEC &&
        iface.subClass === 93 && iface.protocol === 1;
      if ((!isHid && !isXinputVendor) || eps.length === 0) {
        continue;
      }
      if (isHid && iface.subClass === NativeHidController.HID_SUBCLASS_BOOT &&
          (iface.protocol === 0 || iface.protocol === 1 || iface.protocol === 2)) {
        continue;
      }
      let score: number = isXinputVendor && this.productId === 0x0037 ? 10000 : 0;
      for (let j = 0; j < eps.length; j++) {
        if ((eps[j].direction & 0x80) !== 0) {
          score += 10 + eps[j].maxPacketSize;
        }
      }
      if (score > bestScore) {
        bestScore = score;
        bestIndex = i;
      }
    }
    if (bestIndex < 0) {
      Logger.warn('NativeHidController', `${TAG} 未找到可用的 HID 输入接口`);
      return;
    }
    const selected = interfaces[bestIndex];
    if (this.productId === 0x0037 && selected.clazz === UsbClass.USB_CLASS_VENDOR_SPEC) {
      this.isXinputFixedReport = true;
      Logger.info('NativeHidController', `${TAG} 兜底路径确认 XInput 模式，启用固定 X360 报告解析`);
    }
    this.applyInterfaceFromObject(selected, bestIndex, bestScore);
  }

  /** 按接口 id 找到 device.configs[0].interfaces 中的下标 */
  private findInterfaceIndex(interfaceId: number): number {
    if (!this.device.configs || this.device.configs.length === 0) {
      return -1;
    }
    const interfaces = this.device.configs[0].interfaces;
    if (!interfaces) {
      return -1;
    }
    for (let i = 0; i < interfaces.length; i++) {
      if (interfaces[i].id === interfaceId) {
        return i;
      }
    }
    return -1;
  }

  /** 按配置描述符解析结果声明接口、配置端点 */
  private applyInterface(info: HidInterfaceInfo, candidateIndex: number, score: number): boolean {
    if (!this.device.configs || this.device.configs.length === 0) {
      return false;
    }
    const interfaces = this.device.configs[0].interfaces;
    if (!interfaces) {
      return false;
    }
    let targetIndex: number = -1;
    for (let i = 0; i < interfaces.length; i++) {
      if (interfaces[i].id === info.interfaceId) {
        targetIndex = i;
        break;
      }
    }
    if (targetIndex < 0) {
      Logger.warn('NativeHidController', `${TAG} 找不到接口 id=${info.interfaceId} 对应的 USBInterface`);
      return false;
    }
    return this.applyInterfaceFromObject(interfaces[targetIndex], candidateIndex, score);
  }

  private applyInterfaceFromObject(iface: usbManager.USBInterface, candidateIndex: number, score: number): boolean {
    if (this.claimedInterfaceId >= 0 && this.claimedInterfaceId !== iface.id) {
      this.releaseClaimedInterface();
    }
    try {
      const ret: number = usbManager.claimInterface(this.pipe, iface, true);
      Logger.info('NativeHidController', `${TAG} 声明接口 (id=${iface.id}, 候选=${candidateIndex}, score=${score}) ret=${ret}`);
    } catch (err) {
      Logger.error('NativeHidController', `${TAG} 声明接口失败:`, err);
      return false;
    }
    this.claimedInterfaceId = iface.id;
    this.interfaceId = iface.id;

    const eps: usbManager.USBEndpoint[] = iface.endpoints ? iface.endpoints : new Array<usbManager.USBEndpoint>();
    this.inputEndpoint = 0;
    this.outputEndpoint = 0;
    for (let j = 0; j < eps.length; j++) {
      const ep = eps[j];
      if ((ep.direction & 0x80) !== 0) {
        if (this.inputEndpoint === 0) {
          this.inputEndpoint = ep.address;
          this.inputMaxPacketSize = ep.maxPacketSize;
        }
      } else if (this.outputEndpoint === 0) {
        this.outputEndpoint = ep.address;
      }
    }
    if (this.inputMaxPacketSize <= 0) {
      this.inputMaxPacketSize = 64;
    }
    Logger.info('NativeHidController', `${TAG} 端点配置: IN=0x${this.inputEndpoint.toString(16)}/${this.inputMaxPacketSize}B, OUT=0x${this.outputEndpoint.toString(16)}`);
    return true;
  }

  private releaseClaimedInterface(): void {
    if (this.claimedInterfaceId < 0 || !this.device.configs || this.device.configs.length === 0) {
      return;
    }
    const interfaces = this.device.configs[0].interfaces;
    if (!interfaces) {
      return;
    }
    for (let i = 0; i < interfaces.length; i++) {
      if (interfaces[i].id === this.claimedInterfaceId) {
        try {
          usbManager.releaseInterface(this.pipe, interfaces[i]);
          Logger.info('NativeHidController', `${TAG} 已释放接口 id=${this.claimedInterfaceId}`);
        } catch (err) {
          Logger.warn('NativeHidController', `${TAG} 释放接口异常:`, err);
        }
        break;
      }
    }
    this.claimedInterfaceId = -1;
  }

  /**
   * GET_DESCRIPTOR(HID Report) 控制传输。
   * 配置描述符只给出报告描述符的长度，内容必须通过这条请求读取。
   */
  private async readHidReportDescriptor(interfaceId: number, length: number): Promise<Uint8Array | null> {
    const size: number = Math.max(1, Math.min(length, 1024));

    // API 12+：bmRequestType 可完整指定 (IN | STANDARD | INTERFACE) = 0x81
    try {
      const buf = new Uint8Array(size);
      const params: usbManager.USBDeviceRequestParams = {
        bmRequestType: 0x81,
        bRequest: 0x06,       // GET_DESCRIPTOR
        wValue: 0x2200,       // HID Report Descriptor
        wIndex: interfaceId,
        wLength: size,
        data: buf
      };
      const ret: number = await usbManager.usbControlTransfer(this.pipe, params, 300);
      if (ret > 0) {
        return buf.slice(0, Math.min(ret, size));
      }
      Logger.warn('NativeHidController', `${TAG} 接口 id=${interfaceId} usbControlTransfer 返回 ${ret}`);
    } catch (err) {
      Logger.warn('NativeHidController', `${TAG} 接口 id=${interfaceId} usbControlTransfer 失败:`, err);
    }

    // 兼容旧 API（仅当新接口不可用或失败时）
    try {
      const buf = new Uint8Array(size);
      const params: usbManager.USBControlParams = {
        request: 0x06,
        target: usbManager.USBRequestTargetType.USB_REQUEST_TARGET_INTERFACE,
        reqType: usbManager.USBControlRequestType.USB_REQUEST_TYPE_STANDARD,
        value: 0x2200,
        index: interfaceId,
        data: buf
      };
      const ret: number = await usbManager.controlTransfer(this.pipe, params, 300);
      if (ret > 0) {
        return buf.slice(0, Math.min(ret, size));
      }
      Logger.warn('NativeHidController', `${TAG} 接口 id=${interfaceId} controlTransfer 返回 ${ret}`);
    } catch (err) {
      Logger.warn('NativeHidController', `${TAG} 接口 id=${interfaceId} controlTransfer 失败:`, err);
    }
    return null;
  }

  /**
   * 异步校准：逐个读取候选接口的真实 HID 报告描述符，按「含多少游戏手柄字段」
   * 评分选出主输入接口，必要时释放旧接口并重新声明。
   * 失败时保持 hidLayout = null，回退到原有解析路径，不会让情况变差。
   */
  private async initializeDescriptorsAsync(): Promise<void> {
    if (!this.running) {
      return;
    }
    if (this.isXinputFixedReport) {
      // XInput 接口没有 HID 描述符；临时 claim/release 其它接口只会干扰数据通道。
      Logger.info('NativeHidController', `${TAG} XInput 模式跳过 HID 报告描述符校准`);
      return;
    }
    let bestLayout: HidReportLayout | null = null;
    let bestInfo: HidInterfaceInfo | null = null;
    let bestIndex: number = -1;
    let bestScore: number = 0;

    for (let i = 0; i < this.hidInterfaces.length; i++) {
      if (!this.running) {
        return;
      }
      const info: HidInterfaceInfo = this.hidInterfaces[i];
      if (info.clazz !== UsbClass.USB_CLASS_HID || info.inputEndpoint === 0 || info.reportDescriptorLength <= 0) {
        continue;
      }

      // HarmonyOS 只允许对「已声明」的接口做接口级控制传输（实测未声明接口一律 -1）。
      // 因此读取每个候选接口前临时 claim，读完立刻释放，尽量不影响系统 HID 驱动。
      let tempIndex: number = -1;
      let tempClaimed: boolean = false;
      if (info.interfaceId !== this.claimedInterfaceId) {
        tempIndex = this.findInterfaceIndex(info.interfaceId);
        if (tempIndex >= 0) {
          const cfg = this.device.configs[0];
          const ifaces = cfg.interfaces;
          if (ifaces) {
            try {
              usbManager.claimInterface(this.pipe, ifaces[tempIndex], true);
              tempClaimed = true;
              Logger.info('NativeHidController', `${TAG} 临时声明接口 id=${info.interfaceId} 以读取报告描述符`);
            } catch (err) {
              Logger.warn('NativeHidController', `${TAG} 临时声明接口 id=${info.interfaceId} 失败:`, err);
            }
          }
        }
      }

      const desc: Uint8Array | null = await this.readHidReportDescriptor(info.interfaceId, info.reportDescriptorLength);

      if (tempClaimed && tempIndex >= 0) {
        const cfg2 = this.device.configs[0];
        const ifaces2 = cfg2.interfaces;
        if (ifaces2) {
          try {
            usbManager.releaseInterface(this.pipe, ifaces2[tempIndex]);
          } catch (err) {
            Logger.warn('NativeHidController', `${TAG} 释放临时接口 id=${info.interfaceId} 异常:`, err);
          }
        }
      }
      if (desc === null) {
        continue;
      }
      Logger.info('NativeHidController', `${TAG} 接口 id=${info.interfaceId} 报告描述符(${desc.length}B): ${hexDump(desc, 256)}`);
      const layout: HidReportLayout | null = parseHidReportDescriptor(desc);
      if (layout === null) {
        Logger.warn('NativeHidController', `${TAG} 接口 id=${info.interfaceId} 未解析出手柄输入报告`);
        continue;
      }
      const score: number = scoreHidLayout(layout);
      Logger.info('NativeHidController', `${TAG} 接口 id=${info.interfaceId} 手柄评分=${score} ${layout.description}`);
      if (score > bestScore) {
        bestScore = score;
        bestLayout = layout;
        bestInfo = info;
        bestIndex = i;
      }
    }

    if (bestLayout === null || bestInfo === null) {
      // 少数平台会把报告描述符内容直接内嵌在 raw 描述符链里，最后试一次
      try {
        const raw: Uint8Array = usbManager.getRawDescriptor(this.pipe);
        const embedded: HidReportLayout | null = parseRawUsbDescriptors(raw);
        if (embedded !== null) {
          applyKishiUltraButtonMap(embedded, this.vendorId, this.productId);
          this.hidLayout = embedded;
          Logger.info('NativeHidController', `${TAG} 已启用内嵌 HID 描述符动态解析: ${embedded.description}`);
          return;
        }
      } catch (err) {
        Logger.warn('NativeHidController', `${TAG} 读取内嵌描述符失败:`, err);
      }
      // 设备可能在厂商自定义 HID 接口上发送输入（Kishi 的 vendor 通道）。
      // 没有标准 Game Pad 报告时切过去，至少保证能收到原始输入帧用于后续适配。
      let vendorIndex: number = -1;
      for (let k = 0; k < this.hidInterfaces.length; k++) {
        const cand: HidInterfaceInfo = this.hidInterfaces[k];
        if (cand.clazz === UsbClass.USB_CLASS_HID && cand.subClass === 0 && cand.protocol === 0 &&
            cand.inputEndpoint !== 0 && cand.inputMaxPacketSize >= 16) {
          vendorIndex = k;
          break;
        }
      }
      if (vendorIndex >= 0) {
        const vend: HidInterfaceInfo = this.hidInterfaces[vendorIndex];
        if (vend.interfaceId !== this.interfaceId) {
          Logger.warn('NativeHidController', `${TAG} 未找到标准游戏手柄报告，切换到厂商 HID 接口 id=${vend.interfaceId} 以接收输入帧`);
          this.applyInterface(vend, vendorIndex, 0);
        }
      }
      Logger.warn('NativeHidController', `${TAG} 未找到标准游戏手柄报告，使用原有解析路径`);
      return;
    }

    if (bestInfo.interfaceId !== this.interfaceId) {
      Logger.info('NativeHidController', `${TAG} 切换到含手柄报告的接口 id=${bestInfo.interfaceId}`);
      if (!this.applyInterface(bestInfo, bestIndex, bestScore)) {
        Logger.warn('NativeHidController', `${TAG} 切换接口失败，保留当前接口与原有解析路径`);
        return;
      }
    }
    // 部分手柄（Razer Kishi Ultra HID 模式）的按钮 Usage 与标准 gamepad 约定错位，
    // 依据真机逐键实测结果套用专用映射表；其它手柄保持标准映射。
    if (applyKishiUltraButtonMap(bestLayout, this.vendorId, this.productId)) {
      Logger.info('NativeHidController', `${TAG} 已启用 Kishi Ultra 实测按钮映射 (VID=0x${this.vendorId.toString(16)}, PID=0x${this.productId.toString(16)})`);
    }
    this.hidLayout = bestLayout;
    Logger.info('NativeHidController', `${TAG} 已启用 HID 描述符动态解析 (接口 id=${bestInfo.interfaceId}, 评分=${bestScore}): ${bestLayout.description}`);
  }

  start(): boolean {
    if (this.running) {
      return true;
    }

    if (this.inputEndpoint === 0) {
      Logger.error('NativeHidController', `${TAG} 无法启动: 未找到输入端点`);
      return false;
    }

    this.markInitializing();
    this.running = true;
    this.lastActivityTime = Date.now();
    this.notifyDeviceAdded();

    // 先异步读取真实 HID 报告描述符并校准接口，再开始轮询
    this.beginInputAsync();

    Logger.info('NativeHidController', `${TAG} 控制器已启动 (ArkTS 轮询): ${this.deviceName}`);
    return true;
  }

  stop(): void {
    if (!this.running) {
      return;
    }

    this.running = false;
    this.markStopped();

    this.notifyDeviceRemoved();

    // 释放 USB 接口，确保系统 HID 驱动能重新接管设备
    if (this.pipe && this.device) {
      try {
        const config = this.device.configs[0];
        if (config && config.interfaces) {
          let released = false;
          for (let i = 0; i < config.interfaces.length; i++) {
            const iface = config.interfaces[i];
            if (iface.clazz === 3 /* USB_CLASS_HID */ && iface.id === this.interfaceId) {
              usbManager.releaseInterface(this.pipe, iface);
              Logger.info('NativeHidController', `${TAG} 已释放 HID 接口 id=${iface.id}`);
              released = true;
              break;
            }
          }
          if (!released) {
            Logger.warn('NativeHidController', `${TAG} 未找到 HID 接口（共 ${config.interfaces.length} 个接口）`);
          }
        }
      } catch (err) {
        Logger.error('NativeHidController', `${TAG} 释放接口异常:`, err);
      }
    }

    try {
      usbManager.closePipe(this.pipe);
      Logger.info('NativeHidController', `${TAG} closePipe 成功`);
    } catch (err) {
      Logger.error('NativeHidController', `${TAG} closePipe 失败:`, err);
    }

    Logger.info('NativeHidController', `${TAG} 控制器已停止: ${this.deviceName}`);
  }
  
  /**
   * 检查控制器是否已连接
   */
  isConnected(): boolean {
    return this.running;
  }

  /**
   * 获取最后活动时间
   * 用于检测静默断开
   */
  getLastActivityTime(): number {
    return this.lastActivityTime;
  }

  /**
   * 检查控制器是否可能已静默断开
   * @param inactivityThresholdMs 不活动阈值（毫秒）
   * @returns true 表示可能已断开
   */
  mayBeSilentlyDisconnected(inactivityThresholdMs: number = 30000): boolean {
    if (!this.running) {
      return true;
    }
    const now = Date.now();
    return (now - this.lastActivityTime) > inactivityThresholdMs;
  }

  /** 描述符校准完成后再进入轮询，避免前几帧按错误布局解码 */
  private async beginInputAsync(): Promise<void> {
    try {
      await this.initializeDescriptorsAsync();
    } catch (err) {
      Logger.warn('NativeHidController', `${TAG} 描述符校准异常，使用原有解析路径:`, err);
    }
    if (this.running) {
      await this.inputLoop();
    }
  }

  /**
   * ArkTS 后备轮询循环（当 Native Poller 不可用时使用）
   */
  private async inputLoop(): Promise<void> {
    const buffer = new Uint8Array(this.inputMaxPacketSize);
    
    // USB 读取超时时间（与 Android 一致为 3000ms）
    // 用于判断错误是超时还是真正的 I/O 错误
    const BULK_READ_TIMEOUT = 3000;
    // 快速失败阈值：如果读取在超时之前就失败（耗时 < 1秒），认为是 I/O 错误
    const QUICK_FAIL_THRESHOLD = 1000;
    
    // 轮询率统计
    let pollCount = 0;
    let lastStatTime = Date.now();
    const STAT_INTERVAL_MS = 5000;  // 每 5 秒输出一次统计
    
    // 初始化最后活动时间
    this.lastActivityTime = Date.now();
    this.consecutiveTimeouts = 0;
    
    // 智能恢复状态
    let consecutiveErrors = 0;
    const MAX_RECOVERY_ERRORS = 50;  // ~10s
    let recovering = false;

    while (this.running) {
      // 记录读取开始时间（用于检测真正的 I/O 错误 vs 超时）
      const startTime = Date.now();
      
      try {
        const result = await this.bulkRead(buffer);
        if (result > 0) {
          pollCount++;
          // 重置超时计数器
          this.consecutiveTimeouts = 0;
          this.lastActivityTime = Date.now();
          
          // 恢复成功
          if (recovering) {
            Logger.info('NativeHidController', `${TAG} 设备已恢复，累计错误 ${consecutiveErrors} 次`);
            consecutiveErrors = 0;
            recovering = false;
          }
          
          this.processInputReport(buffer.slice(0, result));
          
          // 统计轮询率
          const now = Date.now();
          if (now - lastStatTime >= STAT_INTERVAL_MS) {
            const pollRate = (pollCount * 1000) / (now - lastStatTime);
            Logger.info('NativeHidController', `${TAG} 轮询率: ${pollRate.toFixed(1)} Hz (${pollCount} 次/${((now - lastStatTime) / 1000).toFixed(1)}秒)`);
            pollCount = 0;
            lastStatTime = now;
          }
        }
      } catch (err) {
        if (this.running) {
          const elapsedTime = Date.now() - startTime;
          const usbErr = err instanceof UsbError ? err : null;
          const errorCode = usbErr ? usbErr.code : 0;
          
          // v68: 判据从「错误码是不是 -8」改成「这次读是否跑满了超时」。
          //
          // 实测（11/12 22:10~22:35 多轮）：手柄空闲、不产生输入报告时，
          // 鸿蒙 USB 返回的是 **code=-1** 并附文案「I/O 错误 - 可能是真实的 USB
          // 硬件问题」，但 elapsed 稳定在 3010~3021ms —— 正好等于
          // BULK_READ_TIMEOUT(3000ms)。也就是说这就是**读超时**，只是错误码被
          // 包成了 -1。旧代码只认 -8，于是把「手柄空闲」一条条算成 I/O 错误，
          // 攒满 MAX_RECOVERY_ERRORS(50) 就 stop() + closePipe，连带把共用同一条
          // pipe 的触感接口一起作废 ⇒ 用户感知「放着不按键一阵子就彻底不震」。
          //
          // 真正的硬件错误（设备被拔、pipe 已失效）会**立即**返回，耗时远小于 1 秒。
          // 所以「耗时 ≥ 快速失败阈值」等价于「在等数据、没等到」，与错误码无关。
          const readTimedOut = elapsedTime >= QUICK_FAIL_THRESHOLD;
          if (readTimedOut) {
            // 没有数据 ≠ 断开：手柄空闲时本来就不发报告。
            // 判据只能是「设备还在不在 USB 列表里」：
            //   在   ⇒ 只是空闲，继续等，绝不判断开；
            //   不在 ⇒ 真的拔了，立刻按断开处理（5s 重扫也会兜底，不会漏）。
            this.consecutiveTimeouts++;
            if (this.consecutiveTimeouts >= NativeHidController.SILENT_DISCONNECT_TIMEOUT_COUNT) {
              if (!this.isDeviceStillPresent()) {
                Logger.warn('NativeHidController',
                  `${TAG} 静默断开确认: 连续 ${this.consecutiveTimeouts} 次读超时，且设备已不在 USB 列表`);
                this.buttonFlags = 0;
                this.leftStickX = 0;
                this.leftStickY = 0;
                this.rightStickX = 0;
                this.rightStickY = 0;
                this.leftTrigger = 0;
                this.rightTrigger = 0;
                this.reportInput();
                break;
              }
              // 只打一次，避免空闲时刷屏
              if (this.consecutiveTimeouts === NativeHidController.SILENT_DISCONNECT_TIMEOUT_COUNT) {
                Logger.info('NativeHidController',
                  `${TAG} 设备已连续 ${this.consecutiveTimeouts} 次读超时无数据，但仍在 USB 列表中（空闲状态），继续等待`);
              }
            }
            continue;
          }

          // 快速失败 = 真正的 I/O 错误 → 智能恢复
          consecutiveErrors++;
          
          if (consecutiveErrors >= 3 && !recovering) {
            recovering = true;
            Logger.warn('NativeHidController', `${TAG} 连续 ${consecutiveErrors} 次设备错误 (code=${errorCode}, elapsed=${elapsedTime}ms)，尝试恢复: ${err}`);
          }
          
          // 超过恢复上限 → 归零输入并放弃
          if (consecutiveErrors >= MAX_RECOVERY_ERRORS) {
            Logger.error('NativeHidController', `${TAG} 连续 ${consecutiveErrors} 次错误，恢复超时，停止控制器`);
            this.buttonFlags = 0;
            this.leftStickX = 0;
            this.leftStickY = 0;
            this.rightStickX = 0;
            this.rightStickY = 0;
            this.leftTrigger = 0;
            this.rightTrigger = 0;
            this.reportInput();
            break;
          }
          
          // 指数退避：50ms → 100ms → 200ms (cap)
          const backoffMs = Math.min(50 * Math.pow(2, Math.min(consecutiveErrors - 1, 2)), 200);
          await this.sleep(backoffMs);
        }
      }
    }
    
    // 根据错误类型决定是否通知设备移除
    if (this.running) {
      Logger.warn('NativeHidController', `${TAG} 由于设备断开/IO错误，自动停止控制器`);
      this.stop();
    }
  }

  /**
   * v68: 设备是否仍在 USB 设备列表中。
   *
   * 用来把「手柄空闲不发报告」和「设备真被拔掉」区分开 —— 这两种情况在
   * bulkTransfer 这一层的表现完全一样（都是超时），只能靠枚举列表判定。
   * 只比对 VID:PID:busNum，**不比对 devAddress**：重新枚举后 devAddress 会变，
   * 但对我们来说是同一台设备，不该因为地址变了就判断开。
   * 查询本身失败时按「还在」处理，宁可多等也不要误判断连。
   */
  private isDeviceStillPresent(): boolean {
    try {
      const devices = usbManager.getDevices();
      for (const dev of devices) {
        if (dev.vendorId === this.device.vendorId &&
            dev.productId === this.device.productId &&
            dev.busNum === this.device.busNum) {
          return true;
        }
      }
    } catch (_) {
      return true;
    }
    return false;
  }

  private async bulkRead(buffer: Uint8Array): Promise<number> {
    // 使用较长的超时（与 Android AbstractXboxController 一致为 3000ms）
    // 这不会影响轮询率，因为有数据时会立即返回
    // 较长超时可以减少无数据时的空循环开销
    // 同时用于判断是超时还是真正的 I/O 错误（参见 inputLoop 中的 QUICK_FAIL_THRESHOLD）
    const timeout: number = 3000;

    try {
      const endpoint: usbManager.USBEndpoint = {
        address: this.inputEndpoint,
        attributes: 3,
        interval: 1,
        maxPacketSize: this.inputMaxPacketSize,
        direction: 0x80 as usbManager.USBRequestDirection,
        interfaceId: this.interfaceId,
        number: this.inputEndpoint,
        type: 3
      };

      const ret: number = await usbManager.bulkTransfer(this.pipe, endpoint, buffer, timeout);
      
      // 负数表示错误
      if (ret < 0) {
        throw createUsbError(ret);
      }
      
      return ret;
    } catch (err) {
      throw err as Error;
    }
  }

  /**
   * XInput 模式专用：按标准 Xbox 360 固定报告布局解码。
   * 保留原始帧日志，便于后续确认扩展键（L4/R4）是否落在保留字节里。
   */
  private processXinputReport(data: Uint8Array): void {
    const decoded: HidGamepadState | null = decodeXbox360InputReport(data, this.xinputReportHasHeader);
    if (decoded === null) {
      if (this.otherReportLogCount < 12) {
        this.otherReportLogCount++;
        Logger.warn('NativeHidController', `${TAG} XInput 非输入帧(${data.length}B) 已忽略: ${hexDump(data, 32)}`);
      }
      return;
    }

    if (this.xinputReportHasHeader === null) {
      this.xinputReportHasHeader = data.length >= 16 && data[0] === 0x00 && data[1] === 0x14;
      Logger.info('NativeHidController', `${TAG} XInput 报告形态锁定: ${this.xinputReportHasHeader ? '带头 00 14' : '纯 payload'} (首帧 ${data.length}B)`);
    }

    if (this.xinputLoggedFrames < 6) {
      this.xinputLoggedFrames++;
      Logger.info('NativeHidController', `${TAG} XInput 原始帧(${data.length}B): ${hexDump(data, 24)} -> btn=0x${decoded.buttonFlags.toString(16)}`);
    }
    if (decoded.buttonFlags !== this.lastXinputButtonFlags && this.xinputButtonLogCount < 120) {
      this.lastXinputButtonFlags = decoded.buttonFlags;
      this.xinputButtonLogCount++;
      Logger.info('NativeHidController', `${TAG} XInput 按键帧#${this.xinputButtonLogCount}: ${hexDump(data, 24)} -> btn=0x${decoded.buttonFlags.toString(16)}`);
    }

    this.buttonFlags = decoded.buttonFlags;
    this.leftStickX = decoded.leftStickX;
    this.leftStickY = decoded.leftStickY;
    this.rightStickX = decoded.rightStickX;
    this.rightStickY = decoded.rightStickY;
    this.leftTrigger = decoded.leftTrigger;
    this.rightTrigger = decoded.rightTrigger;
    this.reportInput();
  }

  /**
   * 使用原生层处理 HID 报告
   */
  private processInputReport(data: Uint8Array): void {
    // 打印最初几帧原始报告，便于在动态解析不生效时据此继续适配
    if (this.rawLoggedFrames < 6) {
      this.rawLoggedFrames++;
      Logger.info('NativeHidController', `${TAG} 原始输入帧(${data.length}B): ${hexDump(data, 32)}`);
    }

    // XInput 模式：无 HID 报告描述符，走固定 20 字节 X360 报告解析
    if (this.isXinputFixedReport) {
      this.processXinputReport(data);
      return;
    }

    // 优先按 HID Report Descriptor 动态解码（适配报告布局任意的 HID 手柄）
    const layout: HidReportLayout | null = this.hidLayout;
    if (layout !== null) {
      const decoded: HidGamepadState | null = decodeHidReport(data, layout);
      if (decoded === null) {
        // 已启用动态布局时，其它 Report ID 的帧不能交给固定布局解析器，
        // 否则厂商/扩展报告会被误读成乱按键。直接忽略并记录原始帧供分析。
        if (this.otherReportLogCount < 12) {
          this.otherReportLogCount++;
          Logger.warn('NativeHidController', `${TAG} 非目标报告帧(${data.length}B) 已忽略: ${hexDump(data, 32)}`);
        }
        return;
      }
      this.buttonFlags = decoded.buttonFlags;
      this.leftStickX = decoded.leftStickX;
      this.leftStickY = decoded.leftStickY;
      this.rightStickX = decoded.rightStickX;
      this.rightStickY = decoded.rightStickY;
      this.leftTrigger = decoded.leftTrigger;
      this.rightTrigger = decoded.rightTrigger;
      if (this.hidLoggedFrames < 3) {
        this.hidLoggedFrames++;
        Logger.info('NativeHidController', `${TAG} 动态解析帧: ${hexDump(data, 24)} -> btn=0x${decoded.buttonFlags.toString(16)} L=(${decoded.leftStickX.toFixed(2)},${decoded.leftStickY.toFixed(2)}) R=(${decoded.rightStickX.toFixed(2)},${decoded.rightStickY.toFixed(2)})`);
      }
      // 记录每次按钮/Hat/扳机状态变化，用于确定物理键与 HID Usage 的对应关系。
      // 上限从 200 提到 2000：一次长时间串流里前面几十帧就能用满 200，
      // 导致后面按 L4/R4 等扩展键时完全没有日志可查。
      if (this.buttonLogCount < 2000) {
        const signature: string = describeHidReport(data, layout);
        if (signature !== this.lastButtonSignature) {
          this.lastButtonSignature = signature;
          this.buttonLogCount++;
          Logger.info('NativeHidController', `${TAG} 按键帧#${this.buttonLogCount}: ${hexDump(data, 24)} -> ${signature} flags=0x${decoded.buttonFlags.toString(16)}`);
        }
      }
      // 摇杆/扳机全量数值：按 LT 时若右摇杆真的被串了值，这里会直接显示。
      // 只在摇杆数值变化时打印，静止时不刷屏。
      if (this.stickLogCount < 600) {
        const stickSignature: string =
          `${decoded.leftStickX.toFixed(2)},${decoded.leftStickY.toFixed(2)},` +
          `${decoded.rightStickX.toFixed(2)},${decoded.rightStickY.toFixed(2)},` +
          `${decoded.leftTrigger.toFixed(2)},${decoded.rightTrigger.toFixed(2)}`;
        if (stickSignature !== this.lastStickSignature) {
          this.lastStickSignature = stickSignature;
          this.stickLogCount++;
          Logger.info('NativeHidController', `${TAG} 模拟量#${this.stickLogCount}: ` +
            `L=(${decoded.leftStickX.toFixed(2)},${decoded.leftStickY.toFixed(2)}) ` +
            `R=(${decoded.rightStickX.toFixed(2)},${decoded.rightStickY.toFixed(2)}) ` +
            `LT=${decoded.leftTrigger.toFixed(2)} RT=${decoded.rightTrigger.toFixed(2)}`);
        }
      }
      this.reportInput();
      return;
    }

    // 调用原生解析器，传入强制协议类型
    const state = this.nativeParser.parseHidReport(
      this.vendorId,
      this.productId,
      data,
      this.forceProtocolType
    );

    if (!state) {
      // 原生解析失败，使用 TypeScript 后备解析器
      const hexStr = Array.from(data.slice(0, Math.min(data.length, 16)))
        .map(b => b.toString(16).padStart(2, '0')).join(' ');
      this.fallbackParseHidReport(data, hexStr);
      return;
    }

    // 转换原生按钮标志到 Moonlight 格式
    this.buttonFlags = this.convertNativeButtons(state.buttons);

    // 转换摇杆值（原生层返回 -32768 到 32767，需要转换为 -1.0 到 1.0）
    this.leftStickX = state.leftStickX / AXIS_MAX;
    this.leftStickY = state.leftStickY / AXIS_MAX;
    this.rightStickX = state.rightStickX / AXIS_MAX;
    this.rightStickY = state.rightStickY / AXIS_MAX;

    // 转换扣机值（原生层返回 0-255，需要转换为 0.0 到 1.0）
    this.leftTrigger = state.leftTrigger / TRIGGER_MAX;
    this.rightTrigger = state.rightTrigger / TRIGGER_MAX;

    // 报告输入
    this.reportInput();
  }

  /**
   * TypeScript 后备 HID 解析器
   * 用于原生解析器不可用时的通用 HID 手柄解析
   *
   * 通用手柄格式 (10字节):
   * [0] = Report ID (通常是 0x01)
   * [1] = Left Stick X (0x00-0xFF, 0x80 = center)
   * [2] = Left Stick Y (0x00-0xFF, 0x80 = center)
   * [3] = Right Stick X (0x00-0xFF, 0x80 = center)
   * [4] = Right Stick Y (0x00-0xFF, 0x80 = center)
   * [5] = Triggers/misc (varies)
   * [6] = HAT/D-Pad (0x0F = center, 0-7 = directions)
   * [7] = Buttons byte 1 (function buttons: Start, Select, LS, RS, etc.)
   * [8] = Buttons byte 2 (face buttons: A, B, X, Y, LB, RB)
   * [9] = Reserved
   */
  private fallbackParseHidReport(data: Uint8Array, hexStr: string): void {
    if (data.length < 9) {
      Logger.warn('NativeHidController', `${TAG} HID 数据太短: ${data.length}`);
      return;
    }

    // 解析摇杆 (字节 1-4, 0x00-0xFF, 0x80=中心)
    // 转换为 -1.0 到 1.0
    this.leftStickX = (data[1] - 128) / 127.0;
    this.leftStickY = (data[2] - 128) / 127.0;
    this.rightStickX = (data[3] - 128) / 127.0;
    this.rightStickY = (data[4] - 128) / 127.0;

    // 限制范围
    this.leftStickX = Math.max(-1.0, Math.min(1.0, this.leftStickX));
    this.leftStickY = Math.max(-1.0, Math.min(1.0, this.leftStickY));
    this.rightStickX = Math.max(-1.0, Math.min(1.0, this.rightStickX));
    this.rightStickY = Math.max(-1.0, Math.min(1.0, this.rightStickY));

    // 解析 HAT/D-Pad (字节 6)
    // HAT 值: 0=Up, 1=UpRight, 2=Right, 3=DownRight, 4=Down, 5=DownLeft, 6=Left, 7=UpLeft, 0x0F=Center
    let flags = 0;
    const hat = data[6] & 0x0F;
    switch (hat) {
      case 0: // Up
        flags |= ButtonFlags.UP_FLAG;
        break;
      case 1: // Up-Right
        flags |= ButtonFlags.UP_FLAG | ButtonFlags.RIGHT_FLAG;
        break;
      case 2: // Right
        flags |= ButtonFlags.RIGHT_FLAG;
        break;
      case 3: // Down-Right
        flags |= ButtonFlags.DOWN_FLAG | ButtonFlags.RIGHT_FLAG;
        break;
      case 4: // Down
        flags |= ButtonFlags.DOWN_FLAG;
        break;
      case 5: // Down-Left
        flags |= ButtonFlags.DOWN_FLAG | ButtonFlags.LEFT_FLAG;
        break;
      case 6: // Left
        flags |= ButtonFlags.LEFT_FLAG;
        break;
      case 7: // Up-Left
        flags |= ButtonFlags.UP_FLAG | ButtonFlags.LEFT_FLAG;
        break;
      default: // 0x0F = center, no direction
        break;
    }

    // 解析按钮字节
    const btn1 = data[7]; // 功能按钮
    const btn2 = data[8]; // 面板按钮

    // VID 0x413D 手柄的按钮映射 (通用 HID 手柄格式):
    // 字节 7 (功能按钮):
    //   bit 0 = Select/Back
    //   bit 1 = Start
    //   bit 2 = Home/Guide
    //   bit 3 = Left Stick Click
    //   bit 4 = Right Stick Click
    //   bit 5 = LB
    //   bit 6 = RB
    //   bit 7 = LT (digital)
    //
    // 字节 8 (面板按钮):
    //   bit 0 = A
    //   bit 1 = B
    //   bit 2 = X
    //   bit 3 = Y

    // 功能按钮 (字节 7)
    if (btn1 & 0x01) flags |= ButtonFlags.BACK_FLAG;      // Select/Back
    if (btn1 & 0x02) flags |= ButtonFlags.PLAY_FLAG;      // Start
    if (btn1 & 0x04) flags |= ButtonFlags.SPECIAL_BUTTON_FLAG; // Home/Guide
    if (btn1 & 0x08) flags |= ButtonFlags.LS_CLK_FLAG;    // Left Stick Click
    if (btn1 & 0x10) flags |= ButtonFlags.RS_CLK_FLAG;    // Right Stick Click
    if (btn1 & 0x20) flags |= ButtonFlags.LB_FLAG;        // LB
    if (btn1 & 0x40) flags |= ButtonFlags.RB_FLAG;        // RB

    // 面板按钮 (字节 8)
    if (btn2 & 0x01) flags |= ButtonFlags.A_FLAG;   // A
    if (btn2 & 0x02) flags |= ButtonFlags.B_FLAG;   // B
    if (btn2 & 0x04) flags |= ButtonFlags.X_FLAG;   // X
    if (btn2 & 0x08) flags |= ButtonFlags.Y_FLAG;   // Y

    // 更新按钮标志
    this.buttonFlags = flags;

    // 报告输入
    this.reportInput();
  }

  /**
   * 转换原生按钮标志到 Moonlight 格式
   */
  private convertNativeButtons(nativeButtons: number): number {
    let flags = 0;

    // 方向键
    if (nativeButtons & NativeButtonFlags.UP) flags |= ButtonFlags.UP_FLAG;
    if (nativeButtons & NativeButtonFlags.DOWN) flags |= ButtonFlags.DOWN_FLAG;
    if (nativeButtons & NativeButtonFlags.LEFT) flags |= ButtonFlags.LEFT_FLAG;
    if (nativeButtons & NativeButtonFlags.RIGHT) flags |= ButtonFlags.RIGHT_FLAG;

    // 功能键
    if (nativeButtons & NativeButtonFlags.START) flags |= ButtonFlags.PLAY_FLAG;
    if (nativeButtons & NativeButtonFlags.BACK) flags |= ButtonFlags.BACK_FLAG;
    if (nativeButtons & NativeButtonFlags.LS_CLK) flags |= ButtonFlags.LS_CLK_FLAG;
    if (nativeButtons & NativeButtonFlags.RS_CLK) flags |= ButtonFlags.RS_CLK_FLAG;

    // 肩键
    if (nativeButtons & NativeButtonFlags.LB) flags |= ButtonFlags.LB_FLAG;
    if (nativeButtons & NativeButtonFlags.RB) flags |= ButtonFlags.RB_FLAG;

    // 面板按钮
    if (nativeButtons & NativeButtonFlags.A) flags |= ButtonFlags.A_FLAG;
    if (nativeButtons & NativeButtonFlags.B) flags |= ButtonFlags.B_FLAG;
    if (nativeButtons & NativeButtonFlags.X) flags |= ButtonFlags.X_FLAG;
    if (nativeButtons & NativeButtonFlags.Y) flags |= ButtonFlags.Y_FLAG;

    // 特殊按钮
    if (nativeButtons & NativeButtonFlags.HOME) flags |= ButtonFlags.SPECIAL_BUTTON_FLAG;

    return flags;
  }

  rumble(lowFreqMotor: number, highFreqMotor: number): void {
    if (this.outputEndpoint === 0) {
      return;
    }

    if (this.isXinputFixedReport) {
      // 标准 Xbox 360 震动包，XInput 模式没有原生解析器的厂商命令
      const low: number = (this.clampRumbleValue(lowFreqMotor) >> 8) & 0xFF;
      const high: number = (this.clampRumbleValue(highFreqMotor) >> 8) & 0xFF;
      this.sendOutputReport(new Uint8Array([0x00, 0x08, 0x00, low, high, 0x00, 0x00, 0x00]));
      return;
    }

    // Native parser expects Moonlight rumble values in the same 0..65535 range as AbstractController.
    const cmd = this.nativeParser.createRumbleCommand(
      this.vendorId,
      this.productId,
      this.clampRumbleValue(lowFreqMotor),
      this.clampRumbleValue(highFreqMotor)
    );

    if (cmd) {
      this.sendOutputReport(cmd);
    }
  }

  private clampRumbleValue(value: number): number {
    if (!Number.isFinite(value)) return 0;
    return Math.max(0, Math.min(65535, Math.round(value)));
  }

  private async sendOutputReport(data: Uint8Array): Promise<void> {
    if (this.outputEndpoint === 0) {
      return;
    }

    try {
      const endpoint: usbManager.USBEndpoint = {
        address: this.outputEndpoint,
        attributes: 3,
        interval: 1,
        maxPacketSize: 64,
        direction: 0x00 as usbManager.USBRequestDirection,
        interfaceId: this.interfaceId,
        number: this.outputEndpoint,
        type: 3
      };

      await usbManager.bulkTransfer(this.pipe, endpoint, data, 100);
    } catch (err) {
      Logger.error('NativeHidController', `${TAG} 发送输出报告失败:`, err);
    }
  }

  rumbleTriggers(leftTrigger: number, rightTrigger: number): void {
    // 大多数手柄不支持扳机震动
  }
}
