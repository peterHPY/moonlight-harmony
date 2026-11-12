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
 * 通用 HID 报告描述符解析器
 *
 * 背景：原先的通用手柄解析把输入报告假定为「固定 10 字节 + 固定按键位」，
 * 这只对某几款手柄成立；遇到 Kishi Ultra 这类报告布局不同的设备就会按键全乱。
 *
 * 本模块从 usbManager.getRawDescriptor(pipe) 返回的原始 USB 描述符里提取
 * HID Report Descriptor (bDescriptorType = 0x22)，按 HID 规范解析 short items，
 * 得到每个字段在输入报告中的位偏移 / 位宽 / Usage / 逻辑范围，再据此解码。
 * 这样任何遵循 HID 规范的手柄都能得到正确的摇杆、方向键与按钮位置。
 *
 * 解析失败时返回 null，调用方回退到原有解析路径，不会让情况变差。
 */

import { ButtonFlags } from './ControllerConstants';

/** HID Usage Page */
const PAGE_GENERIC_DESKTOP: number = 0x01;
const PAGE_BUTTON: number = 0x09;
const PAGE_SIMULATION: number = 0x02;

/** Simulation Controls Usage：手柄常把模拟扳机声明为刹车/油门 */
const USAGE_ACCELERATOR: number = 0xC4;
const USAGE_BRAKE: number = 0xC5;

/** Generic Desktop Usage */
const USAGE_X: number = 0x30;
const USAGE_Y: number = 0x31;
const USAGE_Z: number = 0x32;
const USAGE_RX: number = 0x33;
const USAGE_RY: number = 0x34;
const USAGE_RZ: number = 0x35;
const USAGE_HAT: number = 0x39;

/** HID item 类型 */
const TYPE_MAIN: number = 0;
const TYPE_GLOBAL: number = 1;
const TYPE_LOCAL: number = 2;

/** HID Global item tags */
const TAG_USAGE_PAGE: number = 0;
const TAG_LOGICAL_MIN: number = 1;
const TAG_LOGICAL_MAX: number = 2;
const TAG_REPORT_SIZE: number = 7;
const TAG_REPORT_ID: number = 8;
const TAG_REPORT_COUNT: number = 9;

/** HID Local item tags */
const TAG_USAGE: number = 0;
const TAG_USAGE_MIN: number = 1;
const TAG_USAGE_MAX: number = 2;

/** HID Main item tags */
const TAG_INPUT: number = 8;
const TAG_OUTPUT: number = 9;
const TAG_COLLECTION: number = 10;
const TAG_FEATURE: number = 11;
const TAG_END_COLLECTION: number = 12;

/** 单个输入字段的描述 */
export interface HidField {
  bitOffset: number;   // 相对输入报告起始（不含 report ID 字节）的位偏移
  bitSize: number;
  usagePage: number;
  usage: number;
  logicalMin: number;
  logicalMax: number;
  isRelative: boolean;
  reportId: number;
}

/** 选定后的输入报告布局 */
export interface HidReportLayout {
  fields: HidField[];
  reportId: number;    // 0 表示报告不带 ID 前缀
  reportBits: number;
  description: string; // 诊断日志用
  appUsagePage: number; // 顶层 Application Collection 的 Usage Page
  appUsage: number;     // 顶层 Application Collection 的 Usage
  /**
   * 按钮 Usage -> Moonlight 按钮位的覆盖表；null 表示使用标准 HID gamepad 约定。
   * 索引即 HID Button Usage（1..18），值为 0 表示该按钮忽略。
   * 某些手柄（如 Razer Kishi Ultra）的按钮编号与标准约定错位，需专用映射。
   */
  buttonMap: number[] | null;
}

/** 解码后的手柄状态（已映射到 Moonlight 语义） */
export interface HidGamepadState {
  buttonFlags: number;
  leftStickX: number;   // -1 .. 1
  leftStickY: number;
  rightStickX: number;
  rightStickY: number;
  leftTrigger: number;  // 0 .. 1
  rightTrigger: number;
}

interface HidUsage {
  page: number;
  id: number;
}

/** 解析过程中的状态机 */
class HidParseContext {
  usagePage: number = 0;
  logicalMin: number = 0;
  logicalMax: number = 0;
  reportSize: number = 0;
  reportCount: number = 0;
  reportId: number = 0;
  usages: number[] = [];
  usageMin: number = -1;
  usageMax: number = -1;
  inputBitPos: Map<number, number> = new Map<number, number>();
  fields: HidField[] = [];
  /** 当前 Collection 嵌套深度 */
  collectionDepth: number = 0;
  /** 最外层 Application Collection 的 Usage（0/0 表示描述符未声明） */
  appUsagePage: number = 0;
  appUsage: number = 0;
  /** reportId -> (usagePage << 16 | usage)，用于排除触摸屏/键盘等非手柄报告 */
  reportAppUsage: Map<number, number> = new Map<number, number>();

  resetLocal(): void {
    this.usages = [];
    this.usageMin = -1;
    this.usageMax = -1;
  }

  /** 第 index 个字段对应的 Usage（支持 Usage 列表与 Usage Min/Max 范围） */
  resolveUsage(index: number): HidUsage {
    let raw: number = 0;
    if (this.usageMin >= 0 && this.usageMax >= this.usageMin) {
      const span: number = this.usageMax - this.usageMin;
      raw = this.usageMin + Math.min(index, span);
    } else if (this.usages.length > 0) {
      raw = this.usages[Math.min(index, this.usages.length - 1)];
    }
    // 32-bit Usage 时高 16 位自带 Usage Page
    const embeddedPage: number = (raw >>> 16) & 0xFFFF;
    return {
      page: embeddedPage !== 0 ? embeddedPage : this.usagePage,
      id: raw & 0xFFFF
    };
  }
}

/** 读取 item 的数据部分（小端；size 为 0/1/2/4） */
function itemValue(desc: Uint8Array, offset: number, size: number): number {
  let value: number = 0;
  for (let i = 0; i < size; i++) {
    if (offset + i >= desc.length) break;
    value += desc[offset + i] * Math.pow(256, i);
  }
  return value;
}

/** 把按位读出的无符号值按 logicalMin 做补码符号扩展 */
function signExtend(value: number, bitSize: number, logicalMin: number): number {
  if (logicalMin >= 0 || bitSize <= 0 || bitSize > 31) {
    return value;
  }
  const signBit: number = Math.pow(2, bitSize - 1);
  if (value >= signBit) {
    return value - Math.pow(2, bitSize);
  }
  return value;
}

/** 从报告中读取任意位宽字段（HID 采用 LSB-first 位打包） */
function readBits(data: Uint8Array, startByte: number, bitOffset: number, bitSize: number): number {
  let value: number = 0;
  for (let b = 0; b < bitSize; b++) {
    const bit: number = bitOffset + b;
    const idx: number = startByte + Math.floor(bit / 8);
    if (idx >= data.length) break;
    const one: number = (data[idx] >> (bit & 7)) & 1;
    if (one !== 0) {
      value += Math.pow(2, b);
    }
  }
  return value;
}

/** 把逻辑值归一到 -1..1（带轻微死区） */
function normalizeAxis(value: number, logicalMin: number, logicalMax: number): number {
  if (logicalMax <= logicalMin) return 0;
  let n: number = (value - logicalMin) / (logicalMax - logicalMin);
  if (n < 0) n = 0;
  if (n > 1) n = 1;
  let v: number = n * 2 - 1;
  if (Math.abs(v) < 0.02) v = 0;
  return v;
}

/** 把逻辑值归一到 0..1 */
function normalizeTrigger(value: number, logicalMin: number, logicalMax: number): number {
  if (logicalMax <= logicalMin) return 0;
  let n: number = (value - logicalMin) / (logicalMax - logicalMin);
  if (n < 0) n = 0;
  if (n > 1) n = 1;
  return n;
}

/**
 * Razer Kishi Ultra HID 模式实测按钮映射（2026-11-10 真机逐个按键验证）。
 *
 * 描述符声明 18 个按钮（Usage 1..18），但物理编号与 HID gamepad 标准约定错位：
 *   u1=A u2=B u4=X u5=Y u7=LB u8=RB u9=LT(数字) u10=RT(数字)
 *   u11=View/Back u12=Menu/Start u13=Home/Guide(推测)
 *   u14=L3 u15=R3，Hat 0=上。
 * LT/RT 同时有 Simulation Controls 页的 8bit 模拟量，数字位不再映射为按钮，
 * 否则会被误报成「按下左/右摇杆」。
 *
 * 2026-11-10 22:38 真机补测：用户按 L4/R4 时，日志里出现且仅出现 u3/u6 两个
 * 独立按下的位（帧 ... 08 48 00 ... / ... 08 02 00 ...，flags 恒为 0x0）。
 * 因此 L4/R4 = u3/u6，此前推断的 u16..u18 从未被按下，属保留位。
 * 这几个扩展位统一映射为 Moonlight 的 TOUCHPAD_FLAG：主机模拟 DS5 时即为
 * 「触摸板按下」，主机是其它类型时由主机按自身 supportedButtons 忽略。
 */
const KISHI_ULTRA_BUTTON_MAP: number[] = [
  0,
  ButtonFlags.A_FLAG,             // u1  A
  ButtonFlags.B_FLAG,             // u2  B
  ButtonFlags.TOUCHPAD_FLAG,      // u3  L4 背键 → DS5 触摸板按下（真机实测）
  ButtonFlags.X_FLAG,             // u4  X
  ButtonFlags.Y_FLAG,             // u5  Y
  ButtonFlags.TOUCHPAD_FLAG,      // u6  R4 背键 → DS5 触摸板按下（真机实测）
  ButtonFlags.LB_FLAG,            // u7  LB
  ButtonFlags.RB_FLAG,            // u8  RB
  0,                              // u9  LT 数字位（由模拟扳机代替）
  0,                              // u10 RT 数字位（由模拟扳机代替）
  ButtonFlags.BACK_FLAG,          // u11 View / Back（用户实测：与 Start 对调）
  ButtonFlags.PLAY_FLAG,          // u12 Menu / Start（用户实测：与 Back 对调）
  ButtonFlags.SPECIAL_BUTTON_FLAG,// u13 Home / Guide（推测）
  ButtonFlags.LS_CLK_FLAG,        // u14 左摇杆按下 L3
  ButtonFlags.RS_CLK_FLAG,        // u15 右摇杆按下 R3
  ButtonFlags.TOUCHPAD_FLAG,      // u16 保留扩展位，兜底映射
  ButtonFlags.TOUCHPAD_FLAG,      // u17 保留扩展位，兜底映射
  ButtonFlags.TOUCHPAD_FLAG       // u18 保留扩展位，兜底映射
];

/**
 * 判断一份布局是否为 Kishi Ultra 这类「按钮语义错位」的报告：
 * 至少 12 个按钮位、带 Simulation Controls 页的模拟扳机。
 * 只有同时满足设备 VID/PID 与布局特征才启用覆盖表，不影响其它手柄。
 */
export function applyKishiUltraButtonMap(layout: HidReportLayout, vendorId: number, productId: number): boolean {
  if (vendorId !== 0x1532) {
    return false;
  }
  let buttonCount: number = 0;
  let hasSimTrigger: boolean = false;
  for (const f of layout.fields) {
    if (f.usagePage === PAGE_BUTTON && f.bitSize === 1) {
      buttonCount++;
    } else if (f.usagePage === PAGE_SIMULATION &&
      (f.usage === USAGE_BRAKE || f.usage === USAGE_ACCELERATOR)) {
      hasSimTrigger = true;
    }
  }
  if (buttonCount < 12 || !hasSimTrigger) {
    return false;
  }
  layout.buttonMap = KISHI_ULTRA_BUTTON_MAP;
  return true;
}

/** 读取小端 16 位有符号数（ArkTS 无 DataView 依赖，直接按字节拼装） */
function readInt16LE(buf: Uint8Array, at: number): number {
  const raw: number = buf[at] | (buf[at + 1] << 8);
  return raw > 32767 ? raw - 65536 : raw;
}

function clampSigned(value: number): number {
  if (value < -1) return -1;
  if (value > 1) return 1;
  return value;
}

/**
 * 解码标准 Xbox 360 / XInput 有线手柄的固定输入报告。
 *
 * Razer Kishi Ultra 切到 XInput 模式（PID 0x0037）后暴露 0xFF/93/1 厂商接口，
 * 不提供 HID Report Descriptor，无法走动态解析；但它的输入就是标准 X360
 * 20 字节格式，直接按固定偏移解码即可。该路径与 HID 模式完全隔离。
 *
 * 兼容两种形态：
 *   [0x00, 0x14, <18B payload>]  标准 xpad 的 2 字节报告头
 *   [<18B payload>]              部分平台直接给 payload
 *
 * payload 布局（相对 offset）：
 *   0  D-Pad/Start/Back/L3/R3   1  ABXY/LB/RB/Guide
 *   2  LT   3  RT   4..5 LX   6..7 LY   8..9 RX   10..11 RY
 *   12..17 保留
 *
 * reportHasHeader 传 null 时自动判定（首帧一般是空闲帧，形态明确）；
 * 调用方应把首帧判定结果锁定后传给后续帧——否则「dpad=0 + A + Guide」
 * 的纯 payload 帧恰好也是 00 14 开头，会被误剥 2 字节导致按键乱码。
 */
export function decodeXbox360InputReport(data: Uint8Array, reportHasHeader: boolean | null): HidGamepadState | null {
  if (data.length < 14) {
    return null;
  }
  let offset: number = 0;
  const looksLikeHeader: boolean = data.length >= 16 && data[0] === 0x00 && data[1] === 0x14;
  const hasHeader: boolean = reportHasHeader === null ? looksLikeHeader : reportHasHeader;
  if (hasHeader) {
    offset = 2;
  }
  if (data.length - offset < 14) {
    return null;
  }

  const dpad: number = data[offset];
  const face: number = data[offset + 1];
  let flags: number = 0;

  if ((dpad & 0x01) !== 0) flags |= ButtonFlags.UP_FLAG;
  if ((dpad & 0x02) !== 0) flags |= ButtonFlags.DOWN_FLAG;
  if ((dpad & 0x04) !== 0) flags |= ButtonFlags.LEFT_FLAG;
  if ((dpad & 0x08) !== 0) flags |= ButtonFlags.RIGHT_FLAG;
  if ((dpad & 0x10) !== 0) flags |= ButtonFlags.PLAY_FLAG;
  if ((dpad & 0x20) !== 0) flags |= ButtonFlags.BACK_FLAG;
  if ((dpad & 0x40) !== 0) flags |= ButtonFlags.LS_CLK_FLAG;
  if ((dpad & 0x80) !== 0) flags |= ButtonFlags.RS_CLK_FLAG;

  if ((face & 0x10) !== 0) flags |= ButtonFlags.A_FLAG;
  if ((face & 0x20) !== 0) flags |= ButtonFlags.B_FLAG;
  if ((face & 0x40) !== 0) flags |= ButtonFlags.X_FLAG;
  if ((face & 0x80) !== 0) flags |= ButtonFlags.Y_FLAG;
  if ((face & 0x01) !== 0) flags |= ButtonFlags.LB_FLAG;
  if ((face & 0x02) !== 0) flags |= ButtonFlags.RB_FLAG;
  if ((face & 0x04) !== 0) flags |= ButtonFlags.SPECIAL_BUTTON_FLAG;

  const leftTrigger: number = data[offset + 2] / 255;
  const rightTrigger: number = data[offset + 3] / 255;
  const leftStickX: number = clampSigned(readInt16LE(data, offset + 4) / 32768);
  const leftStickY: number = clampSigned(-readInt16LE(data, offset + 6) / 32768);
  const rightStickX: number = clampSigned(readInt16LE(data, offset + 8) / 32768);
  const rightStickY: number = clampSigned(-readInt16LE(data, offset + 10) / 32768);

  return {
    buttonFlags: flags,
    leftStickX: leftStickX,
    leftStickY: leftStickY,
    rightStickX: rightStickX,
    rightStickY: rightStickY,
    leftTrigger: leftTrigger,
    rightTrigger: rightTrigger
  };
}

/** 按钮 Usage -> Moonlight 按钮位（HID Gamepad 的标准按钮序号约定） */
function buttonFlagForUsage(usage: number): number {
  switch (usage) {
    case 1: return ButtonFlags.A_FLAG;
    case 2: return ButtonFlags.B_FLAG;
    case 3: return ButtonFlags.X_FLAG;
    case 4: return ButtonFlags.Y_FLAG;
    case 5: return ButtonFlags.LB_FLAG;
    case 6: return ButtonFlags.RB_FLAG;
    case 7: return ButtonFlags.BACK_FLAG;
    case 8: return ButtonFlags.PLAY_FLAG;
    case 9: return ButtonFlags.LS_CLK_FLAG;
    case 10: return ButtonFlags.RS_CLK_FLAG;
    case 11: return ButtonFlags.SPECIAL_BUTTON_FLAG;
    case 12: return ButtonFlags.MISC_FLAG;
    case 13: return ButtonFlags.PADDLE1_FLAG;
    case 14: return ButtonFlags.PADDLE2_FLAG;
    case 15: return ButtonFlags.PADDLE3_FLAG;
    case 16: return ButtonFlags.PADDLE4_FLAG;
    default: return 0;
  }
}

/** Hat switch 的 8 方向 -> 方向键位 */
function hatFlags(direction: number): number {
  switch (direction) {
    case 0: return ButtonFlags.UP_FLAG;
    case 1: return ButtonFlags.UP_FLAG | ButtonFlags.RIGHT_FLAG;
    case 2: return ButtonFlags.RIGHT_FLAG;
    case 3: return ButtonFlags.DOWN_FLAG | ButtonFlags.RIGHT_FLAG;
    case 4: return ButtonFlags.DOWN_FLAG;
    case 5: return ButtonFlags.DOWN_FLAG | ButtonFlags.LEFT_FLAG;
    case 6: return ButtonFlags.LEFT_FLAG;
    case 7: return ButtonFlags.UP_FLAG | ButtonFlags.LEFT_FLAG;
    default: return 0;
  }
}

/** 统计某个 report 里「像游戏手柄」的字段数量，用于选出主输入报告 */
function gamepadScore(fields: HidField[]): number {
  let score: number = 0;
  const seenAxes: number[] = [];
  for (const f of fields) {
    if (f.usagePage === PAGE_BUTTON && f.usage >= 1 && f.bitSize === 1) {
      score += 1;
    } else if (f.usagePage === PAGE_GENERIC_DESKTOP) {
      // 触摸屏会为多个触点重复声明 X/Y；同一个 Usage 只计一次，避免刷分
      if (seenAxes.indexOf(f.usage) >= 0) {
        continue;
      }
      if (f.usage === USAGE_HAT) {
        score += 6;
        seenAxes.push(f.usage);
      } else if (f.usage >= USAGE_X && f.usage <= USAGE_RZ) {
        score += 4;
        seenAxes.push(f.usage);
      }
    }
  }
  return score;
}

/**
 * 解析 HID Report Descriptor，返回最适合作为手柄输入的那个 report 的布局。
 */
export function parseHidReportDescriptor(desc: Uint8Array): HidReportLayout | null {
  const ctx: HidParseContext = new HidParseContext();
  let i: number = 0;

  while (i < desc.length) {
    const prefix: number = desc[i];
    if (prefix === 0) {
      i++;
      continue;
    }
    const sizeCode: number = prefix & 0x03;
    const size: number = sizeCode === 3 ? 4 : sizeCode;
    const type: number = (prefix >> 2) & 0x03;
    const tag: number = (prefix >> 4) & 0x0F;
    const dataOffset: number = i + 1;
    i = dataOffset + size;
    if (i > desc.length) break;

    // Long item (0xFE)：本解析器不需要，跳过其数据区
    if (prefix === 0xFE) {
      const longLen: number = itemValue(desc, dataOffset, 1);
      i = dataOffset + 3 + longLen;
      continue;
    }

    const value: number = itemValue(desc, dataOffset, size);

    if (type === TYPE_GLOBAL) {
      switch (tag) {
        case TAG_USAGE_PAGE:
          ctx.usagePage = value & 0xFFFF;
          break;
        case TAG_LOGICAL_MIN:
          ctx.logicalMin = signExtend(value, size * 8, -1);
          break;
        case TAG_LOGICAL_MAX:
          // Logical Maximum 按有符号解析；若最小值非负且高位被置，则按无符号看待
          ctx.logicalMax = ctx.logicalMin >= 0
            ? value
            : signExtend(value, size * 8, -1);
          break;
        case TAG_REPORT_SIZE:
          ctx.reportSize = value;
          break;
        case TAG_REPORT_ID:
          ctx.reportId = value;
          break;
        case TAG_REPORT_COUNT:
          ctx.reportCount = value;
          break;
        default:
          break; // Physical Min/Max、Unit、Push/Pop 对本用途无影响
      }
    } else if (type === TYPE_LOCAL) {
      switch (tag) {
        case TAG_USAGE:
          ctx.usages.push(value);
          break;
        case TAG_USAGE_MIN:
          ctx.usageMin = value;
          break;
        case TAG_USAGE_MAX:
          ctx.usageMax = value;
          break;
        default:
          break;
      }
    } else if (type === TYPE_MAIN) {
      if (tag === TAG_INPUT) {
        if (!ctx.reportAppUsage.has(ctx.reportId)) {
          ctx.reportAppUsage.set(ctx.reportId, (ctx.appUsagePage << 16) | ctx.appUsage);
        }
        const isConstant: boolean = (value & 0x01) !== 0;
        const isRelative: boolean = (value & 0x04) !== 0;
        const stored: number | undefined = ctx.inputBitPos.get(ctx.reportId);
        const base: number = stored !== undefined ? stored : 0;
        for (let k = 0; k < ctx.reportCount && k < 256; k++) {
          const u: HidUsage = ctx.resolveUsage(k);
          if (!isConstant) {
            ctx.fields.push({
              bitOffset: base + k * ctx.reportSize,
              bitSize: ctx.reportSize,
              usagePage: u.page,
              usage: u.id,
              logicalMin: ctx.logicalMin,
              logicalMax: ctx.logicalMax,
              isRelative: isRelative,
              reportId: ctx.reportId
            });
          }
        }
        ctx.inputBitPos.set(ctx.reportId, base + ctx.reportCount * ctx.reportSize);
        ctx.resetLocal();
      } else if (tag === TAG_OUTPUT || tag === TAG_FEATURE) {
        // 输出/特性报告与输入报告的位流互相独立，不影响输入字段偏移
        ctx.resetLocal();
      } else if (tag === TAG_COLLECTION) {
        if (ctx.collectionDepth === 0) {
          const app: HidUsage = ctx.resolveUsage(0);
          ctx.appUsagePage = app.page;
          ctx.appUsage = app.id;
        }
        ctx.collectionDepth++;
        ctx.resetLocal();
      } else if (tag === TAG_END_COLLECTION) {
        ctx.collectionDepth = Math.max(0, ctx.collectionDepth - 1);
        if (ctx.collectionDepth === 0) {
          ctx.appUsagePage = 0;
          ctx.appUsage = 0;
        }
        ctx.resetLocal();
      }
    }
  }

  if (ctx.fields.length === 0) {
    return null;
  }

  // 按 reportId 分组，挑出最像手柄输入的那个报告
  const ids: number[] = [];
  for (const f of ctx.fields) {
    if (ids.indexOf(f.reportId) < 0) {
      ids.push(f.reportId);
    }
  }
  let bestId: number = ids[0];
  let bestScore: number = -1;
  for (const id of ids) {
    const group: HidField[] = ctx.fields.filter((f: HidField): boolean => f.reportId === id);
    const score: number = gamepadScore(group);
    if (score > bestScore) {
      bestScore = score;
      bestId = id;
    }
  }
  if (bestScore <= 0) {
    return null;
  }

  const appRaw: number | undefined = ctx.reportAppUsage.get(bestId);
  const appPage: number = appRaw !== undefined ? (appRaw >>> 16) & 0xFFFF : 0;
  const appId: number = appRaw !== undefined ? appRaw & 0xFFFF : 0;
  // 明确属于其它设备类型的 Application Collection 一律排除：
  // Digitizer(0x0D/触摸板)、Consumer(0x0C)、Keyboard(1/6)、Mouse(1/2)、厂商自定义(>=0xFF00)
  const nonGamepad: boolean =
    appPage === 0x0D || appPage === 0x0C ||
    (appPage === PAGE_GENERIC_DESKTOP && (appId === 0x02 || appId === 0x06)) ||
    appPage >= 0xFF00;
  if (nonGamepad) {
    return null;
  }

  const chosen: HidField[] = ctx.fields
    .filter((f: HidField): boolean => f.reportId === bestId && !f.isRelative)
    .sort((a: HidField, b: HidField): number => a.bitOffset - b.bitOffset);

  let reportBits: number = 0;
  for (const f of chosen) {
    reportBits = Math.max(reportBits, f.bitOffset + f.bitSize);
  }

  const parts: string[] = [];
  for (const f of chosen) {
    parts.push(`p${f.usagePage.toString(16)}/u${f.usage.toString(16)}@${f.bitOffset}:${f.bitSize}`);
  }
  const description: string = `app=${appPage.toString(16)}/${appId.toString(16)} report=0x${bestId.toString(16)} bits=${reportBits} fields=[${parts.join(' ')}]`;

  return {
    fields: chosen,
    reportId: bestId,
    reportBits: reportBits,
    description: description,
    appUsagePage: appPage,
    appUsage: appId,
    buttonMap: null
  };
}

/** USB 配置描述符里一个接口的关键信息 */
export interface HidInterfaceInfo {
  interfaceId: number;
  clazz: number;
  subClass: number;
  protocol: number;
  /** HID 描述符 (0x21) 声明的报告描述符长度；报告内容需另行读取 */
  reportDescriptorLength: number;
  inputEndpoint: number;
  inputMaxPacketSize: number;
  outputEndpoint: number;
}

/**
 * 解析 getRawDescriptor() 返回的配置描述符链，列出每个接口的类/子类/协议、
 * HID 报告描述符长度以及中断端点。
 *
 * 重要：USB 配置描述符里出现的 0x22 只是 HID 描述符中的「报告描述符类型」，
 * 紧随其后的两个字节是长度，并不包含报告描述符本身。真正的报告描述符必须
 * 通过 GET_DESCRIPTOR 控制传输（wValue=0x2200, wIndex=接口号）单独读取。
 */
export function parseUsbConfiguration(raw: Uint8Array): HidInterfaceInfo[] {
  const list: HidInterfaceInfo[] = [];
  let currentIndex: number = -1;
  let i: number = 0;

  while (i + 2 <= raw.length) {
    const len: number = raw[i];
    const type: number = raw[i + 1];
    if (len < 2 || i + len > raw.length) {
      i++;
      continue;
    }

    if (type === 0x04 && len >= 9) {
      // Interface Descriptor: [len][0x04][bInterfaceNumber][bAlternateSetting]
      //                       [bNumEndpoints][bInterfaceClass][SubClass][Protocol][iInterface]
      list.push({
        interfaceId: raw[i + 2],
        clazz: raw[i + 5],
        subClass: raw[i + 6],
        protocol: raw[i + 7],
        reportDescriptorLength: 0,
        inputEndpoint: 0,
        inputMaxPacketSize: 0,
        outputEndpoint: 0
      });
      currentIndex = list.length - 1;
    } else if (currentIndex >= 0) {
      const info: HidInterfaceInfo = list[currentIndex];
      if (type === 0x21 && len >= 9) {
        // HID Descriptor: [len][0x21][bcdHID lo][hi][country][numDesc]
        //                 [descType=0x22][wDescriptorLength lo][hi] ...
        for (let k = 6; k + 2 < len; k += 3) {
          if (raw[i + k] === 0x22) {
            info.reportDescriptorLength = raw[i + k + 1] + raw[i + k + 2] * 256;
            break;
          }
        }
      } else if (type === 0x05 && len >= 7) {
        // Endpoint Descriptor: [len][0x05][bEndpointAddress][bmAttributes][wMaxPacketSize][bInterval]
        const addr: number = raw[i + 2];
        const maxPkt: number = raw[i + 4] + raw[i + 5] * 256;
        if ((addr & 0x80) !== 0) {
          if (info.inputEndpoint === 0) {
            info.inputEndpoint = addr;
            info.inputMaxPacketSize = maxPkt;
          }
        } else if (info.outputEndpoint === 0) {
          info.outputEndpoint = addr;
        }
      }
    }
    i += len;
  }
  return list;
}

/** 给布局打「像不像游戏手柄」的分，供调用方在多个接口之间做选择 */
export function scoreHidLayout(layout: HidReportLayout): number {
  return gamepadScore(layout.fields);
}

/**
 * 兼容少数把报告描述符内容直接内嵌在 raw 描述符链里的实现。
 * 标准 USB 配置描述符不会这样，因此这里仅作为兜底。
 */
export function parseRawUsbDescriptors(raw: Uint8Array): HidReportLayout | null {
  let i: number = 0;
  let best: HidReportLayout | null = null;
  let bestScore: number = 0;

  while (i + 2 <= raw.length) {
    const len: number = raw[i];
    const type: number = raw[i + 1];
    if (len < 2) {
      i++;
      continue;
    }
    if (type === 0x22 && i + len <= raw.length && len > 2) {
      const desc: Uint8Array = raw.subarray(i + 2, i + len);
      const layout: HidReportLayout | null = parseHidReportDescriptor(desc);
      if (layout !== null) {
        const score: number = gamepadScore(layout.fields);
        if (score > bestScore) {
          bestScore = score;
          best = layout;
        }
      }
    }
    i += len;
  }
  return best;
}

/**
 * 按布局解码一帧输入报告，并映射到 Moonlight 手柄状态。
 * 返回 null 表示这帧不是目标 report（调用方应走原有解析路径）。
 */
export function decodeHidReport(data: Uint8Array, layout: HidReportLayout): HidGamepadState | null {
  let startByte: number = 0;
  if (layout.reportId !== 0) {
    if (data.length < 2 || data[0] !== layout.reportId) {
      return null;
    }
    startByte = 1;
  }
  if (data.length - startByte < Math.ceil(layout.reportBits / 8)) {
    return null;
  }

  const state: HidGamepadState = {
    buttonFlags: 0,
    leftStickX: 0,
    leftStickY: 0,
    rightStickX: 0,
    rightStickY: 0,
    leftTrigger: 0,
    rightTrigger: 0
  };

  // 先记录各轴向字段，便于区分「Z/Rz 是右摇杆还是扳机」
  let hasRx: boolean = false;
  let hasRy: boolean = false;
  for (const f of layout.fields) {
    if (f.usagePage === PAGE_GENERIC_DESKTOP) {
      if (f.usage === USAGE_RX) hasRx = true;
      if (f.usage === USAGE_RY) hasRy = true;
    }
  }
  const zIsStick: boolean = !hasRx && !hasRy;

  for (const f of layout.fields) {
    if (f.bitSize <= 0 || f.bitSize > 31) continue;
    const raw: number = readBits(data, startByte, f.bitOffset, f.bitSize);
    const value: number = signExtend(raw, f.bitSize, f.logicalMin);

    if (f.usagePage === PAGE_BUTTON) {
      if (value !== 0 && f.bitSize === 1) {
        const override: number[] | null = layout.buttonMap;
        if (override !== null && f.usage >= 0 && f.usage < override.length) {
          state.buttonFlags |= override[f.usage];
        } else {
          state.buttonFlags |= buttonFlagForUsage(f.usage);
        }
      }
    } else if (f.usagePage === PAGE_GENERIC_DESKTOP) {
      switch (f.usage) {
        case USAGE_X:
          state.leftStickX = normalizeAxis(value, f.logicalMin, f.logicalMax);
          break;
        case USAGE_Y:
          state.leftStickY = normalizeAxis(value, f.logicalMin, f.logicalMax);
          break;
        case USAGE_RX:
          state.rightStickX = normalizeAxis(value, f.logicalMin, f.logicalMax);
          break;
        case USAGE_RY:
          state.rightStickY = normalizeAxis(value, f.logicalMin, f.logicalMax);
          break;
        case USAGE_Z:
          if (zIsStick) {
            state.rightStickX = normalizeAxis(value, f.logicalMin, f.logicalMax);
          } else {
            state.leftTrigger = normalizeTrigger(value, f.logicalMin, f.logicalMax);
          }
          break;
        case USAGE_RZ:
          if (zIsStick) {
            state.rightStickY = normalizeAxis(value, f.logicalMin, f.logicalMax);
          } else {
            state.rightTrigger = normalizeTrigger(value, f.logicalMin, f.logicalMax);
          }
          break;
        case USAGE_HAT: {
          const direction: number = Math.round(value - f.logicalMin);
          state.buttonFlags |= hatFlags(direction);
          break;
        }
        default:
          break;
      }
    } else if (f.usagePage === PAGE_SIMULATION) {
      // Kishi Ultra 等手柄把 LT/RT 声明为 Simulation Controls 页的 8bit 模拟量，
      // 而非 Z/Rz 轴；忽略这页会让扳机永远为 0。
      const trigger: number = normalizeTrigger(value, f.logicalMin, f.logicalMax);
      if (f.usage === USAGE_BRAKE) {
        state.leftTrigger = trigger;
      } else if (f.usage === USAGE_ACCELERATOR) {
        state.rightTrigger = trigger;
      }
    }
  }
  return state;
}

/**
 * 生成一帧输入的诊断描述：按下的 HID Button Usage 编号、Hat 方向、扳机值。
 * 用于确定「某个 Usage 实际对应手柄上的哪个物理键」，不参与输入上报。
 */
export function describeHidReport(data: Uint8Array, layout: HidReportLayout): string {
  let startByte: number = 0;
  if (layout.reportId !== 0) {
    if (data.length < 2 || data[0] !== layout.reportId) {
      return `otherReport=0x${data[0].toString(16)}`;
    }
    startByte = 1;
  }
  const pressed: string[] = [];
  let hat: number = -1;
  let lt: number = 0;
  let rt: number = 0;
  for (const f of layout.fields) {
    if (f.bitSize <= 0 || f.bitSize > 31) continue;
    const raw: number = readBits(data, startByte, f.bitOffset, f.bitSize);
    const value: number = signExtend(raw, f.bitSize, f.logicalMin);
    if (f.usagePage === PAGE_BUTTON && f.bitSize === 1 && value !== 0) {
      pressed.push(`u${f.usage}`);
    } else if (f.usagePage === PAGE_GENERIC_DESKTOP && f.usage === USAGE_HAT) {
      hat = value;
    } else if (f.usagePage === PAGE_SIMULATION) {
      const trigger: number = normalizeTrigger(value, f.logicalMin, f.logicalMax);
      if (f.usage === USAGE_BRAKE) lt = trigger;
      else if (f.usage === USAGE_ACCELERATOR) rt = trigger;
    }
  }
  // 扳机量化到 0.25 档：既能记录按下/松开，又不会因模拟量抖动刷屏
  const ltStep: number = Math.round(lt * 4) / 4;
  const rtStep: number = Math.round(rt * 4) / 4;
  return `hat=${hat} btn=[${pressed.join(',')}] lt=${ltStep.toFixed(2)} rt=${rtStep.toFixed(2)}`;
}

/** 把字节数组转成 hex 字符串，用于诊断日志 */
export function hexDump(data: Uint8Array, maxBytes: number): string {
  const parts: string[] = [];
  const n: number = Math.min(data.length, maxBytes);
  for (let i = 0; i < n; i++) {
    parts.push(data[i].toString(16).padStart(2, '0'));
  }
  return parts.join(' ');
}
