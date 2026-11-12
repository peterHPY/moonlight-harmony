# 雷蛇骑仕 Sensa HD 触感驱动集成说明

## 概述

本集成为 Razer Kishi Ultra / Kishi V3 Pro（及后续采用相同触感接口的骑仕型号）
提供真实的游戏 rumble 触感输出，绕过雷蛇官方 Nexus/Synapse 的"音频转触感"
方案，把 Moonlight 串流会话中主机发来的双马达震动直接渲染到骑仕的
Sensa HD 宽频触感线圈上。

协议来源：[Mokomis/rumblebridge](https://github.com/Mokomis/rumblebridge)
（MIT License）对 Kishi 固件 2.0.0.0 "Razer HD Haptics Specifications"
接口的逆向成果，已移植为 ArkTS（`usbManager` 主机 API）。

## 文件

| 文件 | 职责 |
|---|---|
| `entry/src/main/ets/service/usbdriver/RazerKishiHaptics.ets` | 触感驱动：设备识别、接口占用、帧编码、30ms 帧流 |
| `entry/src/main/ets/service/usbdriver/UsbDriverService.ets` | 挂载点：控制器启动后 `tryAttach()`，拔出/停止时 `detach()` |
| `entry/src/main/ets/service/input/GamepadVibrationService.ets` | 输出点：`renderMixedUsb()` 混合结果并行渲染到触感 |

## 数据流

```
主机 rumble 事件 (0-65535 双马达)
  → GamepadVibrationService.handleRumble()
  → renderMixedUsb() 混合游戏/IR/音频三来源
  → controller.rumble(finalLow, finalHigh)      // 标准 rumble 通道（如 XInput）
  → RazerKishiHaptics.render(finalLow, finalHigh)  // 雷蛇触感通道（并行）
      → level(): 0-65535 → 0-63 触感幅度（强度/曲线/马达缩放）
      → 30ms 帧流: output report 2 [2][len][0][1][0x0E][位流]
      → bulk OUT 0x04 + 读取 IN 0x84 应答（链路健康信号）
```

## 协议要点（实测约束，勿破坏）

1. **只发流帧（command 0x0E）**：在该接口上发送任何查询/设置命令
   （mode 0x87、gain 0x88/0x08）会让设备停止播放帧，直到重新插拔。
2. **空闲时完全不发送**：骑仕 15 分钟无输入自动休眠掉线。持续发送保活帧
   会导致设备"应答但不震动"，只能重新插拔恢复。停止时只发一帧静音帧收尾，
   之后彻底静默，设备休眠后由 `streamTick` 的失败计数自动 `detach()`，
   插回后 `UsbDriverService` 重新枚举并 attach。
3. **每帧读取 IN 应答**：设备对每帧都有回答；连续 10 帧无应答视为
   设备已休眠/拔出。
4. **单马达镜像**：游戏只驱动重马达（如部分游戏只用 low band）时，
   触感帧会把它镜像到两侧握柄，避免只有单侧有触感。

## 设备识别

不硬编码 PID（Kishi V3 Pro HID 模式为 `1532:0724`，XInput 模式无触感接口；
Kishi Ultra 的 PID 因固件而异）。识别规则：

- VID = `0x1532`（Razer）
- 存在厂商自定义类接口（`clazz = 0xFF`），且带 bulk IN + bulk OUT 端点

满足即视为触感接口，在控制器 pipe 上额外 `claimInterface(force=true)`。
触感接口与手柄输入接口相互独立，互不影响。

## 已知限制

- 需要 Kishi 处于触感可用的固件/模式（实测基准：Kishi V3 Pro 固件 2.0.0.0）；
  Kishi Ultra 的协议帧格式按 rumblebridge 逆向成果假定一致，首次在真机上
  验证时建议先用较低强度（`applySettings(30, 75, 10, 35, true)`）确认。
- 依赖 `usbManager` 权限流程，与现有控制器权限一致，无新增权限。
- iOS/欧版固件差异、未来雷蛇固件变更可能导致协议失配。

## 上游致谢

- [Mokomis/rumblebridge](https://github.com/Mokomis/rumblebridge) —
  触感协议逆向与参考实现（MIT），版权声明见 `OPEN_SOURCE_LICENSES.md`。
