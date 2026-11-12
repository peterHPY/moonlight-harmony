/*
 * Moonlight for HarmonyOS
 * Copyright (C) 2024-2025 Moonlight/AlkaidLab
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

import { vibrator } from '@kit.SensorServiceKit';
import { deviceInfo } from '@kit.BasicServicesKit';
import { Logger } from '../../common/Logger';

/**
 * 外接手柄震动（动态马达 / Dynamic Vibrator）后端。
 *
 * 鸿蒙 SensorServiceKit 把「设备外置、支持独立控制的多个马达」称为动态马达，
 * 明确用于手柄、遥控器、外接振动器等外接设备。系统会把这些外接马达登记进
 * vibrator 子系统，应用可用 getVibratorInfoSync() 枚举（isLocalVibrator===false），
 * 再用 startVibration(effect, { deviceId, id }) 按 deviceId / vibratorId 定向驱动。
 *
 * 这是相比 USB 直连 / BLE GATT 更干净的官方路径：不需要自己接管 HID 通道。
 * 若系统未把蓝牙手柄登记为动态马达（标准手柄可能不登记），本模块枚举结果为空，
 * 所有方法为空操作，不影响现有 USB / 机身马达行为。
 */

interface ExternalVibrator {
  deviceId: number;
  deviceName: string;
  vibratorIds: number[];
}

export class ExternalGamepadVibrator {
  private static readonly LEASE_MS = 200;
  private static readonly RESCAN_MS = 2000;
  private static _inst: ExternalGamepadVibrator | null = null;

  static get instance(): ExternalGamepadVibrator {
    return this._inst ?? (this._inst = new ExternalGamepadVibrator());
  }

  private vibrators: ExternalVibrator[] = [];
  private initialized = false;
  private lastRefreshMs: number = 0;

  /**
   * 重新枚举外接（非本地）振动器（手柄等）。仅在设备 API 19+ 上调用，
   * 返回的非本地振动器即为可定向驱动的外接手柄马达。
   */
  refresh(): void {
    this.lastRefreshMs = Date.now();
    if (deviceInfo.sdkApiVersion < 19) {
      this.vibrators = [];
      this.initialized = true;
      return;
    }
    try {
      const list: vibrator.VibratorInfo[] = vibrator.getVibratorInfoSync();
      this.vibrators = list
        .filter((info: vibrator.VibratorInfo): boolean => !info.isLocalVibrator)
        .reduce((acc: ExternalVibrator[], info: vibrator.VibratorInfo): ExternalVibrator[] => {
          let entry = acc.find((v): boolean => v.deviceId === info.deviceId);
          if (!entry) {
            entry = { deviceId: info.deviceId, deviceName: info.deviceName ?? '', vibratorIds: [] };
            acc.push(entry);
          }
          if (!entry.vibratorIds.includes(info.vibratorId)) {
            entry.vibratorIds.push(info.vibratorId);
          }
          return acc;
        }, []);
      this.initialized = true;
      Logger.info('ExternalGamepadVibrator',
        `[VIBRATION] 发现外接振动器 ${this.vibrators.length} 个: ` +
        this.vibrators.map((v): string => `${v.deviceName}(dev=${v.deviceId},ids=${v.vibratorIds.join(',')})`).join(' | '));
    } catch (err) {
      Logger.warn('ExternalGamepadVibrator', `[VIBRATION] 枚举外接振动器失败: ${err}`);
      this.vibrators = [];
      this.initialized = true;
    }
  }

  /** 是否检测到可驱动的外接手柄振动器 */
  hasExternalGamepads(): boolean {
    if (!this.initialized ||
        (this.vibrators.length === 0 && Date.now() - this.lastRefreshMs > ExternalGamepadVibrator.RESCAN_MS)) {
      this.refresh();
    }
    return this.vibrators.length > 0;
  }

  /**
   * 向所有外接手柄振动器发送双马达震动；low/high: 0..255。
   * 双马达手柄暴露 2 个 vibratorId 时左右独立；单马达时取较强者。
   */
  rumbleAll(low: number, high: number): void {
    if (!this.initialized ||
        (this.vibrators.length === 0 && Date.now() - this.lastRefreshMs > ExternalGamepadVibrator.RESCAN_MS)) {
      this.refresh();
    }
    if (this.vibrators.length === 0) return;
    const l = Math.max(0, Math.min(255, Math.floor(low)));
    const h = Math.max(0, Math.min(255, Math.floor(high)));
    for (const v of this.vibrators) {
      try {
        if (v.vibratorIds.length >= 2) {
          this.startMotor(v.deviceId, v.vibratorIds[0], l);
          this.startMotor(v.deviceId, v.vibratorIds[1], h);
        } else if (v.vibratorIds.length === 1) {
          this.startMotor(v.deviceId, v.vibratorIds[0], Math.max(l, h));
        }
      } catch (err) {
        Logger.warn('ExternalGamepadVibrator', `[VIBRATION] 外接震动失败 device=${v.deviceId}: ${err}`);
      }
    }
  }

  /** 停止所有外接手柄振动器（仅针对各自 deviceId，不影响本机马达）。 */
  stopAll(): void {
    if (this.vibrators.length === 0) return;
    for (const v of this.vibrators) {
      try {
        vibrator.stopVibration({ deviceId: v.deviceId });
      } catch (err) {
        Logger.warn('ExternalGamepadVibrator', `[VIBRATION] 停止外接震动失败 device=${v.deviceId}: ${err}`);
      }
    }
  }

  private startMotor(deviceId: number, vibratorId: number, intensity0255: number): void {
    const intensity = Math.round(intensity0255 / 255 * 100);
    if (intensity <= 0) return;
    // VibratorPatternBuilder / addContinuousEvent / build 均为 SDK 18+ API，
    // 且外接动态马达本身只在 API 19+ 才会出现；此处用 SDK 版本做静态可达性保护，
    // 既避免低版本设备走到不存在的 API，也规避兼容 SDK(12) 的告警。
    if (deviceInfo.sdkApiVersion < 18) return;
    let pattern: vibrator.VibratorPattern;
    try {
      const builder = new vibrator.VibratorPatternBuilder();
      builder.addContinuousEvent(0, ExternalGamepadVibrator.LEASE_MS, {
        intensity: intensity,
        frequency: 0,
        index: 0
      });
      pattern = builder.build();
    } catch (err) {
      Logger.warn('ExternalGamepadVibrator', `[VIBRATION] 构造震动波形失败: ${err}`);
      return;
    }
    try {
      vibrator.startVibration(
        { type: 'pattern', pattern: pattern },
        { deviceId: deviceId, id: vibratorId, usage: 'media' }
      );
    } catch (err) {
      Logger.warn('ExternalGamepadVibrator',
        `[VIBRATION] 触发外接震动失败 device=${deviceId},id=${vibratorId}: ${err}`);
    }
  }
}
