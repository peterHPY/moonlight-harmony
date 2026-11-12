/*
 * Moonlight for HarmonyOS
 * Copyright (C) 2024-2026 Moonlight/AlkaidLab
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

import { GamepadManager } from './input/GamepadManager';
import { RUMBLE_MAX } from './input/GamepadTypes';
import { AudioHapticFrame, AudioHapticFrameFlag } from './AudioHapticFrame';
import { RazerKishiHaptics } from './usbdriver/RazerKishiHaptics';

interface UsbMotorPair {
  low: number;
  high: number;
}

/**
 * Platform renderer for the portable audio-haptics SDK IR.
 *
 * DSP and scene authoring live in the SDK. This class owns only presentation,
 * device capability fallback, user strength and device/USB routing.
 */
export class AudioVibrationService {
  // 参考 PeaSyo 连续转发音频触觉流的模式：原先 100ms 节流 + 幅度迟滞 8
  // 会把雨声、脚步这类细密连续变化全部抹平（最多只有 10Hz 更新）。
  private static readonly MINIMUM_INTENSITY: number = 2;
  private static readonly TRANSIENT_MINIMUM_INTERVAL_MS: number = 12;
  private static readonly CONTINUOUS_MINIMUM_INTERVAL_MS: number = 25;
  private static readonly CONTINUOUS_AMPLITUDE_HYSTERESIS: number = 2;
  private static readonly MAXIMUM_PRESENTATION_DELAY_MS: number = 300;
  // 整体输出增益：音频触觉默认映射偏保守（高频连续分量仅 0.25x），
  // 拉到 1.7 让同样的 AhHapticFrame 产生更接近 DS5 的满幅马达输出。
  // toRumbleMotor / 混音 headroom 会限幅到 RUMBLE_MAX，不会溢出。
  private static readonly OUTPUT_GAIN: number = 1.7;
  // Safety bound for platforms or controllers that keep the last vibration until explicitly cleared.
  private static readonly CONTINUOUS_OUTPUT_WATCHDOG_MS: number = 1200;
  private static readonly DEFAULT_ONSET_SENSITIVITY: number = 1.6;
  private static readonly MUSIC_ONSET_SENSITIVITY: number = 3.0;

  private enabled: boolean = false;
  private strength: number = 100;
  private vibrationMode: string = '自动';
  private sceneMode: string = '游戏/电影';

  private isDeviceSourceActive: boolean = false;
  private isUsbSourceActive: boolean = false;

  private currentContinuousUsbLow: number = 0;
  private currentContinuousUsbHigh: number = 0;
  private lastRenderedContinuousIntensity: number = 0;
  private lastRenderedTimestampUs: number = -1;
  private lastTransientSubmitTimeMs: number = 0;
  private lastContinuousSubmitTimeMs: number = 0;
  private renderEpoch: number = 0;
  private usbEffectEpoch: number = 0;
  private continuousOutputWatchdogTimer: number = -1;
  private continuousOutputWatchdogEpoch: number = 0;

  setSettings(enabled: boolean, strength: number, vibrationMode: string,
    sceneMode: string = '游戏/电影'): void {
    this.renderEpoch++;
    this.lastRenderedTimestampUs = -1;
    this.enabled = enabled;
    this.strength = Math.max(0, Math.min(100, strength));
    this.vibrationMode = vibrationMode;
    this.sceneMode = sceneMode;
    if (!enabled) {
      this.stopAll();
    }
  }

  /**
   * SDK ABI scene: GAME=0, MUSIC=1, AUTO=2.
   * ABI v1 currently resolves AUTO to the production GAME profile.
   */
  getSceneModeInt(): number {
    switch (this.sceneMode) {
      case '音乐/节奏': return 1;
      case '自动': return 2;
      case '游戏/电影':
      default: return 0;
    }
  }

  getOnsetSensitivity(): number {
    return this.sceneMode === '音乐/节奏'
      ? AudioVibrationService.MUSIC_ONSET_SENSITIVITY
      : AudioVibrationService.DEFAULT_ONSET_SENSITIVITY;
  }

  handleHapticFrame(frame: AudioHapticFrame): void {
    if (!this.enabled) return;

    const hasStop = (frame.flags & AudioHapticFrameFlag.STOP) !== 0;
    const hasTransient =
      (frame.flags & AudioHapticFrameFlag.TRANSIENT) !== 0;
    if (hasStop) {
      this.renderEpoch++;
      this.lastRenderedTimestampUs = -1;
      this.stopAll();
      if (!hasTransient) return;
    }

    const epoch = this.renderEpoch;
    const delayMs = Math.max(0, Math.min(
      AudioVibrationService.MAXIMUM_PRESENTATION_DELAY_MS,
      Math.round(frame.presentationDelayMs)));
    if (delayMs > 1) {
      setTimeout((): void => {
        if (this.enabled && epoch === this.renderEpoch) {
          this.renderFrame(frame);
        }
      }, delayMs);
    } else {
      this.renderFrame(frame);
    }
  }

  stop(): void {
    this.renderEpoch++;
    this.lastRenderedTimestampUs = -1;
    this.stopAll();
  }

  private renderFrame(frame: AudioHapticFrame): void {
    if (!this.enabled) return;
    if (frame.timestampUs < this.lastRenderedTimestampUs) return;

    const hasTransient =
      (frame.flags & AudioHapticFrameFlag.TRANSIENT) !== 0;
    const continuousChanged =
      (frame.flags & AudioHapticFrameFlag.CONTINUOUS_CHANGED) !== 0;
    const hasStop = (frame.flags & AudioHapticFrameFlag.STOP) !== 0;

    if (hasStop) {
      this.stopAll();
      if (!hasTransient) return;
    }

    const strengthScale = this.strength / 100;
    const continuous = this.toIntensity(
      frame.continuousAmplitude * strengthScale);
    const transient = this.toIntensity(
      frame.transientAmplitude * strengthScale);
    const lowBandRatio = Math.max(0, Math.min(100,
      Math.round(frame.lowBandRatio * 100)));
    const stereoBalance = Math.max(0, Math.min(100,
      Math.round((frame.stereoPan + 1) * 50)));

    const selected = hasTransient
      ? Math.max(transient, continuous)
      : continuous;
    // Valid transient intent must reach the actuator policy. Some motors need
    // a platform-specific amplitude floor to make weak musical beats tangible.
    if (!hasTransient &&
        selected < AudioVibrationService.MINIMUM_INTENSITY) {
      if (continuousChanged || hasStop) this.stopAll();
      return;
    }

    if (continuous >= AudioVibrationService.MINIMUM_INTENSITY &&
        (this.isDeviceSourceActive || this.isUsbSourceActive)) {
      this.armContinuousOutputWatchdog();
    }

    const now = Date.now();
    if (!hasTransient) {
      // Do not let a continuous refresh replace a beat that is still being
      // presented, and ignore small envelope jitter while output is active.
      if (now - this.lastTransientSubmitTimeMs <
          AudioVibrationService.CONTINUOUS_MINIMUM_INTERVAL_MS) {
        return;
      }
      if (!hasStop &&
          (this.isDeviceSourceActive || this.isUsbSourceActive) &&
          Math.abs(continuous - this.lastRenderedContinuousIntensity) <
            AudioVibrationService.CONTINUOUS_AMPLITUDE_HYSTERESIS) {
        return;
      }
    }
    const minimumInterval = hasTransient
      ? AudioVibrationService.TRANSIENT_MINIMUM_INTERVAL_MS
      : AudioVibrationService.CONTINUOUS_MINIMUM_INTERVAL_MS;
    const lastSubmitTimeMs = hasTransient
      ? this.lastTransientSubmitTimeMs
      : this.lastContinuousSubmitTimeMs;
    if (now - lastSubmitTimeMs < minimumInterval) return;
    if (hasTransient) {
      this.lastTransientSubmitTimeMs = now;
    } else {
      this.lastContinuousSubmitTimeMs = now;
      this.lastRenderedContinuousIntensity = continuous;
    }
    this.lastRenderedTimestampUs = frame.timestampUs;

    const shouldDevice = this.shouldVibrateDevice();
    const shouldUsb = this.shouldVibrateGamepad();
    if (shouldDevice) {
      this.renderDevice(
        continuous,
        transient,
        frame.transientDurationMs,
        frame.sharpness,
        hasTransient,
        frame.activeScene,
        frame.flags
      );
    } else if (this.isDeviceSourceActive) {
      this.stopDeviceVibration();
    }

    if (shouldUsb) {
      this.renderUsb(
        continuous,
        transient,
        frame.transientDurationMs,
        hasTransient,
        lowBandRatio,
        stereoBalance,
        frame.sharpness,
        continuousChanged
      );
    } else if (this.isUsbSourceActive) {
      this.stopUsbRumble();
    }

    if (continuous >= AudioVibrationService.MINIMUM_INTENSITY &&
        (this.isDeviceSourceActive || this.isUsbSourceActive)) {
      this.armContinuousOutputWatchdog();
    }
  }

  private toIntensity(amplitude: number): number {
    return Math.max(0, Math.min(100, Math.round(amplitude * 100)));
  }

  private shouldVibrateDevice(): boolean {
    switch (this.vibrationMode) {
      case '仅手柄': return false;
      case '仅设备':
      case '协同': return true;
      case '自动':
      default:
        // Automatic audio haptics may use the gamepad path only when this
        // service can actually drive a USB rumble endpoint. Bluetooth/GCK
        // input devices do not currently expose an audio haptics backend, so
        // keep the body fallback enabled for those devices.
        return !GamepadManager.getInstance().hasUsbControllers();
    }
  }

  private shouldVibrateGamepad(): boolean {
    switch (this.vibrationMode) {
      case '仅手柄':
      case '协同': return true;
      case '仅设备': return false;
      case '自动':
      default:
        return GamepadManager.getInstance().hasUsbControllers();
    }
  }

  private renderDevice(continuous: number, transient: number,
    transientDurationMs: number, sharpness: number, hasTransient: boolean,
    activeScene: number, frameFlags: AudioHapticFrameFlag): void {
    GamepadManager.getInstance().submitAudioDeviceHaptics(
      continuous,
      transient,
      transientDurationMs,
      sharpness,
      hasTransient,
      activeScene,
      frameFlags
    );
    this.isDeviceSourceActive = true;
  }

  private renderUsb(continuous: number, transient: number,
    transientDurationMs: number, hasTransient: boolean, lowBandRatio: number,
    stereoBalance: number, sharpness: number,
    continuousChanged: boolean): void {
    const lowRatio = Math.max(0, Math.min(1, lowBandRatio / 100));
    const normalizedSharpness = Math.max(0, Math.min(1, sharpness));
    // 把本地音频分析出的锐度/瞬态/颗粒度交给雷蛇波形合成器。
    // 两个马达标量只能表达「有多强」，这四个特征才能表达「是什么声音」。
    const transientUnit = hasTransient
      ? Math.max(0, Math.min(1, transient / 100)) : 0;
    const texture = Math.min(1, normalizedSharpness * 0.7 + transientUnit * 0.3);
    RazerKishiHaptics.getInstance().setTimbre(
      normalizedSharpness, transientUnit, texture, !hasTransient
    );
    // Standard rumble exposes frequency motors, not independent left/right
    // haptic channels. Keep stereoPan as a bounded spatial hint only.
    const spatialFactor = ((stereoBalance - 50) / 50) * 0.3;
    let continuousMotors: UsbMotorPair = { low: 0, high: 0 };
    if (continuous >= AudioVibrationService.MINIMUM_INTENSITY) {
      continuousMotors = this.mapContinuousUsbMotors(
        continuous, lowRatio, spatialFactor);
    }
    if (continuousChanged || !this.isUsbSourceActive) {
      this.currentContinuousUsbLow = continuousMotors.low;
      this.currentContinuousUsbHigh = continuousMotors.high;
    }

    if (!hasTransient) {
      this.renderCurrentUsbContinuous();
      return;
    }
    if (transient < AudioVibrationService.MINIMUM_INTENSITY) {
      if (continuousChanged) this.renderCurrentUsbContinuous();
      return;
    }

    const transientMotors = this.mapTransientUsbMotors(
      transient, lowRatio, normalizedSharpness, spatialFactor);
    this.submitUsbRumble(
      Math.max(this.currentContinuousUsbLow, transientMotors.low),
      Math.max(this.currentContinuousUsbHigh, transientMotors.high)
    );
    const effectEpoch = ++this.usbEffectEpoch;
    const restoreDelayMs = Math.max(18, Math.min(100,
      Math.round(transientDurationMs)));
    setTimeout((): void => {
      if (!this.enabled || effectEpoch !== this.usbEffectEpoch) return;
      this.renderCurrentUsbContinuous();
    }, restoreDelayMs);
  }

  private mapContinuousUsbMotors(intensity: number, lowRatio: number,
    spatialFactor: number): UsbMotorPair {
    const amplitude = (intensity / 100) * AudioVibrationService.OUTPUT_GAIN;
    return {
      low: this.toRumbleMotor(
        amplitude * (0.55 + 0.45 * lowRatio) * (1 - spatialFactor)),
      high: this.toRumbleMotor(
        amplitude * (1 - lowRatio) * 0.55 * (1 + spatialFactor))
    };
  }

  private mapTransientUsbMotors(intensity: number, lowRatio: number,
    sharpness: number, spatialFactor: number): UsbMotorPair {
    const amplitude = (intensity / 100) * AudioVibrationService.OUTPUT_GAIN;
    return {
      low: this.toRumbleMotor(
        amplitude * (0.25 + 0.75 * lowRatio) *
        (1 - 0.35 * sharpness) * (1 - spatialFactor)),
      high: this.toRumbleMotor(
        amplitude * (0.35 + 0.65 * sharpness) * (1 + spatialFactor))
    };
  }

  private renderCurrentUsbContinuous(): void {
    if (this.currentContinuousUsbLow > 0 ||
        this.currentContinuousUsbHigh > 0) {
      this.submitUsbRumble(
        this.currentContinuousUsbLow,
        this.currentContinuousUsbHigh
      );
    } else if (this.isUsbSourceActive) {
      this.stopUsbRumble();
    }
  }

  private toRumbleMotor(amplitude: number): number {
    return Math.max(0, Math.min(RUMBLE_MAX,
      Math.floor(amplitude * RUMBLE_MAX)));
  }

  private submitUsbRumble(low: number, high: number): void {
    GamepadManager.getInstance().submitAudioUsbRumble(0, low, high);
    this.isUsbSourceActive = true;
  }

  private armContinuousOutputWatchdog(): void {
    const watchdogEpoch = ++this.continuousOutputWatchdogEpoch;
    if (this.continuousOutputWatchdogTimer !== -1) {
      clearTimeout(this.continuousOutputWatchdogTimer);
    }
    this.continuousOutputWatchdogTimer = setTimeout((): void => {
      if (!this.enabled ||
          watchdogEpoch !== this.continuousOutputWatchdogEpoch) {
        return;
      }
      this.continuousOutputWatchdogTimer = -1;
      this.stopAll();
    }, AudioVibrationService.CONTINUOUS_OUTPUT_WATCHDOG_MS);
  }

  private cancelContinuousOutputWatchdog(): void {
    this.continuousOutputWatchdogEpoch++;
    if (this.continuousOutputWatchdogTimer !== -1) {
      clearTimeout(this.continuousOutputWatchdogTimer);
      this.continuousOutputWatchdogTimer = -1;
    }
  }

  private stopAll(): void {
    this.cancelContinuousOutputWatchdog();
    this.usbEffectEpoch++;
    if (this.isDeviceSourceActive) this.stopDeviceVibration();
    if (this.isUsbSourceActive) this.stopUsbRumble();
    this.currentContinuousUsbLow = 0;
    this.currentContinuousUsbHigh = 0;
    this.lastRenderedContinuousIntensity = 0;
    this.lastTransientSubmitTimeMs = 0;
    this.lastContinuousSubmitTimeMs = 0;
    RazerKishiHaptics.getInstance().setTimbre(0, 0, 0, true);
  }

  private stopDeviceVibration(): void {
    GamepadManager.getInstance().clearAudioDeviceHaptics();
    this.isDeviceSourceActive = false;
  }

  private stopUsbRumble(): void {
    GamepadManager.getInstance().submitAudioUsbRumble(0, 0, 0);
    this.isUsbSourceActive = false;
  }
}
