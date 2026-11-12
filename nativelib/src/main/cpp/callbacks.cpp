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
 * @file callbacks.cpp
 * @brief moonlight-common-c 回调处理实现
 * 
 * 实现从 C 库回调到 ArkTS 的机制
 * 参照 Android 的 callbacks.c 实现
 */

#define LOG_TAG "MoonlightCallbacks"
#include "callbacks.h"
#include "opus_libopus.h"
#include "video_decoder.h"
#include "audio_renderer.h"
#include "audio_haptics_engine.h"
#include "moonlight_bridge.h"
#include <hilog/log.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>

// 外部函数：更新 mic 编码器丢包率（定义在 moonlight_bridge.cpp）
extern void MicCapturerUpdatePacketLossPercent(int percent);
#include <cstdarg>
#include <cstdint>
#include <deque>
#include <mutex>
#include <qos/qos.h>
#include <sched.h>
#include <unistd.h>
#include <fstream>
#include <new>
#include <utility>
#include <vector>

extern "C" {
#include "moonlight-common-c/src/Limelight.h"
}


// =============================================================================
// 全局变量
// =============================================================================

static napi_env g_env = nullptr;
static std::mutex g_mutex;

// 回调实例
VideoDecoderCallbacks g_videoCallbacks = {0};
AudioRendererCallbacks g_audioCallbacks = {0};
ConnectionListenerCallbacks g_connCallbacks = {0};

// Opus 配置
static OPUS_MULTISTREAM_CONFIGURATION g_opusConfig;
static short* g_decodedAudioBuffer = nullptr;

// SDK Core 仅由音频解码线程持有和处理。
AudioHapticsEngine g_audioHapticsEngine;

// =============================================================================
// 辅助函数
// =============================================================================

/**
 * 创建线程安全函数
 */
static napi_status CreateThreadsafeFunction(
    napi_env env,
    napi_value callback,
    const char* name,
    napi_threadsafe_function_call_js call_js,
    napi_threadsafe_function* result,
    size_t maxQueueSize = 0
) {
    napi_value resourceName;
    napi_create_string_utf8(env, name, NAPI_AUTO_LENGTH, &resourceName);
    
    return napi_create_threadsafe_function(
        env,
        callback,
        nullptr,
        resourceName,
        maxQueueSize,
        1,  // initial_thread_count
        nullptr,
        nullptr,
        nullptr,
        call_js,
        result
    );
}

static bool ConvertSunshineHdrMetadata(const SS_HDR_METADATA* source, Smpte2086Metadata* target) {
    if (source == nullptr || target == nullptr) {
        return false;
    }

    constexpr float kChromaticityScale = 50000.0f;
    constexpr float kMinLuminanceScale = 10000.0f;
    constexpr uint16_t kChromaticityMax = 50000;
    constexpr uint32_t kMinLuminanceScaleUnits = 10000;

    auto isValidChromaticity = [](uint16_t value) {
        return value > 0 && value <= kChromaticityMax;
    };
    uint64_t maxDisplayLuminanceInMinUnits =
        static_cast<uint64_t>(source->maxDisplayLuminance) * kMinLuminanceScaleUnits;

    if (source->maxDisplayLuminance == 0 ||
        !isValidChromaticity(source->displayPrimaries[0].x) ||
        !isValidChromaticity(source->displayPrimaries[0].y) ||
        !isValidChromaticity(source->displayPrimaries[1].x) ||
        !isValidChromaticity(source->displayPrimaries[1].y) ||
        !isValidChromaticity(source->displayPrimaries[2].x) ||
        !isValidChromaticity(source->displayPrimaries[2].y) ||
        !isValidChromaticity(source->whitePoint.x) ||
        !isValidChromaticity(source->whitePoint.y) ||
        source->minDisplayLuminance > maxDisplayLuminanceInMinUnits) {
        return false;
    }

    target->redX = static_cast<float>(source->displayPrimaries[0].x) / kChromaticityScale;
    target->redY = static_cast<float>(source->displayPrimaries[0].y) / kChromaticityScale;
    target->greenX = static_cast<float>(source->displayPrimaries[1].x) / kChromaticityScale;
    target->greenY = static_cast<float>(source->displayPrimaries[1].y) / kChromaticityScale;
    target->blueX = static_cast<float>(source->displayPrimaries[2].x) / kChromaticityScale;
    target->blueY = static_cast<float>(source->displayPrimaries[2].y) / kChromaticityScale;
    target->whiteX = static_cast<float>(source->whitePoint.x) / kChromaticityScale;
    target->whiteY = static_cast<float>(source->whitePoint.y) / kChromaticityScale;
    target->maxLuminance = static_cast<float>(source->maxDisplayLuminance);
    target->minLuminance = static_cast<float>(source->minDisplayLuminance) / kMinLuminanceScale;
    target->maxContentLightLevel = static_cast<float>(source->maxContentLightLevel);
    target->maxFrameAverageLightLevel = static_cast<float>(source->maxFrameAverageLightLevel);
    return true;
}

// =============================================================================
// 回调 JS 调用函数
// =============================================================================

// 通用参数传递结构
typedef struct {
    int intParams[4];
    double doubleParams[6];
    void* ptrParam;
    int ptrSize;
} CallbackData;

struct AudioHapticCallbackData {
    AhHapticFrame frame{};
    double presentationDelayMs = 0.0;
    std::chrono::steady_clock::time_point enqueuedAt{};
};

// IR frames are high-rate and replaceable. Keep the queue bounded while
// retaining lifecycle boundaries so a stalled ArkTS thread cannot accumulate
// stale haptics or lose stream termination semantics.
struct Ds5HapticsIrFrameData {
    LI_DS5_HAPTICS_IR_FRAME_V2 frame{};
};

// ---------------------------------------------------------------------------
// Host-authored DualSense tactile PCM.
//
// Sunshine exposes the virtual DualSense USB audio endpoint as a 48 kHz
// stereo 16-bit control-stream packet. The client used to advertise only the
// simulated-DualSense capability, so the host analysed this waveform on the
// PC and shipped five lossy scalars (IR v2) instead. We now accept the raw
// waveform, measure its real envelope / band split / transients locally, and
// hand a compact 20 ms description to ArkTS. Keeping the DSP here avoids
// copying ~20 kB/s of PCM across NAPI on the control receive thread.
// ---------------------------------------------------------------------------
struct Ds5PcmFrameData {
    uint8_t flags = 0;
    uint16_t controllerNumber = 0;
    uint32_t frameCount = 0;
    double presentationTimeUs = 0;
    double leftRms = 0;
    double rightRms = 0;
    double leftLowRatio = 0;
    double rightLowRatio = 0;
    double attack = 0;
    double texture = 0;
    // v20: 4 子带频谱权重（0..1，按各子带能量归一化），驱动协议 4 谐波分量。
    double bandAmp[4] = {0.0, 0.0, 0.0, 0.0};
    uint8_t silent = 0;
};

static constexpr uint32_t PCM_WINDOW_FRAMES = 960;  // 20 ms at 48 kHz
static constexpr float PCM_LOWPASS_ALPHA = 0.021f;  // ~160 Hz tactile split
static constexpr float PCM_INPUT_GAIN = 6.0;        // authored PCM is usually low level
// v19: v18 实测纹理段被压在 L=2~8（几乎无感）、静默段持续 L=1（微嗡底噪），
// 导致"不清晰干脆"。两处修正：
//  1) 噪声门：原始频段 RMS < PCM_DEADZONE_RAW 视为真静音 -> 输出 0，
//     消除持续底噪，恢复干净静音（实测 raw≈0.003 的静默段直接归零）。
//  2) 感知扩展：pow(g, PCM_GAMMA)（GAMMA<1）非线性抬升中低档纹理，
//     原 L=2~8 提升到 L≈10~17，彼此仍可分档；强档(>1)经软膝收敛到 ≤1。
static constexpr float PCM_KNEE = 0.95f;            // v21: 软膝上限略升，给强档撞击留更多动态头
static constexpr float PCM_GAMMA = 0.6f;            // 感知扩展指数（<1 抬升低档）
static constexpr float PCM_DEADZONE_RAW = 0.004f;   // v21: 噪声门下调，让细碎雨滴(低幅瞬态)能存活
static constexpr float PCM_MIN_ENVELOPE = 0.0005f;
static constexpr float PCM_CRISP_GAIN = 0.9f;       // v22: 高频带上升沿额外加力（更强），恢复雨滴/铃铛的干脆感

// ===== v61: 相对基线跃升（energy jump）瞬态增强 =====
// v60 用「峰均比 crest」判瞬态，**实测失败**（20:43 日志）：
//   雨滴帧 crest=3.46/4.32、3.84/3.32，比落地还高 ⇒ boost=1.00 全部命中雨滴，
//   把 bandLow 仅 0.012 的弱雨滴直接抬到 outLow=1.0000（满档）。
//   ⇒ 雨滴变成持续满档震动，反而**掩蔽**了落地，用户「雨滴盖过落地」。
// 根因：DS5 触觉音轨上的雨滴本身就是**一串离散脉冲**，峰均比天然高（3~4）。
//   crest 区分不了"雨滴脉冲"与"落地冲击"，二者形态同类，只有**能量量级**不同。
// 解法：改用「本窗口频带能量 / 慢速基线」的比值 jump。
//   雨滴 bandLow 实测 0.002~0.03，基线跟随在同一量级 ⇒ jump≈1~2 ⇒ boost=0；
//   落地 bandLow 冲到 0.13~0.35，基线仍在 0.015 ⇒ jump≈10~20 ⇒ boost 满。
//   ⇒ 只抬高真正的冲击，雨滴保持 v59 已被用户认可的细腻度（"雨滴没问题"）。
static constexpr double PCM_JUMP_KNEE = 3.0;     // 加成起点（保持 v61/v62 的强度曲线）
// v65: 死区 —— jump 未超过此值时**完全不给加成**。
// 为什么不能靠提高 KNEE 来挡雨滴：v64 把 KNEE 提到 4.0，日志实测
//   jump=4.31 → boost 0.26(v61) 变 0.06(v64)，jump=3.98 → 0.20 变 0，
//   而 jump 4~8 正是**普通跳跃落地**的量级 ⇒ 整段被砍 ⇒ 用户「不如 v61」。
// 正确做法：强度曲线维持 v61（KNEE=3.0），另设死区 4.5 —— 雨滴实测最大 3.34，
// 落在死区内拿不到任何加成；中等落地（≥4.5）仍按 v61 曲线给力。
static constexpr double PCM_JUMP_DEADZONE = 4.5;
static constexpr double PCM_JUMP_SCALE = 0.2;    // (jump - KNEE) * SCALE，jump=8 → 1.0 满加成
static constexpr double PCM_TRANSIENT_BOOST = 0.85;  // 瞬态额外叠加到整形后包络（0~1 域）
// v62: 冲击帧 4 子带的保底权重。设备同时合成 4 个 (幅度,频点) 分量，
// 实际输出能量 ∝ ΣbandAmp；落地实测只有 1.46（几乎全在一个频点），
// 雨滴 1.49 —— 二者总驱动相同，这正是"落地幅度满了却不重"的量化原因。
// 抬底到 0.55 后落地 Σ≈0.55*3+1.0=2.65（+81%），且只有冲击帧生效。
// v65: 回到 v62 的 **0.55 全频抬底**（用户实测「好了一点」，是已验证有效的力度）。
// v63 的 0.90 全抬 → 变成宽频"糊"、低频重心被稀释，「还不如上一版」；已放弃。
// v64 的 0.85/0.30 分频本身没坏，但那版同时把 KNEE 提到 4.0 砍掉了中等落地，
// 无法单独判断。现在先固定回已验证的 0.55，变量单一。
static constexpr double PCM_IMPACT_BAND_FLOOR = 0.55;      // 低频~中频（k=0/1/2）
static constexpr double PCM_IMPACT_BAND_FLOOR_HIGH = 0.55;  // 高频（k=3）
static constexpr double PCM_BASE_RISE = 0.010;   // 基线上升系数（慢，≈2s 时间常数，不被单次冲击带偏）
static constexpr double PCM_BASE_FALL = 0.050;   // 基线下降系数（稍快，≈0.4s 跟上雨滴变弱）

// 慢速基线跟踪：只用于判断"本帧能量是否远高于近期常态"，不参与输出。
static inline void pcmUpdateBaseline(double &base, double band) {
    const double a = (band > base) ? PCM_BASE_RISE : PCM_BASE_FALL;
    base += (band - base) * a;
    if (base < static_cast<double>(PCM_DEADZONE_RAW)) {
        base = static_cast<double>(PCM_DEADZONE_RAW);  // 下限=噪声门，避免除零后 jump 虚高
    }
}

// 噪声门 + 6x 增益 + 感知扩展 + 0.92 软膝上限。
// 弱档被非线性抬升（纹理可感、分档清晰），强档平滑饱和到 1.0 但保持排序。
static inline double pcmShape(double raw) {
    if (raw <= static_cast<double>(PCM_DEADZONE_RAW)) {
        return 0.0;  // 噪声门：彻底静音，不再有持续微嗡
    }
    const double g = raw * static_cast<double>(PCM_INPUT_GAIN);
    const double e = std::pow(g, static_cast<double>(PCM_GAMMA));
    if (e <= static_cast<double>(PCM_KNEE)) {
        return e;
    }
    const double span = 1.0 - static_cast<double>(PCM_KNEE);
    return static_cast<double>(PCM_KNEE) +
        span * (1.0 - std::exp(-(e - PCM_KNEE) / span));
}

// v20: 4 子带 Goertzel，把 DS5 波形的真实频谱形状映射到协议的 4 谐波分量。
// 雷蛇 WYVRN 协议的频率是 0-127 索引，索引->Hz 官方未公开；这里在真实 Hz 域
// (48kHz) 做 Goertzel 取频谱形状，用经验映射填索引。关键在于「相对频谱形状」
// （哪个频带有能量）如实进入手柄合成 —— 沙地的高频分量、引擎的低频分量因此
// 能被区分，材质颗粒感即来自此。索引->Hz 的具体标定交 ArkTS 的 PCM_SPECTRAL_*。
static constexpr int PCM_BANDS = 4;
static constexpr float PCM_BAND_HZ[PCM_BANDS] = {50.0f, 180.0f, 550.0f, 1300.0f};
// v22: 频率响应预加重——马达是谐振型执行器，高频段滚降明显。
// 这里按"逆频率响应"提升各子带幅度（越高频提越多），让"脆"段即便在弱响应区也能被手感觉到。
// 数值来自 rumblebridge 对雷蛇马达的实测（低频~59/132Hz 最猛、高频骤降）的经验逆曲线。
static constexpr float PCM_BAND_PREEMP[PCM_BANDS] = {1.0f, 1.3f, 1.7f, 2.2f};
static float g_bandCoeff[PCM_BANDS] = {0.0f, 0.0f, 0.0f, 0.0f};
static bool g_bandInit = false;
static double g_pcmLastBandAmp[4] = {0.0, 0.0, 0.0, 0.0};
static void initBandCoeff() {
    if (g_bandInit) return;
    static const double kPi = 3.14159265358979323846;
    for (int k = 0; k < PCM_BANDS; ++k) {
        g_bandCoeff[k] = static_cast<float>(
            2.0 * std::cos(2.0 * kPi * static_cast<double>(PCM_BAND_HZ[k]) / 48000.0));
    }
    g_bandInit = true;
}

struct PcmAccumulator {
    uint32_t frames = 0;
    uint64_t presentationTimeUs = 0;
    double sumSq[2] = {0.0, 0.0};
    double lowSq[2] = {0.0, 0.0};
    double highSq[2] = {0.0, 0.0};
    uint32_t crossings[2] = {0, 0};
    // v60: 窗口内低/高频带的瞬时峰值（绝对值），用于计算峰均比（crest factor）。
    // 为什么必须有：RMS 会严重低估短瞬态 —— 落地冲击只占窗口 5~20% 时长，
    // RMS 只有真实峰值的 0.22~0.45 倍，实测导致「10 次跳落地丢 5 次」
    // （landing 时 bandLow 时高时低：0.35 → L=63；0.05 → L=30，与雨滴无法区分）。
    float lowPeak[2] = {0.0f, 0.0f};
    float highPeak[2] = {0.0f, 0.0f};
    float lowpass[2] = {0.0f, 0.0f};
    float previous[2] = {0.0f, 0.0f};
    // v20: Goertzel 滑动状态（跨窗口连续，避免每帧 DC 阶跃误判为低频瞬态）。
    double gPrev[2][PCM_BANDS] = {{0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}};
    double gPrev2[2][PCM_BANDS] = {{0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}};
    double gPow[2][PCM_BANDS] = {{0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}};

    // Only the per-window accumulators are cleared. The IIR state must survive
    // the window boundary, otherwise every 20 ms frame starts with a DC step
    // that the band split would report as a false low-frequency transient.
    void reset() {
        frames = 0;
        for (int c = 0; c < 2; ++c) {
            sumSq[c] = lowSq[c] = highSq[c] = 0.0;
            lowPeak[c] = highPeak[c] = 0.0f;
            crossings[c] = 0;
        }
    }
};

static PcmAccumulator g_pcmAccum;
static float g_pcmPrevRms[2] = {0.0f, 0.0f};
static bool g_pcmHasHistory = false;
static uint32_t g_pcmPacketCount = 0;
static uint32_t g_pcmFrameCount = 0;
// v67: 格式不符被丢弃的包数。此前该分支静默 return，hilog 里什么都看不到，
// 是「PCM 明明在发却没触感」这类问题的排查盲区。现在限流告警。
static uint32_t g_pcmFormatReject = 0;
// v18 遥测：每 ~1s 输出一次未整形 RMS 分布（peak/mean）与最近一帧的
// 频段 lane 值，为下一轮数据驱动的增益标定提供真实依据。
static double g_pcmRawPeak = 0.0;
static double g_pcmRawSum = 0.0;
static uint32_t g_pcmRawN = 0;
static double g_pcmLastBandLow = 0.0;
static double g_pcmLastBandHigh = 0.0;
// v19 遥测：最近一帧整形后的输出（噪声门+扩展+软膝之后），用于核对
// 噪声门是否真正归零、纹理档是否被抬升到可感区间。
static double g_pcmLastOutLow = 0.0;
static double g_pcmLastOutHigh = 0.0;
// v61: 慢速基线（用于 jump 比值判据），跟着雨滴常态走，不被单次冲击带偏。
static double g_pcmLowBaseline = 0.004;
static double g_pcmHighBaseline = 0.004;
// v62: 本帧 4 子带抬底值（0 = 非冲击帧，未抬底），用于验证落地是否真的铺开能量。
static double g_pcmLastImpactFloor = 0.0;
// v63: 本帧轻马达抑制后的保留比例（1.00 = 未压，冲击帧应 <1）。
static double g_pcmLastFloorHigh = 0.0;
// v60 遥测：最近一帧的峰均比与瞬态加成，用于确认「冲击帧 boost>0、雨滴帧 boost=0」
static double g_pcmLastCrestLow = 0.0;
static double g_pcmLastCrestHigh = 0.0;
static double g_pcmLastBoostLow = 0.0;
static double g_pcmLastBoostHigh = 0.0;

// v71: v23 的 raw PCM 直灌路径（55 AA 帧格式）已**整体删除**。
// 该协议是从 Android 开源版 PeaSyo 抄错的，v23~v30 全部静音；
// 且它会往同一条 TSFN 管道里塞 55AA 帧污染队列。连带删除的常量：
// RAZER_RAW_OUT_RATE / RAZER_RAW_IN_RATE / RAZER_RAW_SAMPLES_PER_FRAME /
// RAZER_RAW_FRAME_BYTES / g_rawOutPending。**不要再加回来。**
// 与 ArkTS 侧对应的 RazerKishiRawHaptics（RAW_PCM_ENABLED=false）同为死路径。

static void CallJs_Ds5HapticsPcm(napi_env env, napi_value js_callback,
                                 void* context, void* data);
static void PublishDs5PcmFrame(const Ds5PcmFrameData& frame);
static void CallJs_RazerRawHaptic(napi_env env, napi_value js_callback,
                                  void* context, void* data);

struct AdaptiveTriggerCallbackData {
    uint16_t controllerNumber = 0;
    uint8_t eventFlags = 0;
    uint8_t typeLeft = 0;
    uint8_t typeRight = 0;
    std::array<uint8_t, DS_EFFECT_PAYLOAD_SIZE> left{};
    std::array<uint8_t, DS_EFFECT_PAYLOAD_SIZE> right{};
};

static constexpr size_t DS5_IR_QUEUE_CAPACITY = 4;
// The common-c input path supports sixteen controller slots. Keep one pending
// adaptive-trigger state per slot so a busy ArkTS thread cannot accumulate
// stale reports or drop the newest Off state behind an unbounded TSFN queue.
static constexpr size_t ADAPTIVE_TRIGGER_QUEUE_CAPACITY = 16;
static std::deque<LI_DS5_HAPTICS_IR_FRAME_V2> g_ds5IrQueue;
static bool g_ds5IrPumpQueued = false;
static bool g_ds5IrShuttingDown = false;
static std::deque<AdaptiveTriggerCallbackData> g_adaptiveTriggerQueue;
static bool g_adaptiveTriggerPumpQueued = false;

static bool IsDs5IrBoundary(const LI_DS5_HAPTICS_IR_FRAME_V2& frame) {
    return (frame.flags & (LI_DS5_HAPTICS_IR_FLAG_DISCONTINUITY |
                           LI_DS5_HAPTICS_IR_FLAG_STREAM_END |
                           LI_DS5_HAPTICS_IR_FLAG_SILENT)) != 0U;
}

static void QueueDs5IrPump();
static void QueueAdaptiveTriggerPump();

static void CallJs_StageStarting(napi_env env, napi_value js_callback, void* context, void* data) {
    CallbackData* cbData = (CallbackData*)data;
    if (env != nullptr && js_callback != nullptr) {
        napi_value argv[1];
        napi_create_int32(env, cbData->intParams[0], &argv[0]);
        
        napi_value undefined;
        napi_get_undefined(env, &undefined);
        napi_call_function(env, undefined, js_callback, 1, argv, nullptr);
    }
    delete cbData;
}

static void CallJs_StageComplete(napi_env env, napi_value js_callback, void* context, void* data) {
    CallbackData* cbData = (CallbackData*)data;
    if (env != nullptr && js_callback != nullptr) {
        napi_value argv[1];
        napi_create_int32(env, cbData->intParams[0], &argv[0]);
        
        napi_value undefined;
        napi_get_undefined(env, &undefined);
        napi_call_function(env, undefined, js_callback, 1, argv, nullptr);
    }
    delete cbData;
}

static void CallJs_StageFailed(napi_env env, napi_value js_callback, void* context, void* data) {
    CallbackData* cbData = (CallbackData*)data;
    if (env != nullptr && js_callback != nullptr) {
        napi_value argv[2];
        napi_create_int32(env, cbData->intParams[0], &argv[0]);
        napi_create_int32(env, cbData->intParams[1], &argv[1]);
        
        napi_value undefined;
        napi_get_undefined(env, &undefined);
        napi_call_function(env, undefined, js_callback, 2, argv, nullptr);
    }
    delete cbData;
}

static void CallJs_ConnectionStarted(napi_env env, napi_value js_callback, void* context, void* data) {
    if (env != nullptr && js_callback != nullptr) {
        napi_value undefined;
        napi_get_undefined(env, &undefined);
        napi_call_function(env, undefined, js_callback, 0, nullptr, nullptr);
    }
}

static void CallJs_ConnectionTerminated(napi_env env, napi_value js_callback, void* context, void* data) {
    CallbackData* cbData = (CallbackData*)data;
    if (env != nullptr && js_callback != nullptr) {
        napi_value argv[1];
        napi_create_int32(env, cbData->intParams[0], &argv[0]);
        
        napi_value undefined;
        napi_get_undefined(env, &undefined);
        napi_call_function(env, undefined, js_callback, 1, argv, nullptr);
    }
    delete cbData;
}

static void CallJs_Rumble(napi_env env, napi_value js_callback, void* context, void* data) {
    CallbackData* cbData = (CallbackData*)data;
    if (env != nullptr && js_callback != nullptr) {
        napi_value argv[3];
        napi_create_int32(env, cbData->intParams[0], &argv[0]);
        napi_create_int32(env, cbData->intParams[1], &argv[1]);
        napi_create_int32(env, cbData->intParams[2], &argv[2]);
        
        napi_value undefined;
        napi_get_undefined(env, &undefined);
        napi_call_function(env, undefined, js_callback, 3, argv, nullptr);
    }
    delete cbData;
}

static void CallJs_ConnectionStatusUpdate(napi_env env, napi_value js_callback, void* context, void* data) {
    CallbackData* cbData = (CallbackData*)data;
    if (env != nullptr && js_callback != nullptr) {
        napi_value argv[1];
        napi_create_int32(env, cbData->intParams[0], &argv[0]);
        
        napi_value undefined;
        napi_get_undefined(env, &undefined);
        napi_call_function(env, undefined, js_callback, 1, argv, nullptr);
    }
    delete cbData;
}

static void CallJs_SetHdrMode(napi_env env, napi_value js_callback, void* context, void* data) {
    CallbackData* cbData = (CallbackData*)data;
    if (env != nullptr && js_callback != nullptr) {
        // argv: [enabled, hasMetadata, maxDisplayLuminance, minDisplayLuminance,
        //        maxContentLightLevel, maxFrameAverageLightLevel, maxFullFrameLuminance]
        napi_value argv[7];
        napi_get_boolean(env, cbData->intParams[0] != 0, &argv[0]);
        napi_get_boolean(env, cbData->intParams[1] != 0, &argv[1]);
        for (int i = 0; i < 5; i++) {
            napi_create_double(env, cbData->doubleParams[i], &argv[2 + i]);
        }

        napi_value undefined;
        napi_get_undefined(env, &undefined);
        napi_call_function(env, undefined, js_callback, 7, argv, nullptr);
    }
    delete cbData;
}

static void CallJs_SetMotionEventState(napi_env env, napi_value js_callback, void* context, void* data) {
    CallbackData* cbData = (CallbackData*)data;
    if (env != nullptr && js_callback != nullptr) {
        napi_value argv[3];
        napi_create_int32(env, cbData->intParams[0], &argv[0]); // controllerNumber
        napi_create_int32(env, cbData->intParams[1], &argv[1]); // motionType
        napi_create_int32(env, cbData->intParams[2], &argv[2]); // reportRateHz
        
        napi_value undefined;
        napi_get_undefined(env, &undefined);
        napi_call_function(env, undefined, js_callback, 3, argv, nullptr);
    }
    delete cbData;
}

static void CallJs_SetControllerLED(napi_env env, napi_value js_callback, void* context, void* data) {
    CallbackData* cbData = (CallbackData*)data;
    if (env != nullptr && js_callback != nullptr) {
        napi_value argv[4];
        napi_create_int32(env, cbData->intParams[0], &argv[0]); // controllerNumber
        napi_create_int32(env, cbData->intParams[1], &argv[1]); // r
        napi_create_int32(env, cbData->intParams[2], &argv[2]); // g
        napi_create_int32(env, cbData->intParams[3], &argv[3]); // b
        
        napi_value undefined;
        napi_get_undefined(env, &undefined);
        napi_call_function(env, undefined, js_callback, 4, argv, nullptr);
    }
    delete cbData;
}

static void CallJs_RumbleTriggers(napi_env env, napi_value js_callback, void* context, void* data) {
    CallbackData* cbData = (CallbackData*)data;
    if (env != nullptr && js_callback != nullptr) {
        napi_value argv[3];
        napi_create_int32(env, cbData->intParams[0], &argv[0]); // controllerNumber
        napi_create_int32(env, cbData->intParams[1], &argv[1]); // leftTrigger
        napi_create_int32(env, cbData->intParams[2], &argv[2]); // rightTrigger
        
        napi_value undefined;
        napi_get_undefined(env, &undefined);
        napi_call_function(env, undefined, js_callback, 3, argv, nullptr);
    }
    delete cbData;
}

static void CallJs_ResolutionChanged(napi_env env, napi_value js_callback, void* context, void* data) {
    CallbackData* cbData = (CallbackData*)data;
    if (env != nullptr && js_callback != nullptr) {
        napi_value argv[2];
        napi_create_uint32(env, cbData->intParams[0], &argv[0]);
        napi_create_uint32(env, cbData->intParams[1], &argv[1]);
        
        napi_value undefined;
        napi_get_undefined(env, &undefined);
        napi_call_function(env, undefined, js_callback, 2, argv, nullptr);
    }
    delete cbData;
}

static void CallJs_ClipboardData(napi_env env, napi_value js_callback, void* context, void* data) {
    CallbackData* cbData = (CallbackData*)data;
    if (env != nullptr && js_callback != nullptr) {
        napi_value argv[3];
        napi_create_int32(env, cbData->intParams[0], &argv[0]);
        napi_create_double(env, cbData->doubleParams[0], &argv[1]);

        void* bufferData = nullptr;
        napi_value arrayBuffer;
        napi_create_arraybuffer(env, cbData->ptrSize, &bufferData, &arrayBuffer);
        if (cbData->ptrSize > 0 && cbData->ptrParam != nullptr) {
            memcpy(bufferData, cbData->ptrParam, cbData->ptrSize);
        }
        napi_create_typedarray(env, napi_uint8_array, cbData->ptrSize, arrayBuffer, 0, &argv[2]);

        napi_value undefined;
        napi_get_undefined(env, &undefined);
        napi_call_function(env, undefined, js_callback, 3, argv, nullptr);
    }

    if (cbData != nullptr && cbData->ptrParam != nullptr) {
        free(cbData->ptrParam);
    }
    delete cbData;
}

static void CallJs_DrSetup(napi_env env, napi_value js_callback, void* context, void* data) {
    CallbackData* cbData = (CallbackData*)data;
    if (env != nullptr && js_callback != nullptr) {
        napi_value argv[4];
        napi_create_int32(env, cbData->intParams[0], &argv[0]); // videoFormat
        napi_create_int32(env, cbData->intParams[1], &argv[1]); // width
        napi_create_int32(env, cbData->intParams[2], &argv[2]); // height
        napi_create_int32(env, cbData->intParams[3], &argv[3]); // redrawRate
        
        napi_value undefined;
        napi_get_undefined(env, &undefined);
        napi_call_function(env, undefined, js_callback, 4, argv, nullptr);
    }
    delete cbData;
}

static void CallJs_DrSubmitDecodeUnit(napi_env env, napi_value js_callback, void* context, void* data) {
    CallbackData* cbData = (CallbackData*)data;
    if (env != nullptr && js_callback != nullptr && cbData->ptrParam != nullptr) {
        napi_value argv[3];
        
        // 创建 ArrayBuffer
        void* bufferData;
        napi_create_arraybuffer(env, cbData->ptrSize, &bufferData, &argv[0]);
        memcpy(bufferData, cbData->ptrParam, cbData->ptrSize);
        
        napi_create_int32(env, cbData->intParams[0], &argv[1]); // frameNumber
        napi_create_int32(env, cbData->intParams[1], &argv[2]); // frameType
        
        napi_value undefined;
        napi_get_undefined(env, &undefined);
        napi_call_function(env, undefined, js_callback, 3, argv, nullptr);
        
        free(cbData->ptrParam);
    }
    delete cbData;
}

static void CallJs_ArInit(napi_env env, napi_value js_callback, void* context, void* data) {
    CallbackData* cbData = (CallbackData*)data;
    if (env != nullptr && js_callback != nullptr) {
        napi_value argv[3];
        napi_create_int32(env, cbData->intParams[0], &argv[0]); // audioConfiguration
        napi_create_int32(env, cbData->intParams[1], &argv[1]); // sampleRate
        napi_create_int32(env, cbData->intParams[2], &argv[2]); // samplesPerFrame
        
        napi_value undefined;
        napi_get_undefined(env, &undefined);
        napi_call_function(env, undefined, js_callback, 3, argv, nullptr);
    }
    delete cbData;
}

static void CallJs_ArPlaySample(napi_env env, napi_value js_callback, void* context, void* data) {
    CallbackData* cbData = (CallbackData*)data;
    if (env != nullptr && js_callback != nullptr && cbData->ptrParam != nullptr) {
        napi_value argv[1];
        
        // 创建 Int16Array
        void* bufferData;
        napi_value arrayBuffer;
        napi_create_arraybuffer(env, cbData->ptrSize, &bufferData, &arrayBuffer);
        memcpy(bufferData, cbData->ptrParam, cbData->ptrSize);
        
        napi_create_typedarray(env, napi_int16_array, cbData->ptrSize / 2, arrayBuffer, 0, &argv[0]);
        
        napi_value undefined;
        napi_get_undefined(env, &undefined);
        napi_call_function(env, undefined, js_callback, 1, argv, nullptr);
        
        free(cbData->ptrParam);
    }
    delete cbData;
}

static void CallJs_Void(napi_env env, napi_value js_callback, void* context, void* data) {
    if (env != nullptr && js_callback != nullptr) {
        napi_value undefined;
        napi_get_undefined(env, &undefined);
        napi_call_function(env, undefined, js_callback, 0, nullptr, nullptr);
    }
}

static void SetObjectUint32(napi_env env,
                            napi_value object,
                            const char* name,
                            uint32_t value) {
    napi_value property;
    napi_create_uint32(env, value, &property);
    napi_set_named_property(env, object, name, property);
}

static void SetObjectDouble(napi_env env,
                            napi_value object,
                            const char* name,
                            double value) {
    napi_value property;
    napi_create_double(env, value, &property);
    napi_set_named_property(env, object, name, property);
}

static bool SetObjectUint8Array(napi_env env,
                                napi_value object,
                                const char* name,
                                const uint8_t* data,
                                size_t length) {
    void* bufferData = nullptr;
    napi_value arrayBuffer;
    if (napi_create_arraybuffer(env, length, &bufferData, &arrayBuffer) != napi_ok) {
        return false;
    }
    if (length > 0) {
        memcpy(bufferData, data, length);
    }

    napi_value typedArray;
    if (napi_create_typedarray(env, napi_uint8_array, length,
                               arrayBuffer, 0, &typedArray) != napi_ok) {
        return false;
    }
    return napi_set_named_property(env, object, name, typedArray) == napi_ok;
}

static void MergeAdaptiveTriggerUpdate(AdaptiveTriggerCallbackData& target,
                                       const AdaptiveTriggerCallbackData& update) {
    const uint8_t triggerFlags = static_cast<uint8_t>(
        update.eventFlags & (DS_EFFECT_RIGHT_TRIGGER | DS_EFFECT_LEFT_TRIGGER));
    target.eventFlags = static_cast<uint8_t>(target.eventFlags | triggerFlags);
    if ((triggerFlags & DS_EFFECT_RIGHT_TRIGGER) != 0U) {
        target.typeRight = update.typeRight;
        target.right = update.right;
    }
    if ((triggerFlags & DS_EFFECT_LEFT_TRIGGER) != 0U) {
        target.typeLeft = update.typeLeft;
        target.left = update.left;
    }
}

static void CallJs_SetAdaptiveTriggers(napi_env env,
                                       napi_value js_callback,
                                       void* context,
                                       void* data) {
    (void)context;
    (void)data;
    for (size_t processed = 0;
         processed < ADAPTIVE_TRIGGER_QUEUE_CAPACITY;
         ++processed) {
        AdaptiveTriggerCallbackData callbackData{};
        bool hasFrame = false;
        {
            std::lock_guard<std::mutex> lock(g_mutex);
            // The pump callback is no longer pending once it starts running.
            // Reset before checking state so an empty callback cannot strand a
            // later update behind a stale queued flag.
            g_adaptiveTriggerPumpQueued = false;
            if (!g_ds5IrShuttingDown && !g_adaptiveTriggerQueue.empty()) {
                callbackData = std::move(g_adaptiveTriggerQueue.front());
                g_adaptiveTriggerQueue.pop_front();
                hasFrame = true;
            }
        }

        if (!hasFrame) {
            break;
        }

        if (env != nullptr && js_callback != nullptr) {
            napi_value triggerObject;
            if (napi_create_object(env, &triggerObject) == napi_ok) {
                SetObjectUint32(env, triggerObject, "controllerNumber", callbackData.controllerNumber);
                SetObjectUint32(env, triggerObject, "eventFlags", callbackData.eventFlags);
                SetObjectUint32(env, triggerObject, "leftEffectType", callbackData.typeLeft);
                SetObjectUint32(env, triggerObject, "rightEffectType", callbackData.typeRight);

                const bool hasLeft = SetObjectUint8Array(
                    env, triggerObject, "leftEffect", callbackData.left.data(), callbackData.left.size());
                const bool hasRight = SetObjectUint8Array(
                    env, triggerObject, "rightEffect", callbackData.right.data(), callbackData.right.size());
                if (hasLeft && hasRight) {
                    napi_value undefined;
                    napi_get_undefined(env, &undefined);
                    napi_call_function(env, undefined, js_callback, 1, &triggerObject, nullptr);
                }
            }
        }
    }

    QueueAdaptiveTriggerPump();
}

static void SetDs5HapticsIrLane(napi_env env,
                                napi_value laneObject,
                                const LI_DS5_HAPTICS_IR_LANE_V2& lane) {
    SetObjectDouble(env, laneObject, "rmsAmplitude",
                    static_cast<double>(lane.rmsAmplitude));
    SetObjectDouble(env, laneObject, "peakAmplitude",
                    static_cast<double>(lane.peakAmplitude));
    SetObjectDouble(env, laneObject, "transientStrength",
                    static_cast<double>(lane.transientStrength));
    SetObjectDouble(env, laneObject, "lowBandRatio",
                    static_cast<double>(lane.lowBandRatio));
    SetObjectDouble(env, laneObject, "zeroCrossingRateHz",
                    static_cast<double>(lane.zeroCrossingRateHz));
}

static void CallJs_Ds5HapticsIrV2(napi_env env,
                                  napi_value js_callback,
                                  void* context,
                                  void* data) {
    (void)data;
    for (size_t processed = 0; processed < DS5_IR_QUEUE_CAPACITY; ++processed) {
        Ds5HapticsIrFrameData callbackData{};
        bool hasFrame = false;
        {
            std::lock_guard<std::mutex> lock(g_mutex);
            // This pump callback is no longer pending once it starts running.
            // Reset before checking state so an empty callback cannot strand
            // future frames behind a stale queued flag.
            g_ds5IrPumpQueued = false;
            if (!g_ds5IrShuttingDown && !g_ds5IrQueue.empty()) {
                callbackData.frame = g_ds5IrQueue.front();
                g_ds5IrQueue.pop_front();
                hasFrame = true;
            }
        }

        if (!hasFrame) {
            break;
        }

        if (env != nullptr && js_callback != nullptr) {
            napi_value frameObject;
            napi_create_object(env, &frameObject);
            SetObjectUint32(env, frameObject, "flags",
                            static_cast<uint32_t>(callbackData.frame.flags));
            SetObjectUint32(env, frameObject, "controllerNumber",
                            static_cast<uint32_t>(callbackData.frame.controllerNumber));
            SetObjectUint32(env, frameObject, "sourceSequenceNumber",
                            callbackData.frame.sourceSequenceNumber);
            SetObjectDouble(env, frameObject, "timestampUs",
                            static_cast<double>(callbackData.frame.timestampUs));
            SetObjectUint32(env, frameObject, "sourceFrameCount",
                            callbackData.frame.sourceFrameCount);
            SetObjectDouble(env, frameObject, "laneCorrelation",
                            static_cast<double>(callbackData.frame.laneCorrelation));

            napi_value lanes;
            napi_create_array_with_length(env, 2, &lanes);
            for (uint32_t index = 0; index < 2; ++index) {
                napi_value laneObject;
                napi_create_object(env, &laneObject);
                SetDs5HapticsIrLane(env, laneObject, callbackData.frame.lanes[index]);
                napi_set_element(env, lanes, index, laneObject);
            }
            napi_set_named_property(env, frameObject, "lanes", lanes);

            napi_value undefined;
            napi_get_undefined(env, &undefined);
            napi_call_function(env, undefined, js_callback, 1, &frameObject, nullptr);
        }
    }

    QueueDs5IrPump();
}

static void QueueDs5IrPump() {
    napi_threadsafe_function irTsfn = nullptr;
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        if (g_ds5IrShuttingDown || g_ds5IrPumpQueued || g_ds5IrQueue.empty()) {
            return;
        }
        irTsfn = g_connCallbacks.tsfn_ds5HapticsIrV2;
        if (irTsfn == nullptr ||
            napi_acquire_threadsafe_function(irTsfn) != napi_ok) {
            return;
        }
        g_ds5IrPumpQueued = true;
    }

    const napi_status status = napi_call_threadsafe_function(
        irTsfn, nullptr, napi_tsfn_nonblocking);
    napi_release_threadsafe_function(irTsfn, napi_tsfn_release);
    if (status != napi_ok) {
        std::lock_guard<std::mutex> lock(g_mutex);
        g_ds5IrPumpQueued = false;
    }
}

static void QueueAdaptiveTriggerPump() {
    napi_threadsafe_function triggerTsfn = nullptr;
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        if (g_ds5IrShuttingDown || g_adaptiveTriggerPumpQueued ||
            g_adaptiveTriggerQueue.empty()) {
            return;
        }
        triggerTsfn = g_connCallbacks.tsfn_setAdaptiveTriggers;
        if (triggerTsfn == nullptr ||
            napi_acquire_threadsafe_function(triggerTsfn) != napi_ok) {
            return;
        }
        g_adaptiveTriggerPumpQueued = true;
    }

    const napi_status status = napi_call_threadsafe_function(
        triggerTsfn, nullptr, napi_tsfn_nonblocking);
    napi_release_threadsafe_function(triggerTsfn, napi_tsfn_release);
    if (status != napi_ok) {
        std::lock_guard<std::mutex> lock(g_mutex);
        g_adaptiveTriggerPumpQueued = false;
    }
}

static void EnqueueAdaptiveTriggerUpdate(const AdaptiveTriggerCallbackData& update) {
    const uint8_t triggerFlags = static_cast<uint8_t>(
        update.eventFlags & (DS_EFFECT_RIGHT_TRIGGER | DS_EFFECT_LEFT_TRIGGER));
    if (triggerFlags == 0U ||
        update.controllerNumber >= ADAPTIVE_TRIGGER_QUEUE_CAPACITY) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(g_mutex);
        if (g_ds5IrShuttingDown ||
            g_connCallbacks.tsfn_setAdaptiveTriggers == nullptr) {
            return;
        }

        auto pending = std::find_if(
            g_adaptiveTriggerQueue.begin(), g_adaptiveTriggerQueue.end(),
            [&update](const AdaptiveTriggerCallbackData& queued) {
                return queued.controllerNumber == update.controllerNumber;
            });
        if (pending != g_adaptiveTriggerQueue.end()) {
            MergeAdaptiveTriggerUpdate(*pending, update);
        } else if (g_adaptiveTriggerQueue.size() < ADAPTIVE_TRIGGER_QUEUE_CAPACITY) {
            g_adaptiveTriggerQueue.push_back(update);
        } else {
            // Valid controller numbers are limited to the same sixteen slots,
            // so this is only a defensive fallback for malformed input.
            g_adaptiveTriggerQueue.front() = update;
        }
    }
    QueueAdaptiveTriggerPump();
}

static void EnqueueDs5IrFrame(const LI_DS5_HAPTICS_IR_FRAME_V2& frame) {
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        if (g_ds5IrShuttingDown || g_connCallbacks.tsfn_ds5HapticsIrV2 == nullptr) {
            return;
        }

        if (g_ds5IrQueue.size() < DS5_IR_QUEUE_CAPACITY) {
            g_ds5IrQueue.push_back(frame);
        } else if (!IsDs5IrBoundary(frame)) {
            // Normal analysis frames are replaceable; preserve queued lifecycle
            // boundaries and keep only the newest ordinary frame.
            for (auto it = g_ds5IrQueue.rbegin(); it != g_ds5IrQueue.rend(); ++it) {
                if (!IsDs5IrBoundary(*it)) {
                    *it = frame;
                    break;
                }
            }
        } else {
            // Make room for a boundary by evicting the oldest replaceable frame.
            auto it = std::find_if(g_ds5IrQueue.begin(), g_ds5IrQueue.end(),
                                   [](const LI_DS5_HAPTICS_IR_FRAME_V2& queued) {
                                       return !IsDs5IrBoundary(queued);
                                   });
            if (it != g_ds5IrQueue.end()) {
                g_ds5IrQueue.erase(it);
            } else {
                // Boundary bursts are rare; retain the newest lifecycle state.
                g_ds5IrQueue.pop_front();
            }
            g_ds5IrQueue.push_back(frame);
        }
    }
    QueueDs5IrPump();
}

static void CallJs_AudioHapticFrame(napi_env env,
                                    napi_value js_callback,
                                    void* context,
                                    void* data) {
    auto* callbackData = static_cast<AudioHapticCallbackData*>(data);
    const double bridgeDelayMs =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - callbackData->enqueuedAt).count();
    const double remainingPresentationDelayMs =
        std::max(0.0, callbackData->presentationDelayMs - bridgeDelayMs);
    if (env != nullptr && js_callback != nullptr) {
        napi_value frameObject;
        napi_create_object(env, &frameObject);
        SetObjectUint32(env, frameObject, "flags", callbackData->frame.flags);
        SetObjectDouble(env, frameObject, "timestampUs",
                        static_cast<double>(callbackData->frame.timestamp_us));
        SetObjectDouble(env, frameObject, "continuousAmplitude",
                        callbackData->frame.continuous_amplitude);
        SetObjectDouble(env, frameObject, "transientAmplitude",
                        callbackData->frame.transient_amplitude);
        SetObjectDouble(env, frameObject, "transientDurationMs",
                        callbackData->frame.transient_duration_ms);
        SetObjectDouble(env, frameObject, "sharpness",
                        callbackData->frame.sharpness);
        SetObjectDouble(env, frameObject, "lowBandRatio",
                        callbackData->frame.low_band_ratio);
        SetObjectDouble(env, frameObject, "stereoPan",
                        callbackData->frame.stereo_pan);
        SetObjectDouble(env, frameObject, "confidence",
                        callbackData->frame.confidence);
        SetObjectUint32(env, frameObject, "activeScene",
                        callbackData->frame.active_scene);
        SetObjectDouble(env, frameObject, "presentationDelayMs",
                        remainingPresentationDelayMs);

        napi_value undefined;
        napi_get_undefined(env, &undefined);
        napi_call_function(env, undefined, js_callback, 1, &frameObject, nullptr);
    }
    delete callbackData;
}

// =============================================================================
// 回调初始化
// =============================================================================

void Callbacks_Init(napi_env env, napi_value callbacks) {
    std::lock_guard<std::mutex> lock(g_mutex);
    g_env = env;
    g_ds5IrQueue.clear();
    g_ds5IrPumpQueued = false;
    g_adaptiveTriggerQueue.clear();
    g_adaptiveTriggerPumpQueued = false;
    g_ds5IrShuttingDown = false;
    g_pcmAccum.reset();
    g_pcmHasHistory = false;
    g_pcmPrevRms[0] = 0.0f;
    g_pcmPrevRms[1] = 0.0f;
    g_pcmPacketCount = 0;
    g_pcmFrameCount = 0;

    napi_value callback;
    
    // 视频解码器回调
    if (napi_get_named_property(env, callbacks, "drSetup", &callback) == napi_ok) {
        CreateThreadsafeFunction(env, callback, "drSetup", CallJs_DrSetup, &g_videoCallbacks.tsfn_setup);
    }
    if (napi_get_named_property(env, callbacks, "drStart", &callback) == napi_ok) {
        CreateThreadsafeFunction(env, callback, "drStart", CallJs_Void, &g_videoCallbacks.tsfn_start);
    }
    if (napi_get_named_property(env, callbacks, "drStop", &callback) == napi_ok) {
        CreateThreadsafeFunction(env, callback, "drStop", CallJs_Void, &g_videoCallbacks.tsfn_stop);
    }
    if (napi_get_named_property(env, callbacks, "drCleanup", &callback) == napi_ok) {
        CreateThreadsafeFunction(env, callback, "drCleanup", CallJs_Void, &g_videoCallbacks.tsfn_cleanup);
    }
    if (napi_get_named_property(env, callbacks, "drSubmitDecodeUnit", &callback) == napi_ok) {
        CreateThreadsafeFunction(env, callback, "drSubmitDecodeUnit", CallJs_DrSubmitDecodeUnit, &g_videoCallbacks.tsfn_submitDecodeUnit);
    }
    
    // 音频渲染器回调
    if (napi_get_named_property(env, callbacks, "arInit", &callback) == napi_ok) {
        CreateThreadsafeFunction(env, callback, "arInit", CallJs_ArInit, &g_audioCallbacks.tsfn_init);
    }
    if (napi_get_named_property(env, callbacks, "arStart", &callback) == napi_ok) {
        CreateThreadsafeFunction(env, callback, "arStart", CallJs_Void, &g_audioCallbacks.tsfn_start);
    }
    if (napi_get_named_property(env, callbacks, "arStop", &callback) == napi_ok) {
        CreateThreadsafeFunction(env, callback, "arStop", CallJs_Void, &g_audioCallbacks.tsfn_stop);
    }
    if (napi_get_named_property(env, callbacks, "arCleanup", &callback) == napi_ok) {
        CreateThreadsafeFunction(env, callback, "arCleanup", CallJs_Void, &g_audioCallbacks.tsfn_cleanup);
    }
    if (napi_get_named_property(env, callbacks, "arPlaySample", &callback) == napi_ok) {
        CreateThreadsafeFunction(env, callback, "arPlaySample", CallJs_ArPlaySample, &g_audioCallbacks.tsfn_playSample);
    }
    if (napi_get_named_property(env, callbacks, "audioHapticFrame", &callback) == napi_ok) {
        CreateThreadsafeFunction(env, callback, "audioHapticFrame",
                                 CallJs_AudioHapticFrame,
                                 &g_audioCallbacks.tsfn_audioHapticFrame,
                                 16);
    }
    
    // 连接监听器回调
    if (napi_get_named_property(env, callbacks, "stageStarting", &callback) == napi_ok) {
        CreateThreadsafeFunction(env, callback, "stageStarting", CallJs_StageStarting, &g_connCallbacks.tsfn_stageStarting);
    }
    if (napi_get_named_property(env, callbacks, "stageComplete", &callback) == napi_ok) {
        CreateThreadsafeFunction(env, callback, "stageComplete", CallJs_StageComplete, &g_connCallbacks.tsfn_stageComplete);
    }
    if (napi_get_named_property(env, callbacks, "stageFailed", &callback) == napi_ok) {
        CreateThreadsafeFunction(env, callback, "stageFailed", CallJs_StageFailed, &g_connCallbacks.tsfn_stageFailed);
    }
    if (napi_get_named_property(env, callbacks, "connectionStarted", &callback) == napi_ok) {
        CreateThreadsafeFunction(env, callback, "connectionStarted", CallJs_ConnectionStarted, &g_connCallbacks.tsfn_connectionStarted);
    }
    if (napi_get_named_property(env, callbacks, "connectionTerminated", &callback) == napi_ok) {
        CreateThreadsafeFunction(env, callback, "connectionTerminated", CallJs_ConnectionTerminated, &g_connCallbacks.tsfn_connectionTerminated);
    }
    if (napi_get_named_property(env, callbacks, "rumble", &callback) == napi_ok) {
        CreateThreadsafeFunction(env, callback, "rumble", CallJs_Rumble, &g_connCallbacks.tsfn_rumble);
    }
    if (napi_get_named_property(env, callbacks, "ds5HapticsIrV2", &callback) == napi_ok) {
        CreateThreadsafeFunction(env, callback, "ds5HapticsIrV2",
                                 CallJs_Ds5HapticsIrV2,
                                 &g_connCallbacks.tsfn_ds5HapticsIrV2,
                                 1);
    }
    if (napi_get_named_property(env, callbacks, "ds5HapticsPcm", &callback) == napi_ok) {
        CreateThreadsafeFunction(env, callback, "ds5HapticsPcm",
                                 CallJs_Ds5HapticsPcm,
                                 &g_connCallbacks.tsfn_ds5HapticsPcm,
                                 4);
    }
    if (napi_get_named_property(env, callbacks, "razerRawHaptic", &callback) == napi_ok) {
        CreateThreadsafeFunction(env, callback, "razerRawHaptic",
                                 CallJs_RazerRawHaptic,
                                 &g_connCallbacks.tsfn_razerRawHaptic,
                                 16);
    }
    if (napi_get_named_property(env, callbacks, "connectionStatusUpdate", &callback) == napi_ok) {
        CreateThreadsafeFunction(env, callback, "connectionStatusUpdate", CallJs_ConnectionStatusUpdate, &g_connCallbacks.tsfn_connectionStatusUpdate);
    }
    if (napi_get_named_property(env, callbacks, "setHdrMode", &callback) == napi_ok) {
        CreateThreadsafeFunction(env, callback, "setHdrMode", CallJs_SetHdrMode, &g_connCallbacks.tsfn_setHdrMode);
    }
    if (napi_get_named_property(env, callbacks, "setMotionEventState", &callback) == napi_ok) {
        CreateThreadsafeFunction(env, callback, "setMotionEventState", CallJs_SetMotionEventState, &g_connCallbacks.tsfn_setMotionEventState);
    }
    if (napi_get_named_property(env, callbacks, "setControllerLED", &callback) == napi_ok) {
        CreateThreadsafeFunction(env, callback, "setControllerLED", CallJs_SetControllerLED, &g_connCallbacks.tsfn_setControllerLED);
    }
    if (napi_get_named_property(env, callbacks, "setAdaptiveTriggers", &callback) == napi_ok) {
        CreateThreadsafeFunction(env, callback, "setAdaptiveTriggers",
                                 CallJs_SetAdaptiveTriggers,
                                 &g_connCallbacks.tsfn_setAdaptiveTriggers,
                                 1);
    }
    if (napi_get_named_property(env, callbacks, "rumbleTriggers", &callback) == napi_ok) {
        CreateThreadsafeFunction(env, callback, "rumbleTriggers", CallJs_RumbleTriggers, &g_connCallbacks.tsfn_rumbleTriggers);
    }
    if (napi_get_named_property(env, callbacks, "resolutionChanged", &callback) == napi_ok) {
        CreateThreadsafeFunction(env, callback, "resolutionChanged", CallJs_ResolutionChanged, &g_connCallbacks.tsfn_resolutionChanged);
    }
    if (napi_get_named_property(env, callbacks, "clipboardData", &callback) == napi_ok) {
        CreateThreadsafeFunction(env, callback, "clipboardData", CallJs_ClipboardData, &g_connCallbacks.tsfn_clipboardData);
    }
    
    OH_LOG_INFO(LOG_APP, "Callbacks initialized");
}

void Callbacks_Cleanup(void) {
    std::lock_guard<std::mutex> lock(g_mutex);
    g_ds5IrShuttingDown = true;
    g_ds5IrQueue.clear();
    g_ds5IrPumpQueued = false;
    g_adaptiveTriggerQueue.clear();
    g_adaptiveTriggerPumpQueued = false;
    
    // 释放线程安全函数
    if (g_videoCallbacks.tsfn_setup) napi_release_threadsafe_function(g_videoCallbacks.tsfn_setup, napi_tsfn_release);
    if (g_videoCallbacks.tsfn_start) napi_release_threadsafe_function(g_videoCallbacks.tsfn_start, napi_tsfn_release);
    if (g_videoCallbacks.tsfn_stop) napi_release_threadsafe_function(g_videoCallbacks.tsfn_stop, napi_tsfn_release);
    if (g_videoCallbacks.tsfn_cleanup) napi_release_threadsafe_function(g_videoCallbacks.tsfn_cleanup, napi_tsfn_release);
    if (g_videoCallbacks.tsfn_submitDecodeUnit) napi_release_threadsafe_function(g_videoCallbacks.tsfn_submitDecodeUnit, napi_tsfn_release);
    
    if (g_audioCallbacks.tsfn_init) napi_release_threadsafe_function(g_audioCallbacks.tsfn_init, napi_tsfn_release);
    if (g_audioCallbacks.tsfn_start) napi_release_threadsafe_function(g_audioCallbacks.tsfn_start, napi_tsfn_release);
    if (g_audioCallbacks.tsfn_stop) napi_release_threadsafe_function(g_audioCallbacks.tsfn_stop, napi_tsfn_release);
    if (g_audioCallbacks.tsfn_cleanup) napi_release_threadsafe_function(g_audioCallbacks.tsfn_cleanup, napi_tsfn_release);
    if (g_audioCallbacks.tsfn_playSample) napi_release_threadsafe_function(g_audioCallbacks.tsfn_playSample, napi_tsfn_release);
    if (g_audioCallbacks.tsfn_audioHapticFrame) {
        napi_release_threadsafe_function(
            g_audioCallbacks.tsfn_audioHapticFrame, napi_tsfn_release);
    }
    
    if (g_connCallbacks.tsfn_stageStarting) napi_release_threadsafe_function(g_connCallbacks.tsfn_stageStarting, napi_tsfn_release);
    if (g_connCallbacks.tsfn_stageComplete) napi_release_threadsafe_function(g_connCallbacks.tsfn_stageComplete, napi_tsfn_release);
    if (g_connCallbacks.tsfn_stageFailed) napi_release_threadsafe_function(g_connCallbacks.tsfn_stageFailed, napi_tsfn_release);
    if (g_connCallbacks.tsfn_connectionStarted) napi_release_threadsafe_function(g_connCallbacks.tsfn_connectionStarted, napi_tsfn_release);
    if (g_connCallbacks.tsfn_connectionTerminated) napi_release_threadsafe_function(g_connCallbacks.tsfn_connectionTerminated, napi_tsfn_release);
    if (g_connCallbacks.tsfn_rumble) napi_release_threadsafe_function(g_connCallbacks.tsfn_rumble, napi_tsfn_release);
    if (g_connCallbacks.tsfn_ds5HapticsIrV2) napi_release_threadsafe_function(g_connCallbacks.tsfn_ds5HapticsIrV2, napi_tsfn_release);
    if (g_connCallbacks.tsfn_ds5HapticsPcm) napi_release_threadsafe_function(g_connCallbacks.tsfn_ds5HapticsPcm, napi_tsfn_release);
    if (g_connCallbacks.tsfn_razerRawHaptic) napi_release_threadsafe_function(g_connCallbacks.tsfn_razerRawHaptic, napi_tsfn_release);
    if (g_connCallbacks.tsfn_connectionStatusUpdate) napi_release_threadsafe_function(g_connCallbacks.tsfn_connectionStatusUpdate, napi_tsfn_release);
    if (g_connCallbacks.tsfn_setHdrMode) napi_release_threadsafe_function(g_connCallbacks.tsfn_setHdrMode, napi_tsfn_release);
    if (g_connCallbacks.tsfn_setMotionEventState) napi_release_threadsafe_function(g_connCallbacks.tsfn_setMotionEventState, napi_tsfn_release);
    if (g_connCallbacks.tsfn_setControllerLED) napi_release_threadsafe_function(g_connCallbacks.tsfn_setControllerLED, napi_tsfn_release);
    if (g_connCallbacks.tsfn_setAdaptiveTriggers) napi_release_threadsafe_function(g_connCallbacks.tsfn_setAdaptiveTriggers, napi_tsfn_release);
    if (g_connCallbacks.tsfn_rumbleTriggers) napi_release_threadsafe_function(g_connCallbacks.tsfn_rumbleTriggers, napi_tsfn_release);
    if (g_connCallbacks.tsfn_resolutionChanged) napi_release_threadsafe_function(g_connCallbacks.tsfn_resolutionChanged, napi_tsfn_release);
    if (g_connCallbacks.tsfn_clipboardData) napi_release_threadsafe_function(g_connCallbacks.tsfn_clipboardData, napi_tsfn_release);
    
    memset(&g_videoCallbacks, 0, sizeof(g_videoCallbacks));
    memset(&g_audioCallbacks, 0, sizeof(g_audioCallbacks));
    memset(&g_connCallbacks, 0, sizeof(g_connCallbacks));
    
    // 清理 AVCodec Opus 解码器
    MoonlightOpusDecoder::Cleanup();
    
    if (g_decodedAudioBuffer) {
        free(g_decodedAudioBuffer);
        g_decodedAudioBuffer = nullptr;
    }
    
    g_env = nullptr;
    
    OH_LOG_INFO(LOG_APP, "Callbacks cleaned up");
}

// =============================================================================
// moonlight-common-c 回调桥接实现
// =============================================================================

// 视频解码器配置参数（保存用于创建解码器）
static int g_videoFormat = 0;
static int g_videoWidth = 0;
static int g_videoHeight = 0;
static int g_videoFps = 0;

// 视频解码器回调
int BridgeDrSetup(int videoFormat, int width, int height, int redrawRate, void* context, int drFlags) {
    OH_LOG_INFO(LOG_APP, "BridgeDrSetup: format=0x%{public}x, %{public}dx%{public}d@%{public}d, drFlags=0x%{public}x", 
                videoFormat, width, height, redrawRate, drFlags);
    
    // 先清理之前的解码器资源（如果有的话）
    VideoDecoderInstance::Cleanup();
    
    // 保存视频参数
    g_videoFormat = videoFormat;
    g_videoWidth = width;
    g_videoHeight = height;
    g_videoFps = redrawRate;
    
    // 初始化视频解码器（如果 Surface 已设置，会立即创建解码器）
    int ret = VideoDecoderInstance::Setup(videoFormat, width, height, redrawRate);
    if (ret != 0) {
        OH_LOG_ERROR(LOG_APP, "VideoDecoderInstance::Setup failed: %{public}d", ret);
    }
    
    // 通知 ArkTS 层设置视频参数
    if (g_videoCallbacks.tsfn_setup) {
        CallbackData* data = new CallbackData();
        data->intParams[0] = videoFormat;
        data->intParams[1] = width;
        data->intParams[2] = height;
        data->intParams[3] = redrawRate;
        napi_call_threadsafe_function(g_videoCallbacks.tsfn_setup, data, napi_tsfn_blocking);
    }
    
    OH_LOG_INFO(LOG_APP, "BridgeDrSetup completed with ret=%{public}d", ret);
    return ret;
}

void BridgeDrStart(void) {
    OH_LOG_INFO(LOG_APP, "BridgeDrStart: starting video decoder...");
    
    // 启动视频解码器（此时才真正创建解码器）
    int ret = VideoDecoderInstance::Start();
    if (ret != 0) {
        OH_LOG_ERROR(LOG_APP, "VideoDecoderInstance::Start failed: %{public}d", ret);
        // BridgeDrStart 返回类型是 void，无法返回错误
        // 但解码器会在 SubmitDecodeUnit 时失败
    } else {
        OH_LOG_INFO(LOG_APP, "BridgeDrStart: video decoder started successfully");
    }
    
    if (g_videoCallbacks.tsfn_start) {
        napi_call_threadsafe_function(g_videoCallbacks.tsfn_start, nullptr, napi_tsfn_blocking);
    }
    
    OH_LOG_INFO(LOG_APP, "BridgeDrStart: completed");
}

void BridgeDrStop(void) {
    OH_LOG_INFO(LOG_APP, "BridgeDrStop");
    
    // 停止视频解码器
    VideoDecoderInstance::Stop();
    
    if (g_videoCallbacks.tsfn_stop) {
        napi_call_threadsafe_function(g_videoCallbacks.tsfn_stop, nullptr, napi_tsfn_blocking);
    }
}

void BridgeDrCleanup(void) {
    OH_LOG_INFO(LOG_APP, "BridgeDrCleanup");
    
    // 清理视频解码器
    VideoDecoderInstance::Cleanup();
    
    if (g_videoCallbacks.tsfn_cleanup) {
        napi_call_threadsafe_function(g_videoCallbacks.tsfn_cleanup, nullptr, napi_tsfn_blocking);
    }
}

int BridgeDrSubmitDecodeUnit(void* decodeUnitPtr) {
    PDECODE_UNIT decodeUnit = (PDECODE_UNIT)decodeUnitPtr;
    
    // 计算总大小和分段数量
    int totalSize = 0;
    int segmentCount = 0;
    PLENTRY entry = decodeUnit->bufferList;
    while (entry != nullptr) {
        totalSize += entry->length;
        segmentCount++;
        entry = entry->next;
    }
    
    // 构建 scatter-gather 分段数组（栈上分配，避免动态内存）
    // 典型帧由 SPS/PPS/VPS + NAL 组成，分段数很少（通常 < 16）
    constexpr int kMaxStackSegments = 32;
    BufferSegment stackSegments[kMaxStackSegments];
    BufferSegment* segments = stackSegments;
    
    // 极端情况下分段数超限，回退到堆分配
    bool heapAllocated = false;
    if (segmentCount > kMaxStackSegments) {
        segments = new BufferSegment[segmentCount];
        heapAllocated = true;
    }
    
    int i = 0;
    entry = decodeUnit->bufferList;
    while (entry != nullptr) {
        segments[i].data = reinterpret_cast<const uint8_t*>(entry->data);
        segments[i].length = entry->length;
        i++;
        entry = entry->next;
    }
    
    // 直接提交到硬件解码器（scatter-gather 零拷贝：链表数据直写 AVBuffer）
    int result = VideoDecoderInstance::SubmitDecodeUnitScatter(
        segments,
        segmentCount,
        totalSize,
        decodeUnit->frameNumber, 
        decodeUnit->frameType,
        decodeUnit->frameHostProcessingLatency,
        static_cast<int64_t>(decodeUnit->presentationTimeUs)
    );
    
    if (heapAllocated) {
        delete[] segments;
    }
    
    // 注意: 不再通过 tsfn_submitDecodeUnit 通知 ArkTS 层
    // 之前每帧都分配 CallbackData (ptrParam=nullptr) 并 dispatch 到 JS 线程，
    // 但 CallJs_DrSubmitDecodeUnit 检测到 ptrParam==null 后直接 delete，
    // 导致 ~60Hz 的无用 new/delete + 事件循环唤醒，累积 GC 压力引起顿卡。
    // 视频帧已在本地直接提交硬件解码器，无需再通知 JS。
    
    return result == 0 ? DR_OK : DR_NEED_IDR;
}

// 音频渲染器回调
int BridgeArInit(int audioConfiguration, void* opusConfigPtr, void* context, int flags) {
    POPUS_MULTISTREAM_CONFIGURATION opusConfig = (POPUS_MULTISTREAM_CONFIGURATION)opusConfigPtr;
    OH_LOG_INFO(LOG_APP, "BridgeArInit: config=%{public}d, sampleRate=%{public}d, channels=%{public}d", 
                audioConfiguration, opusConfig->sampleRate, opusConfig->channelCount);
    
    // 保存配置
    memcpy(&g_opusConfig, opusConfig, sizeof(g_opusConfig));
    
    // 使用 HarmonyOS AVCodec Opus 解码器
    int err = MoonlightOpusDecoder::Init(opusConfig);
    if (err != 0) {
        OH_LOG_ERROR(LOG_APP, "Failed to create AVCodec Opus decoder: %{public}d", err);
        return -1;
    }
    
    // 分配解码缓冲区（先释放旧的，避免由上轮 Cleanup 未完整走完导致的泄漏 / size 不一致越界）
    if (g_decodedAudioBuffer) {
        free(g_decodedAudioBuffer);
        g_decodedAudioBuffer = nullptr;
    }
    g_decodedAudioBuffer = (short*)malloc(opusConfig->channelCount * opusConfig->samplesPerFrame * sizeof(short));
    
    // 初始化音频播放器
    err = AudioRendererInstance::Init(opusConfig->sampleRate, opusConfig->channelCount, opusConfig->samplesPerFrame);
    if (err != 0) {
        OH_LOG_ERROR(LOG_APP, "Failed to init audio renderer: %{public}d", err);
        // 继续执行，让 ArkTS 层处理音频
    }
    
    if (!g_audioHapticsEngine.Init(opusConfig->sampleRate,
                                   opusConfig->channelCount)) {
        OH_LOG_ERROR(LOG_APP,
                     "Audio haptics SDK init failed: rate=%{public}d, ch=%{public}d",
                     opusConfig->sampleRate, opusConfig->channelCount);
    } else {
        OH_LOG_INFO(
            LOG_APP,
            "Audio haptics SDK initialized: version=%{public}s params=%{public}s "
            "rate=%{public}d ch=%{public}d",
            ah_get_version_string(), ah_get_parameter_set_version(),
            opusConfig->sampleRate, opusConfig->channelCount);
    }
    
    if (g_audioCallbacks.tsfn_init) {
        CallbackData* data = new CallbackData();
        data->intParams[0] = audioConfiguration;
        data->intParams[1] = opusConfig->sampleRate;
        data->intParams[2] = opusConfig->samplesPerFrame;
        napi_call_threadsafe_function(g_audioCallbacks.tsfn_init, data, napi_tsfn_blocking);
    }
    
    return 0;
}

void BridgeArStart(void) {
    OH_LOG_INFO(LOG_APP, "BridgeArStart");
    
    // 启动音频播放器
    AudioRendererInstance::Start();
    
    if (g_audioCallbacks.tsfn_start) {
        napi_call_threadsafe_function(g_audioCallbacks.tsfn_start, nullptr, napi_tsfn_blocking);
    }
}

void BridgeArStop(void) {
    OH_LOG_INFO(LOG_APP, "BridgeArStop");
    
    // 停止音频播放器
    AudioRendererInstance::Stop();
    g_audioHapticsEngine.Reset();
    
    if (g_audioCallbacks.tsfn_stop) {
        napi_call_threadsafe_function(g_audioCallbacks.tsfn_stop, nullptr, napi_tsfn_blocking);
    }
}

void BridgeArCleanup(void) {
    OH_LOG_INFO(LOG_APP, "BridgeArCleanup");
    
    // 清理音频播放器
    AudioRendererInstance::Cleanup();
    
    // 清理 AVCodec Opus 解码器
    MoonlightOpusDecoder::Cleanup();
    g_audioHapticsEngine.Cleanup();
    
    if (g_decodedAudioBuffer) {
        free(g_decodedAudioBuffer);
        g_decodedAudioBuffer = nullptr;
    }
    
    if (g_audioCallbacks.tsfn_cleanup) {
        napi_call_threadsafe_function(g_audioCallbacks.tsfn_cleanup, nullptr, napi_tsfn_blocking);
    }
}

void BridgeArDecodeAndPlaySample(char* sampleData, int sampleLength) {
    // DIRECT_SUBMIT 模式下，此函数运行在 AudioRecv 线程
    // 设置 QoS 和大核绑定以降低调度延迟（thread_local 确保每线程只执行一次）
    static thread_local bool audioThreadSetup = false;
    if (!audioThreadSetup) {
        audioThreadSetup = true;
        
        // 设置 QoS：优先 USER_INTERACTIVE（最高等级）
        int qosRet = OH_QoS_SetThreadQoS(QOS_USER_INTERACTIVE);
        if (qosRet == 0) {
            OH_LOG_INFO(LOG_APP, "AudioRecv thread QoS: USER_INTERACTIVE (highest)");
        } else {
            qosRet = OH_QoS_SetThreadQoS(QOS_USER_INITIATED);
            if (qosRet == 0) {
                OH_LOG_INFO(LOG_APP, "AudioRecv thread QoS: USER_INITIATED (fallback)");
            } else {
                OH_LOG_WARN(LOG_APP, "AudioRecv thread: failed to set QoS (ret=%{public}d)", qosRet);
            }
        }
        
        // 尝试绑定大核 — 读取 cpufreq 检测高频核心
        int numCpus = sysconf(_SC_NPROCESSORS_CONF);
        if (numCpus > 1) {
            long maxFreq = 0;
            std::vector<std::pair<int, long>> cpuFreqs;
            for (int i = 0; i < numCpus; i++) {
                char path[128];
                snprintf(path, sizeof(path),
                    "/sys/devices/system/cpu/cpu%d/cpufreq/cpuinfo_max_freq", i);
                std::ifstream ifs(path);
                long freq = 0;
                if (ifs >> freq) {
                    cpuFreqs.push_back({i, freq});
                    if (freq > maxFreq) maxFreq = freq;
                }
            }
            
            // 将最高频率 80% 以上的核心视为大核
            if (maxFreq > 0) {
                long threshold = maxFreq * 80 / 100;
                cpu_set_t cpuset;
                CPU_ZERO(&cpuset);
                int bigCount = 0;
                for (auto& [cpu, freq] : cpuFreqs) {
                    if (freq >= threshold) {
                        CPU_SET(cpu, &cpuset);
                        bigCount++;
                    }
                }
                if (bigCount > 0 && bigCount < numCpus) {
                    int ret = sched_setaffinity(0, sizeof(cpuset), &cpuset);
                    if (ret == 0) {
                        OH_LOG_INFO(LOG_APP, "AudioRecv thread bound to %{public}d big cores", bigCount);
                    }
                    // 失败也不要紧，QoS 已暗示调度器优先使用大核
                }
            }
        }
    }
    
    if (g_decodedAudioBuffer == nullptr) {
        return;
    }
    
    // 使用 HarmonyOS AVCodec Opus 解码器
    // 注意：sampleData 可能为 NULL（丢包补偿 PLC），MoonlightOpusDecoder::Decode 内部会处理
    const bool isPlcFrame = sampleData == nullptr || sampleLength <= 0;
    int decodeLen = MoonlightOpusDecoder::Decode(
        (const unsigned char*)sampleData,
        sampleLength,
        g_decodedAudioBuffer,
        g_opusConfig.samplesPerFrame
    );

    // ---- 诊断采样：每 ~1s 输出一行 PCM 统计，异常帧立即 WARN ----
    // 用于排查"断连后重连卡爆音 + 手柄乱震"类问题
    // sequence 关联：BridgeArInit/Cleanup 已有 INFO 日志，便于按时间轴对位
    static thread_local uint64_t s_arvFrameSeq = 0;
    static thread_local uint64_t s_arvDecodeFailCount = 0;
    static thread_local uint64_t s_arvPlcFrameCount = 0;
    static thread_local int64_t  s_arvSatSum = 0;        // saturated sample 累计
    static thread_local int64_t  s_arvAbsSum = 0;
    static thread_local int      s_arvMaxAbs = 0;
    static thread_local int      s_arvSampleCount = 0;
    static thread_local int      s_arvHapticFrames = 0;
    static thread_local int      s_arvHapticAmplitudeMax = 0;
    static thread_local uint64_t s_arvHapticErrors = 0;
    static thread_local int64_t  s_arvHapticProcessMaxUs = 0;
    static thread_local uint64_t s_arvHapticProcessOverBudget = 0;
    static thread_local uint64_t s_arvHapticQueueDrops = 0;
    constexpr int kSatThreshold = 30000;
    constexpr int kFramesPerLog = 200;  // ~1s @5ms/frame

    s_arvFrameSeq++;
    if (isPlcFrame) {
        ++s_arvPlcFrameCount;
    }
    if (decodeLen <= 0) {
        s_arvDecodeFailCount++;
        if (s_arvDecodeFailCount <= 3 || (s_arvDecodeFailCount % 50) == 0) {
            OH_LOG_WARN(LOG_APP, "[AUDIO_DIAG] decode failed/empty: seq=%{public}llu len=%{public}d failCount=%{public}llu",
                        (unsigned long long)s_arvFrameSeq, decodeLen, (unsigned long long)s_arvDecodeFailCount);
        }
    }

    if (decodeLen > 0) {
        // 始终写入解码后的音频，不在解码层丢帧
        // 延迟控制由环形缓冲区内部处理：满时丢弃旧数据、写入新数据
        // 这样波形始终连续，避免丢帧导致的电流滋啦声
        AudioRendererInstance::PlaySamples(g_decodedAudioBuffer, decodeLen);

        // 累计本帧 PCM 统计
        const int totalSamples = decodeLen * g_opusConfig.channelCount;
        int frameMaxAbs = 0;
        int64_t frameAbsSum = 0;
        int frameSat = 0;
        for (int i = 0; i < totalSamples; i++) {
            int v = g_decodedAudioBuffer[i];
            int a = v < 0 ? -v : v;
            if (a > frameMaxAbs) frameMaxAbs = a;
            frameAbsSum += a;
            if (a >= kSatThreshold) frameSat++;
        }
        s_arvSatSum += frameSat;
        s_arvAbsSum += frameAbsSum;
        s_arvSampleCount += totalSamples;
        if (frameMaxAbs > s_arvMaxAbs) s_arvMaxAbs = frameMaxAbs;

        // 单帧异常告警：饱和率 > 50%
        if (totalSamples > 0 && frameSat * 2 > totalSamples) {
            OH_LOG_WARN(LOG_APP, "[AUDIO_DIAG] frame saturation: seq=%{public}llu satRatio=%{public}d%% maxAbs=%{public}d totalSamples=%{public}d",
                        (unsigned long long)s_arvFrameSeq,
                        (int)(frameSat * 100 / totalSamples),
                        frameMaxAbs, totalSamples);
        }

        // Portable SDK: decoded PCM -> sparse haptic IR. This runs after the
        // PCM has been queued, so it must never be allowed to build an
        // unbounded ArkTS backlog or hide an audio underrun in diagnostics.
        std::array<AhHapticFrame, 32> hapticFrames{};
        uint32_t hapticFrameCount = 0;
        const auto hapticProcessStart = std::chrono::steady_clock::now();
        if (!g_audioHapticsEngine.ProcessFrame(
                g_decodedAudioBuffer,
                static_cast<uint32_t>(decodeLen),
                hapticFrames.data(),
                static_cast<uint32_t>(hapticFrames.size()),
                &hapticFrameCount)) {
            ++s_arvHapticErrors;
            if (s_arvHapticErrors <= 3U || (s_arvHapticErrors % 200U) == 0U) {
                OH_LOG_WARN(LOG_APP,
                            "[AUDIO_HAPTICS] SDK process failed: count=%{public}llu",
                            static_cast<unsigned long long>(s_arvHapticErrors));
            }
        } else if (hapticFrameCount > 0) {
            napi_threadsafe_function hapticTsfn = nullptr;
            {
                std::lock_guard<std::mutex> lock(g_mutex);
                hapticTsfn = g_audioCallbacks.tsfn_audioHapticFrame;
                if (hapticTsfn != nullptr &&
                    napi_acquire_threadsafe_function(hapticTsfn) != napi_ok) {
                    hapticTsfn = nullptr;
                }
            }

            if (hapticTsfn != nullptr) {
                // The decoded PCM is ahead of audible output by the current ring
                // buffer depth. Subtract a conservative actuator lead and let
                // ArkTS schedule the sparse frame on the presentation timeline.
                constexpr double kActuatorLeadMs = 8.0;
                // Bluetooth and processed routes can legitimately exceed
                // 50 ms. The renderer clock validates the route estimate
                // before it reaches this bounded scheduling window.
                constexpr double kMaximumPresentationDelayMs = 300.0;
                const double presentationDelayMs = std::clamp(
                    AudioRendererInstance::GetPresentationLatencyMs() -
                        kActuatorLeadMs,
                    0.0, kMaximumPresentationDelayMs);

                for (uint32_t index = 0; index < hapticFrameCount; ++index) {
                    auto* data = new AudioHapticCallbackData();
                    data->frame = hapticFrames[index];
                    data->presentationDelayMs = presentationDelayMs;
                    data->enqueuedAt = std::chrono::steady_clock::now();
                    const float amplitude = std::max(
                        data->frame.continuous_amplitude,
                        data->frame.transient_amplitude);
                    s_arvHapticAmplitudeMax = std::max(
                        s_arvHapticAmplitudeMax,
                        static_cast<int>(std::lround(amplitude * 100.0F)));
                    ++s_arvHapticFrames;
                    const napi_status status = napi_call_threadsafe_function(
                        hapticTsfn, data, napi_tsfn_nonblocking);
                    if (status != napi_ok) {
                        ++s_arvHapticQueueDrops;
                        delete data;
                    }
                }
                napi_release_threadsafe_function(
                    hapticTsfn, napi_tsfn_release);
            }
        }
        const int64_t hapticProcessUs = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - hapticProcessStart).count();
        s_arvHapticProcessMaxUs = std::max(s_arvHapticProcessMaxUs, hapticProcessUs);
        if (hapticProcessUs > 2000) {
            ++s_arvHapticProcessOverBudget;
        }
    }

    // v35 修正：官方编码器【不再】喂游戏声。触觉源改为「主机下发的 DS5 触觉 PCM」，
    // 在 BridgeClDs5HapticsPcm() 中喂入编码器（见该函数内）。此处只保留解码与
    // 0x0E 保底合成（0x0E 在编码器活跃时自动让路）。

    if ((s_arvFrameSeq % kFramesPerLog) == 0 && s_arvSampleCount > 0) {
        const int satPct = (int)(s_arvSatSum * 100 / s_arvSampleCount);
        const int meanAbs = (int)(s_arvAbsSum / s_arvSampleCount);
        const AudioRendererStats audioStats = AudioRendererInstance::GetStats();
        OH_LOG_INFO(LOG_APP,
            "[AUDIO_DIAG] window seq=%{public}llu frames=%{public}d samples=%{public}d "
            "maxAbs=%{public}d meanAbs=%{public}d satPct=%{public}d%% "
            "hapticFrames=%{public}d amplitudeMax=%{public}d "
            "hapticErrors=%{public}llu decodeFails=%{public}llu "
            "plcFrames=%{public}llu "
            "hapticMaxUs=%{public}lld hapticOver2ms=%{public}llu "
            "hapticQueueDrops=%{public}llu "
            "audioUnderruns=%{public}u audioDropped=%{public}llu audioBufferedMs=%{public}.2f",
            (unsigned long long)s_arvFrameSeq, kFramesPerLog,
            s_arvSampleCount, s_arvMaxAbs, meanAbs, satPct,
            s_arvHapticFrames, s_arvHapticAmplitudeMax,
            static_cast<unsigned long long>(s_arvHapticErrors),
            (unsigned long long)s_arvDecodeFailCount,
            static_cast<unsigned long long>(s_arvPlcFrameCount),
            static_cast<long long>(s_arvHapticProcessMaxUs),
            static_cast<unsigned long long>(s_arvHapticProcessOverBudget),
            static_cast<unsigned long long>(s_arvHapticQueueDrops),
            audioStats.underruns,
            static_cast<unsigned long long>(audioStats.droppedSamples),
            audioStats.latencyMs);
        s_arvSatSum = 0;
        s_arvAbsSum = 0;
        s_arvSampleCount = 0;
        s_arvMaxAbs = 0;
        s_arvHapticFrames = 0;
        s_arvHapticAmplitudeMax = 0;
        s_arvHapticProcessMaxUs = 0;
        s_arvHapticProcessOverBudget = 0;
        s_arvHapticQueueDrops = 0;
        // decodeFails 不清零，便于看累计
    }
}

// 连接监听器回调
void BridgeClStageStarting(int stage) {
    OH_LOG_INFO(LOG_APP, "Stage starting: %{public}d", stage);
    if (g_connCallbacks.tsfn_stageStarting) {
        CallbackData* data = new CallbackData();
        data->intParams[0] = stage;
        napi_call_threadsafe_function(g_connCallbacks.tsfn_stageStarting, data, napi_tsfn_blocking);
    }
}

void BridgeClStageComplete(int stage) {
    OH_LOG_INFO(LOG_APP, "Stage complete: %{public}d", stage);
    if (g_connCallbacks.tsfn_stageComplete) {
        CallbackData* data = new CallbackData();
        data->intParams[0] = stage;
        napi_call_threadsafe_function(g_connCallbacks.tsfn_stageComplete, data, napi_tsfn_blocking);
    }
}

void BridgeClStageFailed(int stage, int errorCode) {
    OH_LOG_ERROR(LOG_APP, "Stage failed: %{public}d, error: %{public}d", stage, errorCode);
    if (g_connCallbacks.tsfn_stageFailed) {
        CallbackData* data = new CallbackData();
        data->intParams[0] = stage;
        data->intParams[1] = errorCode;
        napi_call_threadsafe_function(g_connCallbacks.tsfn_stageFailed, data, napi_tsfn_blocking);
    }
}

void BridgeClConnectionStarted(void) {
    OH_LOG_INFO(LOG_APP, "Connection started");
    if (g_connCallbacks.tsfn_connectionStarted) {
        napi_call_threadsafe_function(g_connCallbacks.tsfn_connectionStarted, nullptr, napi_tsfn_blocking);
    }
}

void BridgeClConnectionTerminated(int errorCode) {
    OH_LOG_INFO(LOG_APP, "Connection terminated: %{public}d", errorCode);
    MoonBridge_OnConnectionTerminated();
    if (g_connCallbacks.tsfn_connectionTerminated) {
        CallbackData* data = new CallbackData();
        data->intParams[0] = errorCode;
        napi_call_threadsafe_function(g_connCallbacks.tsfn_connectionTerminated, data, napi_tsfn_blocking);
    }
}

void BridgeClRumble(unsigned short controllerNumber, unsigned short lowFreqMotor, unsigned short highFreqMotor) {
    if (g_connCallbacks.tsfn_rumble) {
        CallbackData* data = new CallbackData();
        data->intParams[0] = controllerNumber;
        data->intParams[1] = lowFreqMotor;
        data->intParams[2] = highFreqMotor;
        napi_status st = napi_call_threadsafe_function(g_connCallbacks.tsfn_rumble, data, napi_tsfn_nonblocking);
        if (st != napi_ok) delete data;
    }
}

static void CallJs_Ds5HapticsPcm(napi_env env, napi_value js_callback,
                                 void* context, void* data) {
    (void)context;
    Ds5PcmFrameData* frame = static_cast<Ds5PcmFrameData*>(data);
    if (frame == nullptr) return;
    if (env != nullptr && js_callback != nullptr) {
        napi_value object;
        if (napi_create_object(env, &object) == napi_ok) {
            SetObjectUint32(env, object, "flags", frame->flags);
            SetObjectUint32(env, object, "controllerNumber", frame->controllerNumber);
            SetObjectUint32(env, object, "frameCount", frame->frameCount);
            SetObjectUint32(env, object, "silent", frame->silent);
            SetObjectDouble(env, object, "presentationTimeUs", frame->presentationTimeUs);
            SetObjectDouble(env, object, "leftRms", frame->leftRms);
            SetObjectDouble(env, object, "rightRms", frame->rightRms);
            SetObjectDouble(env, object, "leftLowRatio", frame->leftLowRatio);
            SetObjectDouble(env, object, "rightLowRatio", frame->rightLowRatio);
            SetObjectDouble(env, object, "attack", frame->attack);
            SetObjectDouble(env, object, "texture", frame->texture);
            napi_value bandArr;
            if (napi_create_array_with_length(env, PCM_BANDS, &bandArr) == napi_ok) {
                for (int k = 0; k < PCM_BANDS; ++k) {
                    napi_value v;
                    if (napi_create_double(env, frame->bandAmp[k], &v) == napi_ok) {
                        napi_set_element(env, bandArr, k, v);
                    }
                }
                napi_set_named_property(env, object, "bandAmp", bandArr);
            }

            napi_value undefined;
            napi_get_undefined(env, &undefined);
            napi_call_function(env, undefined, js_callback, 1, &object, nullptr);
        }
    }
    delete frame;
}

static void PublishDs5PcmFrame(const Ds5PcmFrameData& frame) {
    // v67: 这里原来有一段离线诊断落盘（每帧 fopen /data/local/tmp/ds5_pcm_diag.log
    // + fprintf + fflush）。50Hz 帧流下等于在音频回调线程每秒做 50 次同步 I/O，
    // 且本机 app 对该目录并无写权限，属定版清理漏项。已删除，勿再加回来。
    napi_threadsafe_function tsfn = g_connCallbacks.tsfn_ds5HapticsPcm;
    if (tsfn == nullptr) return;
    Ds5PcmFrameData* payload = new (std::nothrow) Ds5PcmFrameData(frame);
    if (payload == nullptr) return;
    const napi_status status = napi_call_threadsafe_function(
        tsfn, payload, napi_tsfn_nonblocking);
    if (status != napi_ok) delete payload;
}

// v71: 官方编码器（libwin_codec.so）相关代码与开关**已全部删除**。
// 原因有二，缺一不可：
//   1) 合规：libwin_codec.so 是从商业 APK（PeaSyo Android 2.1.0）提取的
//      Razer Interhaptics 私有实现，没有任何可再分发授权，随 HAP 分发即侵权；
//   2) 体感：v52~v57 实测编码器 12B/3 频带位流的上限低于 0x0E 软件合成，
//      且与增益/削波/AGC 全无关，故 0x0E 为唯一正式链路。
// 曾经为了「技术档案」保留 live 直灌实现，但那样会让私有库的调用者有理由
// 重新编译进来；直接删干净，才能保证二进制里再也出不来那段代码。
// 若将来真要再验证编码器：私有库副本在工程外 reverse/denise-prebuilt/arm64-v8a/，
// 需自行重建适配层，**不要**把 lib 放回打包路径。

// v23: 55AA 直灌路径已删除，见上方 CallJs_* 声明处说明。
// （FeedRazerRawPcm 已于 v71 删除：55AA 是错误的帧格式，实测全部静音。）

static void CallJs_RazerRawHaptic(napi_env env, napi_value js_callback, void* context, void* data) {
    (void)context;
    std::vector<uint8_t>* frames = static_cast<std::vector<uint8_t>*>(data);
    if (frames == nullptr) return;
    if (env != nullptr && js_callback != nullptr) {
        napi_value arrayBuffer;
        void* buf = nullptr;
        if (napi_create_arraybuffer(env, frames->size(), &buf, &arrayBuffer) == napi_ok && buf != nullptr) {
            memcpy(buf, frames->data(), frames->size());
            napi_value typedArray;
            if (napi_create_typedarray(env, napi_uint8_array, frames->size(), arrayBuffer, 0, &typedArray) == napi_ok) {
                napi_value undefined;
                napi_get_undefined(env, &undefined);
                napi_call_function(env, undefined, js_callback, 1, &typedArray, nullptr);
            }
        }
    }
    delete frames;
}

void BridgeClDs5HapticsPcm(const LI_DS5_HAPTICS_PCM_FRAME* frame) {
    initBandCoeff();
    if (frame == nullptr || frame->pcmData == nullptr || frame->frameCount == 0) {
        return;
    }
    constexpr uint8_t STREAM_END = LI_DS5_HAPTICS_PCM_FLAG_STREAM_END;
    constexpr uint8_t DISCONTINUITY = LI_DS5_HAPTICS_PCM_FLAG_DISCONTINUITY;

    if ((frame->flags & (STREAM_END | DISCONTINUITY)) != 0) {
        // Flush what we have, then publish an explicit silent frame so ArkTS
        // can mute immediately instead of waiting for a hold timer.
        Ds5PcmFrameData silent;
        silent.controllerNumber = frame->controllerNumber;
        silent.flags = frame->flags;
        silent.silent = 1;
        silent.presentationTimeUs = static_cast<double>(frame->presentationTimeUs);
        PublishDs5PcmFrame(silent);
        g_pcmAccum.reset();
        g_pcmHasHistory = false;
        return;
    }

    if (frame->sampleRate != 48000 || frame->channelCount != 2 ||
            frame->bitsPerSample != 16) {
        // v67: 不再静默丢弃。格式不符意味着整条 0x0E 触感链路没有输入，
        // 但过去这里一条日志都不打，只能看到 packets= 不涨而无法判断原因。
        ++g_pcmFormatReject;
        if (g_pcmFormatReject == 1 || g_pcmFormatReject % 2000 == 0) {
            OH_LOG_WARN(LOG_APP,
                "DS5 haptics PCM 格式不符，已丢弃 %{public}u 包: "
                "rate=%{public}d ch=%{public}d bits=%{public}d frameCount=%{public}d",
                g_pcmFormatReject, static_cast<int>(frame->sampleRate),
                static_cast<int>(frame->channelCount),
                static_cast<int>(frame->bitsPerSample),
                static_cast<int>(frame->frameCount));
        }
        return;
    }
    if (++g_pcmPacketCount == 1 || g_pcmPacketCount % 200 == 0) {
        OH_LOG_INFO(LOG_APP,
            "DS5 haptics PCM stream: packets=%{public}u frames=%{public}u "
            "lastPacketFrames=%{public}u rate=%{public}u",
            g_pcmPacketCount, g_pcmFrameCount, frame->frameCount, frame->sampleRate);
        OH_LOG_INFO(LOG_APP,
            "DS5 PCM envelope: rawPeak=%{public}.4f rawMean=%{public}.4f "
            "bandLow=%{public}.4f bandHigh=%{public}.4f "
            "outLow=%{public}.4f outHigh=%{public}.4f "
            "jump=%{public}.2f/%{public}.2f base=%{public}.4f/%{public}.4f "
            "boost=%{public}.2f/%{public}.2f floor=%{public}.2f hf=%{public}.2f "
            "bandAmp=%{public}.2f/%{public}.2f/%{public}.2f/%{public}.2f",
            g_pcmRawPeak, g_pcmRawN > 0 ? g_pcmRawSum / g_pcmRawN : 0.0,
            g_pcmLastBandLow, g_pcmLastBandHigh,
            g_pcmLastOutLow, g_pcmLastOutHigh,
            g_pcmLastCrestLow, g_pcmLastCrestHigh,
            g_pcmLowBaseline, g_pcmHighBaseline,
            g_pcmLastBoostLow, g_pcmLastBoostHigh, g_pcmLastImpactFloor, g_pcmLastFloorHigh,
            g_pcmLastBandAmp[0], g_pcmLastBandAmp[1],
            g_pcmLastBandAmp[2], g_pcmLastBandAmp[3]);
        g_pcmRawPeak = 0.0;
        g_pcmRawSum = 0.0;
        g_pcmRawN = 0;
    }
    const uint8_t* pcm = frame->pcmData;
    const uint32_t frames = frame->frameCount;
    if (frame->pcmDataLength < frames * 4U) {
        return;
    }

    // v71: 原先这里有两段死支路，均已删除：
    //   1) FeedRazerRawPcm(pcm, frames) —— v36 起注释停用，55AA 协议本身是错的；
    //   2) if (kLiveEncoderEnabled) { …DeniseEncoder_FeedLivePcm… } —— 官方编码器
    //      直灌，v57 起恒不执行，v71 连同实现一起删掉（合规原因见上方说明）。
    // ⚠️ 下面这段 0x0E 的 PCM 分析链是**唯一正式链路**，任何改动都要极其谨慎。

    PcmAccumulator& acc = g_pcmAccum;
    if (acc.frames == 0) {
        acc.presentationTimeUs = frame->presentationTimeUs;
    }

    for (uint32_t i = 0; i < frames; ++i) {
        const size_t offset = static_cast<size_t>(i) * 4U;
        for (int c = 0; c < 2; ++c) {
            const uint8_t* p = pcm + offset + c * 2U;
            int16_t raw = static_cast<int16_t>(
                static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8));
            const float x = static_cast<float>(raw) / 32768.0f;
            acc.lowpass[c] += PCM_LOWPASS_ALPHA * (x - acc.lowpass[c]);
            const float low = acc.lowpass[c];
            const float high = x - low;
            acc.sumSq[c] += static_cast<double>(x) * x;
            acc.lowSq[c] += static_cast<double>(low) * low;
            acc.highSq[c] += static_cast<double>(high) * high;
            // v60: 跟踪带内瞬时峰值（峰均比的分子）
            const float aLow = std::fabs(low);
            const float aHigh = std::fabs(high);
            if (aLow > acc.lowPeak[c]) acc.lowPeak[c] = aLow;
            if (aHigh > acc.highPeak[c]) acc.highPeak[c] = aHigh;
            if ((x >= 0.0f) != (acc.previous[c] >= 0.0f)) {
                acc.crossings[c]++;
            }
            acc.previous[c] = x;
            // v20: 对原始全频带采样更新 4 子带 Goertzel 滑动状态。
            for (int k = 0; k < PCM_BANDS; ++k) {
                const float s = x + g_bandCoeff[k] * static_cast<float>(acc.gPrev[c][k])
                    - static_cast<float>(acc.gPrev2[c][k]);
                acc.gPrev2[c][k] = acc.gPrev[c][k];
                acc.gPrev[c][k] = s;
            }
        }
        acc.frames++;
    }

    if (acc.frames < PCM_WINDOW_FRAMES) {
        return;  // keep accumulating; host sends ~5 ms packets
    }

    Ds5PcmFrameData out;
    out.controllerNumber = frame->controllerNumber;
    out.flags = frame->flags;
    out.frameCount = acc.frames;
    out.presentationTimeUs = static_cast<double>(acc.presentationTimeUs);

    double rms[2];
    double attack = 0.0;
    double texture = 0.0;
    for (int c = 0; c < 2; ++c) {
        rms[c] = std::sqrt(acc.sumSq[c] / acc.frames);
        const double bandTotal = acc.lowSq[c] + acc.highSq[c];
        const double lowRatio = bandTotal > 1e-12 ? acc.lowSq[c] / bandTotal : 0.0;
        if (c == 0) out.leftLowRatio = lowRatio;
        else out.rightLowRatio = lowRatio;
        // A rising envelope is a real waveform transient, not a heuristic.
        const double delta = rms[c] - g_pcmPrevRms[c];
        attack = std::max(attack,
            g_pcmHasHistory ? std::min(1.0, delta / 0.02) : 0.0);
        // Zero crossings per second, normalised against a 3 kHz tactile band.
        const double zcr = static_cast<double>(acc.crossings[c]) * 48000.0 / acc.frames;
        texture = std::max(texture, std::min(1.0, zcr / 3000.0));
        g_pcmPrevRms[c] = static_cast<float>(rms[c]);
    }
    // v18: DS5 双通道是左右执行器，内容近乎相同；直接 L->重马达 / R->轻马达
    // 拿不到频谱分离（v17 实测两路输出几乎一致，颗粒感只能靠 tex 一个参数）。
    // 改用实测频段能量做 lane：低频带包络 -> 重通道，高频带包络 -> 轻通道，
    // 沙砾/金属等材质颗粒真正落在轻马达上。
    const double lowBand = std::sqrt(
        (acc.lowSq[0] + acc.lowSq[1]) / (2.0 * acc.frames));
    const double highBand = std::sqrt(
        (acc.highSq[0] + acc.highSq[1]) / (2.0 * acc.frames));
    // v19: 噪声门 + 感知扩展 + 软膝上限，输出即整形后的包络。
    // v61: 叠加「相对基线跃升」瞬态增强 —— 只有能量远超近期常态的冲击帧受益，
    //      雨滴与基线同量级 ⇒ jump≈1~2 ⇒ 加成恒为 0，细腻度与 v59 完全一致。
    pcmUpdateBaseline(g_pcmLowBaseline, lowBand);
    pcmUpdateBaseline(g_pcmHighBaseline, highBand);
    const double jumpLow = lowBand / g_pcmLowBaseline;
    const double jumpHigh = highBand / g_pcmHighBaseline;
    const double boostLow = (jumpLow > PCM_JUMP_DEADZONE && lowBand > static_cast<double>(PCM_DEADZONE_RAW))
        ? std::max(0.0, std::min(1.0, (jumpLow - PCM_JUMP_KNEE) * PCM_JUMP_SCALE))
        : 0.0;
    const double boostHigh = (jumpHigh > PCM_JUMP_DEADZONE && highBand > static_cast<double>(PCM_DEADZONE_RAW))
        ? std::max(0.0, std::min(1.0, (jumpHigh - PCM_JUMP_KNEE) * PCM_JUMP_SCALE))
        : 0.0;
    out.leftRms = std::min(1.0, pcmShape(lowBand) + boostLow * PCM_TRANSIENT_BOOST);
    out.rightRms = std::min(1.0, pcmShape(highBand) + boostHigh * PCM_TRANSIENT_BOOST);
    // v64: **[v63 的轻马达压制已撤销]**
    // v63 用 `out.rightRms *= (1 - 0.55*impact)` 压低轻马达来减少掩蔽，实测失败：
    //   ① 雨滴帧 jumpHigh 会偶尔到 3.29~3.34（只差一点点越门槛）⇒ 也拿到微小 boost
    //      ⇒ impact>0.01 成立 ⇒ 轻马达被压 ⇒ 雨滴纹理被打断；
    //   ② 更致命：ArkTS `renderPcmFrame` 的余震锁定 `pcmHoldHigh` 记的是**压制后**的值，
    //      落地时 high 被压到 ~28，**之后 120ms 余震期也一直被压在低值**
    //      ⇒ 每次落地后都会丢一段雨滴纹理 ⇒ 用户实测「雨滴有不震动的真空期」。
    // ⇒ 结论：**不能在已认可的通路上做减法**。想加重只能加低频（下面 bandAmp 那条），
    //   不能压高频。此路今后不要再走。
    // 遥测：整形前的真实 RMS 分布 + 本帧频段 lane + 整形后输出（每秒随周期日志输出）。
    {
        const double rawOverall = (rms[0] + rms[1]) * 0.5;
        if (rawOverall > g_pcmRawPeak) g_pcmRawPeak = rawOverall;
        g_pcmRawSum += rawOverall;
        ++g_pcmRawN;
        g_pcmLastBandLow = lowBand;
        g_pcmLastBandHigh = highBand;
        g_pcmLastOutLow = out.leftRms;
        g_pcmLastOutHigh = out.rightRms;
        g_pcmLastCrestLow = jumpLow;
        g_pcmLastCrestHigh = jumpHigh;
        g_pcmLastBoostLow = boostLow;
        g_pcmLastBoostHigh = boostHigh;
    }
    // v20: 4 子带 Goertzel 频谱权重（携带 DS5 真实频谱指纹到 4 谐波分量）。
    for (int c = 0; c < 2; ++c) {
        double maxPow = 1e-12;
        for (int k = 0; k < PCM_BANDS; ++k) {
            const double p = acc.gPrev[c][k] * acc.gPrev[c][k]
                + acc.gPrev2[c][k] * acc.gPrev2[c][k]
                - static_cast<double>(g_bandCoeff[k]) * acc.gPrev[c][k] * acc.gPrev2[c][k];
            acc.gPow[c][k] = p;
            if (p > maxPow) maxPow = p;
        }
        for (int k = 0; k < PCM_BANDS; ++k) {
            double a = (maxPow > 1e-9)
                ? std::sqrt(acc.gPow[c][k] / maxPow) : 0.0;
            // v22: 频率响应预加重——马达高频段滚降，逆响应提升高频带幅度，
            // 让"脆"段（雨滴/铃铛）即便在弱响应区也能被手感觉到。
            a = std::min(1.0, a * static_cast<double>(PCM_BAND_PREEMP[k]));
            out.bandAmp[k] = a;
        }
        // v21: 高频带(子带3)上升沿锐化——雨滴/铃铛敲击是高频瞬态，
        // 相对上一帧的跳变额外加力，让"脆"从频谱权重上凸显出来。
        const double onset = out.bandAmp[3] - g_pcmLastBandAmp[3];
        if (onset > 0.0) {
            out.bandAmp[3] = std::min(1.0, out.bandAmp[3] + onset * static_cast<double>(PCM_CRISP_GAIN));
        }
    }
    for (int k = 0; k < PCM_BANDS; ++k) g_pcmLastBandAmp[k] = out.bandAmp[k];
    // ===== v62: 冲击帧 4 子带抬底（决定「落地够不够重」的关键）=====
    // v61 实测：落地帧 bandLow=0.13~0.35、outLow 已满 1.0000、boost 也满，
    // 但用户仍「落地震动还是不够」。原因不在档位，在**能量只挂在一个频点**：
    //   落地 bandAmp=1.00/0.24/0.16/0.06 ⇒ Σ≈1.46
    //   雨滴 bandAmp=1.00/0.49/0.00/0.00 ⇒ Σ≈1.49
    // bandAmp 是「按本窗口最强子带归一化」的**相对**指纹，Σ 恒定在 1.5 上下；
    // 而设备把 4 个 (幅度,频点) 分量**同时合成**，实际输出能量 ∝ Σ。
    // ⇒ 落地把 63 档全给了一个频点，另外三个几乎为 0，总驱动只有满配的 ~36%，
    //   所以"档位满了、体感还是弱"。雨滴 Σ 与落地相同，故雨滴一直"没问题"。
    // 修法：冲击帧（jump 判据给出的 boost 高）给 4 个子带一个**保底**，
    // 让能量在频域铺开（真实重冲击本就是宽带的）。雨滴 boost=0 ⇒ 抬底为 0
    // ⇒ 频谱指纹与细腻度完全不变（"只增不减"约束）。
    // v63: 保底 0.55 → 0.90（用户实测"好了一点、还不够"，同向加大）。
    // v64: 改为**分频抬底** —— 低频 k=0/1/2 抬 0.85（冲击变厚），
    //      高频 k=3 只抬 0.30（保留脆/碎裂质感，不把高频糊成白噪音）。
    {
        const double impact = std::max(boostLow, boostHigh);
        g_pcmLastImpactFloor = 0.0;
        g_pcmLastFloorHigh = 0.0;
        if (impact > 0.01) {
            g_pcmLastImpactFloor = PCM_IMPACT_BAND_FLOOR * impact;
            g_pcmLastFloorHigh = PCM_IMPACT_BAND_FLOOR_HIGH * impact;
            for (int k = 0; k < PCM_BANDS; ++k) {
                const double floorAmp = ((k == PCM_BANDS - 1)
                    ? PCM_IMPACT_BAND_FLOOR_HIGH : PCM_IMPACT_BAND_FLOOR) * impact;
                if (out.bandAmp[k] < floorAmp) out.bandAmp[k] = floorAmp;
            }
        }
    }
    out.attack = std::max(0.0, attack);
    out.texture = texture;
    g_pcmHasHistory = true;

    const bool silent = out.leftRms < PCM_MIN_ENVELOPE && out.rightRms < PCM_MIN_ENVELOPE;
    out.silent = silent ? 1 : 0;
    PublishDs5PcmFrame(out);
    ++g_pcmFrameCount;
    acc.reset();
}

void BridgeClDs5HapticsIrV2(const LI_DS5_HAPTICS_IR_FRAME_V2* frame) {
    if (frame == nullptr) {
        return;
    }

    if ((frame->flags & (LI_DS5_HAPTICS_IR_FLAG_DISCONTINUITY |
                         LI_DS5_HAPTICS_IR_FLAG_STREAM_END)) != 0U) {
        OH_LOG_INFO(LOG_APP,
                    "DS5 IR stream boundary: controller=%{public}u flags=0x%{public}x",
                    frame->controllerNumber, frame->flags);
    }

    // The IR stream is a separate source from ordinary rumble. Preserve the
    // normalized lanes and lifecycle flags for ArkTS, where source-aware
    // mixing and actuator selection are performed.
    EnqueueDs5IrFrame(*frame);
}

void BridgeClConnectionStatusUpdate(int connectionStatus) {
    OH_LOG_INFO(LOG_APP, "Connection status: %{public}d", connectionStatus);
    
    // 网络质量差时主动请求 IDR 帧，加速画面恢复
    // CONN_STATUS_POOR = 1，此时很可能有帧丢失导致解码器参考帧损坏
    // 提前请求 IDR 可避免等待超时才触发恢复
    if (connectionStatus == 1) { // CONN_STATUS_POOR
        LiRequestIdrFrame();
        OH_LOG_WARN(LOG_APP, "Poor connection detected, proactively requesting IDR frame");
    }
    
    // 动态更新所有 mic 编码器的丢包率预估
    // POOR → 15%（增加 FEC 冗余），OKAY → 1%（恢复正常）
    {
        int lossPercent = (connectionStatus == 1) ? 15 : 1;
        MicCapturerUpdatePacketLossPercent(lossPercent);
    }
    
    if (g_connCallbacks.tsfn_connectionStatusUpdate) {
        CallbackData* data = new CallbackData();
        data->intParams[0] = connectionStatus;
        napi_status st = napi_call_threadsafe_function(g_connCallbacks.tsfn_connectionStatusUpdate, data, napi_tsfn_nonblocking);
        if (st != napi_ok) delete data;
    }
}

void BridgeClSetHdrMode(bool enabled, void* hdrMetadata) {
    OH_LOG_INFO(LOG_APP, "Set HDR mode: %{public}d, metadata=%{public}p", enabled ? 1 : 0, hdrMetadata);

    if (enabled && hdrMetadata != nullptr) {
        Smpte2086Metadata convertedMetadata = {};
        if (ConvertSunshineHdrMetadata(static_cast<const SS_HDR_METADATA*>(hdrMetadata), &convertedMetadata)) {
            VideoDecoderInstance::SetHdrStaticMetadata(&convertedMetadata);
            OH_LOG_INFO(LOG_APP, "HDR metadata applied: R=(%.4f,%.4f), G=(%.4f,%.4f), B=(%.4f,%.4f), "
                        "WP=(%.4f,%.4f), maxLum=%.3f, minLum=%.4f, maxCLL=%.3f, maxFALL=%.3f",
                        convertedMetadata.redX, convertedMetadata.redY,
                        convertedMetadata.greenX, convertedMetadata.greenY,
                        convertedMetadata.blueX, convertedMetadata.blueY,
                        convertedMetadata.whiteX, convertedMetadata.whiteY,
                        convertedMetadata.maxLuminance, convertedMetadata.minLuminance,
                        convertedMetadata.maxContentLightLevel,
                        convertedMetadata.maxFrameAverageLightLevel);
        } else {
            VideoDecoderInstance::SetHdrStaticMetadata(nullptr);
            OH_LOG_WARN(LOG_APP, "HDR metadata missing or invalid, decoder will use fallback static metadata");
        }
    } else {
        VideoDecoderInstance::SetHdrStaticMetadata(nullptr);
    }
    
    // 通知 ArkTS 层 HDR 状态变化;透传 SS_HDR_METADATA 原始亮度字段
    // (不经 Smpte2086 转换,那会丢 minLuminance 缩放语义与 full-frame 字段)
    if (g_connCallbacks.tsfn_setHdrMode) {
        CallbackData* data = new CallbackData();
        data->intParams[0] = enabled;
        if (enabled && hdrMetadata != nullptr) {
            const SS_HDR_METADATA* meta = static_cast<const SS_HDR_METADATA*>(hdrMetadata);
            data->intParams[1] = 1;
            data->doubleParams[0] = static_cast<double>(meta->maxDisplayLuminance);
            data->doubleParams[1] = static_cast<double>(meta->minDisplayLuminance);
            data->doubleParams[2] = static_cast<double>(meta->maxContentLightLevel);
            data->doubleParams[3] = static_cast<double>(meta->maxFrameAverageLightLevel);
            data->doubleParams[4] = static_cast<double>(meta->maxFullFrameLuminance);
        }
        napi_status st = napi_call_threadsafe_function(g_connCallbacks.tsfn_setHdrMode, data, napi_tsfn_nonblocking);
        if (st != napi_ok) delete data;
    }
}

void BridgeClRumbleTriggers(unsigned short controllerNumber, unsigned short leftTrigger, unsigned short rightTrigger) {
    if (g_connCallbacks.tsfn_rumbleTriggers) {
        CallbackData* data = new CallbackData();
        data->intParams[0] = controllerNumber;
        data->intParams[1] = leftTrigger;
        data->intParams[2] = rightTrigger;
        napi_status st = napi_call_threadsafe_function(g_connCallbacks.tsfn_rumbleTriggers, data, napi_tsfn_nonblocking);
        if (st != napi_ok) delete data;
    }
}

void BridgeClSetMotionEventState(unsigned short controllerNumber, unsigned char motionType, unsigned short reportRateHz) {
    OH_LOG_INFO(LOG_APP, "SetMotionEventState: controller=%u, type=%u, rate=%u", controllerNumber, motionType, reportRateHz);
    if (g_connCallbacks.tsfn_setMotionEventState) {
        CallbackData* data = new CallbackData();
        data->intParams[0] = controllerNumber;
        data->intParams[1] = motionType;
        data->intParams[2] = reportRateHz;
        napi_status st = napi_call_threadsafe_function(g_connCallbacks.tsfn_setMotionEventState, data, napi_tsfn_nonblocking);
        if (st != napi_ok) delete data;
    }
}

void BridgeClSetControllerLED(unsigned short controllerNumber, unsigned char r, unsigned char g, unsigned char b) {
    if (g_connCallbacks.tsfn_setControllerLED) {
        CallbackData* data = new CallbackData();
        data->intParams[0] = controllerNumber;
        data->intParams[1] = r;
        data->intParams[2] = g;
        data->intParams[3] = b;
        napi_status st = napi_call_threadsafe_function(g_connCallbacks.tsfn_setControllerLED, data, napi_tsfn_nonblocking);
        if (st != napi_ok) delete data;
    }
}

void BridgeClSetAdaptiveTriggers(unsigned short controllerNumber, unsigned char eventFlags,
                                 unsigned char typeLeft, unsigned char typeRight,
                                 unsigned char* left, unsigned char* right) {
    if (left == nullptr || right == nullptr) {
        return;
    }

    AdaptiveTriggerCallbackData data{};
    data.controllerNumber = controllerNumber;
    data.eventFlags = eventFlags;
    data.typeLeft = typeLeft;
    data.typeRight = typeRight;
    std::copy_n(left, data.left.size(), data.left.begin());
    std::copy_n(right, data.right.size(), data.right.begin());
    EnqueueAdaptiveTriggerUpdate(data);
}

void BridgeClResolutionChanged(unsigned int width, unsigned int height) {
    OH_LOG_INFO(LOG_APP, "Resolution changed: %ux%u", width, height);
    if (g_connCallbacks.tsfn_resolutionChanged) {
        CallbackData* data = new CallbackData();
        data->intParams[0] = width;
        data->intParams[1] = height;
        napi_call_threadsafe_function(g_connCallbacks.tsfn_resolutionChanged, data, napi_tsfn_blocking);
    }
}

void BridgeClClipboardData(const char* data, int length) {
    static constexpr int kClipboardWireHeaderSize = 10;
    static constexpr uint32_t kMaxClipboardPayloadBytes = 65500 - kClipboardWireHeaderSize;

    if (g_connCallbacks.tsfn_clipboardData == nullptr || data == nullptr || length < 10) {
        return;
    }

    const uint8_t* frame = reinterpret_cast<const uint8_t*>(data);
    const uint8_t version = frame[0];
    const uint8_t kind = frame[1];
    const uint32_t token =
        static_cast<uint32_t>(frame[2]) |
        (static_cast<uint32_t>(frame[3]) << 8) |
        (static_cast<uint32_t>(frame[4]) << 16) |
        (static_cast<uint32_t>(frame[5]) << 24);
    const uint32_t payloadLength =
        static_cast<uint32_t>(frame[6]) |
        (static_cast<uint32_t>(frame[7]) << 8) |
        (static_cast<uint32_t>(frame[8]) << 16) |
        (static_cast<uint32_t>(frame[9]) << 24);
    const int availablePayloadLength = length - kClipboardWireHeaderSize;

    if (version != 1) {
        OH_LOG_WARN(LOG_APP, "BridgeClClipboardData: unsupported version=%{public}u", version);
        return;
    }

    if (payloadLength != static_cast<uint32_t>(availablePayloadLength)) {
        OH_LOG_WARN(LOG_APP, "BridgeClClipboardData: malformed payload len=%{public}u available=%{public}d",
                    payloadLength, availablePayloadLength);
        return;
    }

    if (payloadLength > kMaxClipboardPayloadBytes ||
        static_cast<uint32_t>(availablePayloadLength) > kMaxClipboardPayloadBytes) {
        OH_LOG_WARN(LOG_APP,
                    "BridgeClClipboardData: payload too large len=%{public}u available=%{public}d max=%{public}u",
                    payloadLength, availablePayloadLength, kMaxClipboardPayloadBytes);
        return;
    }

    CallbackData* cbData = new CallbackData();
    memset(cbData, 0, sizeof(*cbData));
    cbData->intParams[0] = static_cast<int>(kind);
    cbData->doubleParams[0] = static_cast<double>(token);
    cbData->ptrSize = availablePayloadLength;

    if (availablePayloadLength > 0) {
        cbData->ptrParam = malloc(availablePayloadLength);
        if (cbData->ptrParam == nullptr) {
            delete cbData;
            OH_LOG_ERROR(LOG_APP, "BridgeClClipboardData: malloc failed for %{public}d bytes", availablePayloadLength);
            return;
        }
        memcpy(cbData->ptrParam, frame + kClipboardWireHeaderSize, availablePayloadLength);
    }

    napi_status st = napi_call_threadsafe_function(g_connCallbacks.tsfn_clipboardData, cbData, napi_tsfn_blocking);
    if (st != napi_ok) {
        if (cbData->ptrParam != nullptr) {
            free(cbData->ptrParam);
        }
        delete cbData;
    }
}

void BridgeClLogMessage(const char* format, ...) {
    va_list args;
    va_start(args, format);
    char buffer[1024];
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    OH_LOG_INFO(LOG_APP, "[Moonlight] %{public}s", buffer);
}
