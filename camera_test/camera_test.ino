#include <Arduino.h>
#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "img_converters.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"

// -----------------------------------------------------------------------------
// XIAO ESP32-S3 Sense camera test for "A Sole's Travel Diary"
//
// What this sketch does:
//   1. Captures QVGA RGB565 frames at 8 fps into a 5-frame PSRAM ring buffer.
//   2. Detects heel contact with an FSR voltage divider on D0 / GPIO1.
//   3. After two post-contact frames arrive, picks the sharpest frame in the ring.
//   4. Rejects it when its small grayscale thumbnail resembles the last saved one.
//   5. JPEG-encodes accepted frames and saves them under /shoe on microSD.
//
// FSR wiring (the ADC value rises when pressed):
//
//       3.3 V
//         |
//       FSR UX 402
//         |
//         +---------- D0 / GPIO1
//         |
//        10 kOhm
//         |
//        GND
//
// Arduino IDE settings:
//   Board: "XIAO_ESP32S3"
//   PSRAM: "OPI PSRAM"
//
// Send 't' in Serial Monitor to simulate a landing without pressing the FSR.
// -----------------------------------------------------------------------------

// XIAO ESP32-S3 Sense camera pins.
constexpr int PWDN_GPIO_NUM = -1;
constexpr int RESET_GPIO_NUM = -1;
constexpr int XCLK_GPIO_NUM = 10;
constexpr int SIOD_GPIO_NUM = 40;
constexpr int SIOC_GPIO_NUM = 39;
constexpr int Y9_GPIO_NUM = 48;
constexpr int Y8_GPIO_NUM = 11;
constexpr int Y7_GPIO_NUM = 12;
constexpr int Y6_GPIO_NUM = 14;
constexpr int Y5_GPIO_NUM = 16;
constexpr int Y4_GPIO_NUM = 18;
constexpr int Y3_GPIO_NUM = 17;
constexpr int Y2_GPIO_NUM = 15;
constexpr int VSYNC_GPIO_NUM = 38;
constexpr int HREF_GPIO_NUM = 47;
constexpr int PCLK_GPIO_NUM = 13;

// ---- Experiment settings -----------------------------------------------------

constexpr int FSR_PIN = 1;  // D0 / GPIO1, an ADC-capable pin.

// These values assume: 3.3V -> FSR -> ADC -> 10kOhm -> GND.
// Watch the [fsr] Serial output and tune these for the actual insole/heel load.
constexpr uint16_t FSR_PRESS_THRESHOLD = 900;
constexpr uint16_t FSR_RELEASE_THRESHOLD = 550;
constexpr uint32_t FSR_PRESS_DEBOUNCE_MS = 25;
constexpr uint32_t FSR_RELEASE_DEBOUNCE_MS = 80;
constexpr float FSR_FILTER_ALPHA = 0.22f;

constexpr uint16_t CAMERA_WIDTH = 320;
constexpr uint16_t CAMERA_HEIGHT = 240;
constexpr size_t BYTES_PER_PIXEL = 2;  // RGB565
constexpr size_t FRAME_BYTES =
    static_cast<size_t>(CAMERA_WIDTH) * CAMERA_HEIGHT * BYTES_PER_PIXEL;

constexpr uint8_t RING_SIZE = 5;
constexpr uint8_t POST_LANDING_FRAMES = 2;
constexpr uint8_t CAPTURE_FPS = 8;
constexpr uint32_t FRAME_INTERVAL_MS = 1000 / CAPTURE_FPS;

// JPEG encoder quality is 1..100 here (higher is better/larger).
constexpr uint8_t JPEG_QUALITY = 82;

// Mean absolute difference of brightness-normalized 32x24 thumbnails.
// Lower means more similar. Raise this to discard more images.
constexpr float DUPLICATE_MAD_THRESHOLD = 10.0f;

constexpr bool CAMERA_VFLIP = false;
constexpr bool CAMERA_HMIRROR = false;

// Sense expansion-board documentation has used GPIO21 as SD CS. Some newer
// pinout sheets show GPIO3, so mountSdCard() safely tries both revisions.
constexpr int SD_CS_CANDIDATES[] = {21, 3};
constexpr int SD_SCK_PIN = 7;
constexpr int SD_MISO_PIN = 8;
constexpr int SD_MOSI_PIN = 9;
constexpr uint32_t SD_SPI_HZ = 4000000;

constexpr uint8_t THUMB_WIDTH = 32;
constexpr uint8_t THUMB_HEIGHT = 24;
constexpr size_t THUMB_PIXELS =
    static_cast<size_t>(THUMB_WIDTH) * THUMB_HEIGHT;

struct FrameSlot {
  uint8_t *pixels = nullptr;
  uint32_t capturedAtMs = 0;
  uint32_t sharpness = 0;
  bool valid = false;
};

struct JpegWriteContext {
  File *file = nullptr;
  size_t bytesWritten = 0;
  bool ok = true;
};

FrameSlot ringFrames[RING_SIZE];
uint8_t ringWriteIndex = 0;
uint8_t ringCount = 0;

bool cameraReady = false;
bool ringReady = false;
bool sdReady = false;
int activeSdCsPin = -1;

float fsrFiltered = 0.0f;
uint16_t fsrRaw = 0;
bool fsrFilterInitialized = false;
bool fsrContact = false;
uint32_t fsrAboveSinceMs = 0;
uint32_t fsrBelowSinceMs = 0;

bool landingCaptureActive = false;
uint8_t postFramesRemaining = 0;
uint32_t landingAtMs = 0;
uint16_t landingFsrPeak = 0;
uint32_t landingEventNumber = 0;

uint8_t lastSavedThumbnail[THUMB_PIXELS];
bool hasLastSavedThumbnail = false;
uint32_t nextImageNumber = 1;

uint32_t nextFrameAtMs = 0;
uint32_t lastFsrPrintMs = 0;

// カメラが返すbig-endianのRGB565を、画像評価用の8bit輝度へ変換する。
// リングバッファには色情報を残しつつ、シャープネスと類似度は輝度だけで
// 安価に計算したいため、評価時に必要な画素だけを変換する。
// pixelsはCAMERA_WIDTH x CAMERA_HEIGHTのRGB565バッファであることが前提。
inline uint8_t pixelLuma(const uint8_t *pixels, uint16_t x, uint16_t y) {
  const size_t index =
      (static_cast<size_t>(y) * CAMERA_WIDTH + x) * BYTES_PER_PIXEL;
  const uint16_t rgb565 =
      (static_cast<uint16_t>(pixels[index]) << 8) | pixels[index + 1];

  const uint8_t r5 = (rgb565 >> 11) & 0x1F;
  const uint8_t g6 = (rgb565 >> 5) & 0x3F;
  const uint8_t b5 = rgb565 & 0x1F;
  const uint8_t r8 = (r5 << 3) | (r5 >> 2);
  const uint8_t g8 = (g6 << 2) | (g6 >> 4);
  const uint8_t b8 = (b5 << 3) | (b5 >> 2);

  return static_cast<uint8_t>(
      (77U * r8 + 150U * g8 + 29U * b8) >> 8);
}

// 1枚の画像について、エッジの強さを表す相対的なシャープネス値を返す。
// 絶対的な画質判定ではなく、同じ着地前後に撮られた候補の順位付け専用。
// 全画素を処理すると目標fpsの維持に不利なので3画素おきに評価する。間隔を3に
// しているのは、4/8画素周期の模様とサンプリング位置が揃うのを避けるため。
uint32_t calculateSharpness(const uint8_t *pixels) {
  uint64_t squaredLaplacianSum = 0;
  uint32_t sampleCount = 0;

  for (uint16_t y = 1; y < CAMERA_HEIGHT - 1; y += 3) {
    for (uint16_t x = 1; x < CAMERA_WIDTH - 1; x += 3) {
      const int center = pixelLuma(pixels, x, y);
      const int laplacian =
          4 * center - pixelLuma(pixels, x - 1, y) -
          pixelLuma(pixels, x + 1, y) - pixelLuma(pixels, x, y - 1) -
          pixelLuma(pixels, x, y + 1);
      squaredLaplacianSum +=
          static_cast<int64_t>(laplacian) * laplacian;
      ++sampleCount;
    }
  }

  return sampleCount == 0
             ? 0
             : static_cast<uint32_t>(squaredLaplacianSum / sampleCount);
}

// 保存候補を類似比較するため、RGB565画像から小さな輝度サムネイルを作る。
// この解像度は床の大まかな模様を残しつつ、歩行中の計算量と保持メモリを
// 小さくするためのもの。thumbnailにはTHUMB_PIXELS分の領域が必要。
void makeThumbnail(const uint8_t *pixels, uint8_t *thumbnail) {
  for (uint8_t ty = 0; ty < THUMB_HEIGHT; ++ty) {
    const uint16_t sourceY =
        (static_cast<uint32_t>(ty) * CAMERA_HEIGHT + CAMERA_HEIGHT / 2) /
        THUMB_HEIGHT;
    for (uint8_t tx = 0; tx < THUMB_WIDTH; ++tx) {
      const uint16_t sourceX =
          (static_cast<uint32_t>(tx) * CAMERA_WIDTH + CAMERA_WIDTH / 2) /
          THUMB_WIDTH;
      const uint16_t boundedX =
          sourceX < CAMERA_WIDTH ? sourceX : CAMERA_WIDTH - 1;
      const uint16_t boundedY =
          sourceY < CAMERA_HEIGHT ? sourceY : CAMERA_HEIGHT - 1;
      thumbnail[static_cast<size_t>(ty) * THUMB_WIDTH + tx] =
          pixelLuma(pixels, boundedX, boundedY);
    }
  }
}

// 2枚のサムネイルの類似度を、平均輝度を除いた平均絶対差(MAD)で返す。
// 日なた・日陰など明るさだけが変わった同じ床を重複として扱えるよう、比較前に
// 各画像の平均輝度を差し引く。0に近いほど似ており、完全一致なら0になる。
float normalizedThumbnailMad(const uint8_t *a, const uint8_t *b) {
  uint32_t sumA = 0;
  uint32_t sumB = 0;
  for (size_t i = 0; i < THUMB_PIXELS; ++i) {
    sumA += a[i];
    sumB += b[i];
  }

  const int meanA = sumA / THUMB_PIXELS;
  const int meanB = sumB / THUMB_PIXELS;
  uint32_t differenceSum = 0;

  for (size_t i = 0; i < THUMB_PIXELS; ++i) {
    const int normalizedA = static_cast<int>(a[i]) - meanA;
    const int normalizedB = static_cast<int>(b[i]) - meanB;
    differenceSum += abs(normalizedA - normalizedB);
  }

  return static_cast<float>(differenceSum) / THUMB_PIXELS;
}

// RING_SIZE枚分のRGB565バッファをPSRAMに確保する。
// 内部RAMを圧迫するとカメラやSD処理が不安定になるため、PSRAMがない場合は
// 小さいバッファへ暗黙に縮退せず失敗させる。成功した領域は動作中ずっと保持する。
bool allocateRingBuffer() {
  if (!psramFound()) {
    Serial.println("[fatal] PSRAM was not found. Select OPI PSRAM in Arduino IDE.");
    return false;
  }

  for (uint8_t i = 0; i < RING_SIZE; ++i) {
    ringFrames[i].pixels = static_cast<uint8_t *>(heap_caps_malloc(
        FRAME_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (ringFrames[i].pixels == nullptr) {
      Serial.printf("[fatal] PSRAM allocation failed at ring slot %u.\n", i);
      for (uint8_t j = 0; j < i; ++j) {
        heap_caps_free(ringFrames[j].pixels);
        ringFrames[j].pixels = nullptr;
      }
      return false;
    }
  }

  Serial.printf("[ram] ring buffer: %u frames, %u bytes (%.1f KiB)\n",
                RING_SIZE, static_cast<unsigned>(FRAME_BYTES * RING_SIZE),
                (FRAME_BYTES * RING_SIZE) / 1024.0f);
  return true;
}

// XIAO ESP32-S3 SenseのカメラをQVGA/RGB565で初期化する。
// JPEGではなくRGB565を使うのは、候補選別を伸長処理なしで行い、選ばれた1枚だけ
// 最後にJPEG化するため。初期化後は自動露出・WBが落ち着くまで数フレーム捨てる。
// 成功後のカメラはプログラム終了まで初期化済みとして扱う。
bool initializeCamera() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_RGB565;
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;  // Not used for RGB565 capture.
  config.fb_count = 2;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_PSRAM;

  const esp_err_t error = esp_camera_init(&config);
  if (error != ESP_OK) {
    Serial.printf("[fatal] Camera init failed: 0x%x\n", error);
    return false;
  }

  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor != nullptr) {
    sensor->set_vflip(sensor, CAMERA_VFLIP ? 1 : 0);
    sensor->set_hmirror(sensor, CAMERA_HMIRROR ? 1 : 0);
  }

  for (uint8_t i = 0; i < 8; ++i) {
    camera_fb_t *frame = esp_camera_fb_get();
    if (frame != nullptr) {
      esp_camera_fb_return(frame);
    }
    delay(35);
  }

  Serial.printf("[camera] ready: %ux%u RGB565 at target %u fps\n",
                CAMERA_WIDTH, CAMERA_HEIGHT, CAPTURE_FPS);
  return true;
}

// 指定したCSピンでmicroSDをマウントできるか1回だけ試す。
// 成功時はSDをマウントしたままactiveSdCsPinを更新し、失敗時は次の候補を
// 安全に試せるようSDを終了状態へ戻す。
bool tryMountSdWithCs(int csPin) {
  Serial.printf("[sd] trying CS GPIO%d...\n", csPin);
  if (!SD.begin(csPin, SPI, SD_SPI_HZ)) {
    SD.end();
    return false;
  }
  if (SD.cardType() == CARD_NONE) {
    SD.end();
    return false;
  }
  activeSdCsPin = csPin;
  return true;
}

// Sense拡張ボードのmicroSDを初期化し、保存先の/shoeを用意する。
// Seeedの資料によってCSがGPIO21/GPIO3の記載になっているため、
// 両方を順に試す。失敗してもカメラ選別の検証は続けられるよう、致命的エラーには
// せずfalseを返す。
bool mountSdCard() {
  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, -1);

  bool mounted = false;
  for (int csPin : SD_CS_CANDIDATES) {
    if (tryMountSdWithCs(csPin)) {
      mounted = true;
      break;
    }
  }

  if (!mounted) {
    Serial.println(
        "[warning] microSD mount failed on GPIO21 and GPIO3. Capture selection "
        "will run, but images cannot be saved.");
    return false;
  }

  if (!SD.exists("/shoe") && !SD.mkdir("/shoe")) {
    Serial.println("[warning] Could not create /shoe on microSD.");
    SD.end();
    activeSdCsPin = -1;
    return false;
  }

  Serial.printf("[sd] mounted with CS GPIO%d, size %.1f MiB\n", activeSdCsPin,
                SD.cardSize() / (1024.0f * 1024.0f));
  return true;
}

// 既存JPEGを上書きしない最初の連番を探し、nextImageNumberへ設定する。
// この試作にはRTCがなく、再起動するとmillis()が戻るため、時刻ではなくSD上の
// ファイルを確認して採番する。呼び出し時はSDがマウント済みであることが前提。
void findNextImageNumber() {
  char path[40];
  for (uint32_t number = 1; number < 100000; ++number) {
    snprintf(path, sizeof(path), "/shoe/step_%05lu.jpg",
             static_cast<unsigned long>(number));
    if (!SD.exists(path)) {
      nextImageNumber = number;
      return;
    }
  }
  nextImageNumber = millis();
}

// JPEGエンコーダが生成した断片を、そのまま開いているFileへ書き出す。
// 完成JPEG用の大きな中間バッファを別途確保しないためのコールバックで、短い
// 書き込みが起きた場合はcontext.okをfalseにして呼び出し元へ失敗を伝える。
size_t jpegWriteCallback(void *argument, size_t index, const void *data,
                         size_t length) {
  (void)index;
  JpegWriteContext *context = static_cast<JpegWriteContext *>(argument);
  if (!context->ok || context->file == nullptr) {
    return 0;
  }

  const size_t written = context->file->write(
      static_cast<const uint8_t *>(data), length);
  context->bytesWritten += written;
  if (written != length) {
    context->ok = false;
  }
  return written;
}

// 選別済みのRGB565フレーム1枚だけをJPEG化してpathへ保存する。
// 候補全枚をSDへ書かないことで書き込み量と待ち時間を抑える。変換または書き込みに
// 失敗した場合は不完全なJPEGを削除し、成功時だけbytesWrittenを有効値として返す。
bool saveJpeg(const FrameSlot &frame, const char *path, size_t &bytesWritten) {
  File file = SD.open(path, FILE_WRITE);
  if (!file) {
    Serial.printf("[sd] could not open %s\n", path);
    return false;
  }

  JpegWriteContext context;
  context.file = &file;
  const bool encoded = fmt2jpg_cb(
      frame.pixels, FRAME_BYTES, CAMERA_WIDTH, CAMERA_HEIGHT,
      PIXFORMAT_RGB565, JPEG_QUALITY, jpegWriteCallback, &context);
  file.flush();
  file.close();

  bytesWritten = context.bytesWritten;
  if (!encoded || !context.ok || context.bytesWritten == 0) {
    SD.remove(path);
    Serial.printf("[sd] JPEG encode/write failed for %s\n", path);
    return false;
  }
  return true;
}

// 保存画像に対応する検証値を/shoe/index.csvへ追記する。
// JPEG自体の保存成功を優先するため、CSV追記に失敗しても画像は削除しない。
// このログは後からFSR閾値・シャープネス・類似度閾値を調整するために使う。
void appendMetadata(const char *path, const FrameSlot &frame,
                    float similarityMad) {
  const char *metadataPath = "/shoe/index.csv";
  const bool needsHeader = !SD.exists(metadataPath);
  File file = SD.open(metadataPath, FILE_APPEND);
  if (!file) {
    Serial.println("[sd] warning: could not append /shoe/index.csv");
    return;
  }

  if (needsHeader) {
    file.println(
        "event_id,filename,capture_ms,landing_ms,fsr_peak,sharpness,"
        "similarity_mad");
  }
  file.printf("%lu,%s,%lu,%lu,%u,%lu,%.2f\n",
              static_cast<unsigned long>(landingEventNumber), path,
              static_cast<unsigned long>(frame.capturedAtMs),
              static_cast<unsigned long>(landingAtMs), landingFsrPeak,
              static_cast<unsigned long>(frame.sharpness), similarityMad);
  file.close();
}

// 1回の着地についてリング内の候補を評価し、保存または重複破棄を決定する。
// 呼び出し時点のリングは、通常「着地前およそ3枚＋着地後2枚」になっている。
// 最もシャープな1枚だけを直前の保存画像と比較し、採用時はJPEGとCSVを更新する。
// 評価後はlandingCaptureActiveを解除するが、リングの内容は次の撮影で再利用する。
void processLandingEvent() {
  landingCaptureActive = false;

  FrameSlot *best = nullptr;
  Serial.printf("[event %lu] evaluating %u frame(s)\n",
                static_cast<unsigned long>(landingEventNumber), ringCount);

  for (uint8_t i = 0; i < RING_SIZE; ++i) {
    FrameSlot &candidate = ringFrames[i];
    if (!candidate.valid) {
      continue;
    }
    const int32_t offsetMs =
        static_cast<int32_t>(candidate.capturedAtMs - landingAtMs);
    Serial.printf("  slot %u: %+ld ms, sharpness=%lu\n", i,
                  static_cast<long>(offsetMs),
                  static_cast<unsigned long>(candidate.sharpness));
    if (best == nullptr || candidate.sharpness > best->sharpness) {
      best = &candidate;
    }
  }

  if (best == nullptr) {
    Serial.println("[event] no valid camera frame; skipped");
    return;
  }

  uint8_t candidateThumbnail[THUMB_PIXELS];
  makeThumbnail(best->pixels, candidateThumbnail);
  const float similarityMad =
      hasLastSavedThumbnail
          ? normalizedThumbnailMad(candidateThumbnail, lastSavedThumbnail)
          : 999.0f;

  Serial.printf("[event %lu] best sharpness=%lu, similarity MAD=%.2f\n",
                static_cast<unsigned long>(landingEventNumber),
                static_cast<unsigned long>(best->sharpness), similarityMad);

  if (hasLastSavedThumbnail &&
      similarityMad < DUPLICATE_MAD_THRESHOLD) {
    Serial.printf("[event %lu] too similar to previous saved image; discarded\n",
                  static_cast<unsigned long>(landingEventNumber));
    return;
  }

  if (!sdReady) {
    Serial.printf("[event %lu] selected but not saved because SD is unavailable\n",
                  static_cast<unsigned long>(landingEventNumber));
    return;
  }

  char path[40];
  snprintf(path, sizeof(path), "/shoe/step_%05lu.jpg",
           static_cast<unsigned long>(nextImageNumber));
  size_t jpegBytes = 0;
  if (!saveJpeg(*best, path, jpegBytes)) {
    return;
  }

  memcpy(lastSavedThumbnail, candidateThumbnail, THUMB_PIXELS);
  hasLastSavedThumbnail = true;
  appendMetadata(path, *best, similarityMad);
  ++nextImageNumber;

  Serial.printf("[event %lu] saved %s (%u bytes)\n",
                static_cast<unsigned long>(landingEventNumber), path,
                static_cast<unsigned>(jpegBytes));
}

// カメラから最新フレームを1枚取得し、所有権を持つリング領域へコピーする。
// esp_cameraのフレームはドライバへすぐ返す必要があるため、ポインタをリングに保持
// してはいけない。着地後の必要枚数が揃った場合、この関数内で選別・保存まで行う。
// カメラ取得または形式確認に失敗した場合はリングを変更せずfalseを返す。
bool captureIntoRing() {
  camera_fb_t *cameraFrame = esp_camera_fb_get();
  if (cameraFrame == nullptr) {
    Serial.println("[camera] frame capture failed");
    return false;
  }

  const bool formatMatches =
      cameraFrame->format == PIXFORMAT_RGB565 &&
      cameraFrame->width == CAMERA_WIDTH &&
      cameraFrame->height == CAMERA_HEIGHT &&
      cameraFrame->len >= FRAME_BYTES;
  if (!formatMatches) {
    Serial.printf(
        "[camera] unexpected frame: %ux%u format=%d bytes=%u; skipped\n",
        cameraFrame->width, cameraFrame->height, cameraFrame->format,
        static_cast<unsigned>(cameraFrame->len));
    esp_camera_fb_return(cameraFrame);
    return false;
  }

  FrameSlot &slot = ringFrames[ringWriteIndex];
  memcpy(slot.pixels, cameraFrame->buf, FRAME_BYTES);
  slot.capturedAtMs = millis();
  slot.sharpness = calculateSharpness(slot.pixels);
  slot.valid = true;
  esp_camera_fb_return(cameraFrame);

  ringWriteIndex = (ringWriteIndex + 1) % RING_SIZE;
  if (ringCount < RING_SIZE) {
    ++ringCount;
  }

  if (landingCaptureActive && postFramesRemaining > 0) {
    --postFramesRemaining;
    if (postFramesRemaining == 0) {
      processLandingEvent();
    }
  }
  return true;
}

// 着地イベントを開始し、着地後フレームの収集完了を待つ状態へ移す。
// この時点では保存を決めず、POST_LANDING_FRAMES枚がリングへ入ってから評価する。
// 既にイベント処理中、またはカメラ/リングが使えない場合はイベントを重ねない。
// simulatedはシリアル操作による検証か、実際のFSR検出かをログで区別するために使う。
void startLandingEvent(bool simulated) {
  if (landingCaptureActive || !cameraReady || !ringReady) {
    return;
  }

  ++landingEventNumber;
  landingAtMs = millis();
  landingFsrPeak = static_cast<uint16_t>(fsrFiltered);
  postFramesRemaining = POST_LANDING_FRAMES;
  landingCaptureActive = true;

  Serial.printf("[event %lu] %s detected, ADC=%u; waiting for %u post frame(s)\n",
                static_cast<unsigned long>(landingEventNumber),
                simulated ? "simulated landing" : "landing", landingFsrPeak,
                POST_LANDING_FRAMES);
}

// FSRを継続的に読み、安定した押下/解放イベントへ変換する。
// ローパスフィルタ、押下/解放で異なる閾値（ヒステリシス）、時間デバウンスを組み
// 合わせ、歩行時の振動や閾値付近の揺れで1歩が複数回発火するのを防ぐ。
// 押下の立ち上がりだけが撮影イベントを開始し、接触中の最大値はCSV用に保持する。
void updateFsr() {
  const uint32_t now = millis();
  fsrRaw = analogRead(FSR_PIN);
  if (!fsrFilterInitialized) {
    fsrFiltered = fsrRaw;
    fsrFilterInitialized = true;
  } else {
    fsrFiltered += FSR_FILTER_ALPHA * (fsrRaw - fsrFiltered);
  }

  if (landingCaptureActive) {
    const uint16_t filteredValue = static_cast<uint16_t>(fsrFiltered);
    if (filteredValue > landingFsrPeak) {
      landingFsrPeak = filteredValue;
    }
  }

  if (!fsrContact) {
    fsrBelowSinceMs = 0;
    if (fsrFiltered >= FSR_PRESS_THRESHOLD) {
      if (fsrAboveSinceMs == 0) {
        fsrAboveSinceMs = now;
      } else if (now - fsrAboveSinceMs >= FSR_PRESS_DEBOUNCE_MS) {
        fsrContact = true;
        fsrAboveSinceMs = 0;
        startLandingEvent(false);
      }
    } else {
      fsrAboveSinceMs = 0;
    }
  } else {
    fsrAboveSinceMs = 0;
    if (fsrFiltered <= FSR_RELEASE_THRESHOLD) {
      if (fsrBelowSinceMs == 0) {
        fsrBelowSinceMs = now;
      } else if (now - fsrBelowSinceMs >= FSR_RELEASE_DEBOUNCE_MS) {
        fsrContact = false;
        fsrBelowSinceMs = 0;
        Serial.println("[fsr] released; next landing armed");
      }
    } else {
      fsrBelowSinceMs = 0;
    }
  }

  if (now - lastFsrPrintMs >= 500) {
    lastFsrPrintMs = now;
    Serial.printf("[fsr] raw=%u filtered=%u contact=%d ring=%u/%u\n", fsrRaw,
                  static_cast<unsigned>(fsrFiltered), fsrContact, ringCount,
                  RING_SIZE);
  }
}

// シリアルから't'を受けたとき、FSR入力を使わず着地処理を開始する。
// カメラ・リング・SD側の問題とFSR配線/閾値の問題を切り分けるための診断経路。
void pollSerialTrigger() {
  while (Serial.available() > 0) {
    const char command = static_cast<char>(Serial.read());
    if (command == 't' || command == 'T') {
      startLandingEvent(true);
    }
  }
}

// 電源投入時に、FSR、PSRAM、カメラ、SDの順で試作環境を準備する。
// モバイルバッテリー単体でも動作できるようSerial接続は待ち続けない。SDだけが
// 失敗した場合は、シリアルログによる撮影・選別検証を継続できる。
void setup() {
  Serial.begin(115200);
  delay(1200);  // Do not wait forever for Serial when powered by a battery.
  Serial.println();
  Serial.println("=== A Sole's Travel Diary: shoe camera experiment ===");

  pinMode(FSR_PIN, INPUT);
  analogReadResolution(12);

  ringReady = allocateRingBuffer();
  cameraReady = ringReady && initializeCamera();
  sdReady = mountSdCard();
  if (sdReady) {
    findNextImageNumber();
  }

  nextFrameAtMs = millis();
  Serial.printf(
      "[ready] camera=%d ring=%d sd=%d, press=%u release=%u. Send 't' to "
      "simulate.\n",
      cameraReady, ringReady, sdReady, FSR_PRESS_THRESHOLD,
      FSR_RELEASE_THRESHOLD);
}

// FSR監視を優先しながら、millis()ベースでカメラを目標fpsにスケジュールする。
// JPEG変換やSD保存で遅れた後にフレームを連続取得すると「直近」の意味が崩れるため、
// 遅延分を取り戻すバースト撮影は行わず、現在時刻から次の1枚を予約し直す。
void loop() {
  updateFsr();
  pollSerialTrigger();

  const uint32_t now = millis();
  if (cameraReady && ringReady &&
      static_cast<int32_t>(now - nextFrameAtMs) >= 0) {
    captureIntoRing();

    nextFrameAtMs += FRAME_INTERVAL_MS;
    if (static_cast<int32_t>(millis() - nextFrameAtMs) >=
        static_cast<int32_t>(FRAME_INTERVAL_MS)) {
      nextFrameAtMs = millis() + FRAME_INTERVAL_MS;
    }
  }

  delay(2);
}
