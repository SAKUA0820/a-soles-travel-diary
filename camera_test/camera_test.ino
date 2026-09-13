#include <Arduino.h>
#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "img_converters.h"
#include "esp_system.h"
#include "FS.h"
#include <HTTPClient.h>
#include <Preferences.h>
#include "SD.h"
#include "SPI.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>

// -----------------------------------------------------------------------------
// 「A Sole's Travel Diary」用のXIAO ESP32-S3 Senseカメラ検証スケッチ
//
// このスケッチの動作：
//   1. QVGAのRGB565画像を8 fpsで撮影し、PSRAM上の5枚分のリングバッファに保持する。
//   2. D0 / GPIO1に接続したFSRの分圧回路で、かかとの着地を検出する。
//   3. 着地後の2枚が揃ったら、リング内で最もシャープな1枚を選ぶ。
//   4. 小さなグレースケールサムネイルが直前の保存画像と似ていれば破棄する。
//   5. 採用画像をJPEGへ変換し、microSDの/shoe配下に保存する。
//   6. かかとのホールセンサーがマットの磁石を検出したら、メタデータを先に送信し、
//      続いて選別済みのJPEGファイルをSupabaseへアップロードする。
//
// FSRの配線（押すとADCの読み取り値が上がる）：
//
//       3.3 V
//         |
//       FSR UX 402
//         |
//         +---------- D0 / GPIO1
//         |
//        10 kΩ
//         |
//        GND
//
// デジタルホールセンサーの配線（磁石検出時に出力がLOWになる部品を想定）：
//
//       ホールセンサー VCC ---------- 3.3 V
//       ホールセンサー GND ---------- GND
//       ホールセンサー OUT ---------- D1 / GPIO2
//
// Arduino IDEの設定：
//   ボード："XIAO_ESP32S3"
//   PSRAM："OPI PSRAM"
//
// シリアルモニターから送信できるコマンド：
//   t：FSRを押さずに着地を模擬する
//   u：靴をマットに置いた状態を模擬してアップロードを開始する
//   d：直近で全件転送できた1バッチのJPEGだけをmicroSDから削除する
//      Supabase上の画像とindex.csvは残す。次のバッチの転送開始で対象を入れ替える。
// -----------------------------------------------------------------------------

// XIAO ESP32-S3 Senseのカメラ用ピン割り当て。
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

// ---- 実験用の設定 ------------------------------------------------------------

constexpr int FSR_PIN = 1;  // D0 / GPIO1。ADC入力に対応するピン。

// D1 / GPIO2は、FSR・カメラ・Sense拡張ボードのSD用ピンと重複しない。
// 多くのデジタルホールスイッチは磁石が近づくとOUTがLOWになる。
// 使用する部品の出力が逆の場合は、HALL_ACTIVE_LEVELを変更する。
constexpr int HALL_PIN = 2;
constexpr uint8_t HALL_ACTIVE_LEVEL = LOW;
constexpr uint32_t HALL_DEBOUNCE_MS = 120;
constexpr uint32_t TRANSFER_RETRY_INTERVAL_MS = 30000;

// 転送の動作確認前に、以下の仮の設定値を実際の値に置き換える。SUPABASE_KEYには
// 低権限のpublishableキー（または旧anonキー）を使い、secret/service_roleキーは使わない。
constexpr char WIFI_SSID[] = "WIFI_SSID";
constexpr char WIFI_PASSWORD[] = "WIFI_PASSWORD";
constexpr char SUPABASE_URL[] = "SUPABASE_URL";
constexpr char SUPABASE_KEY[] = "SUPABASE_KEY";
constexpr char STORAGE_BUCKET[] = "a-soles-travel-diary";

constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000;
constexpr uint32_t HTTP_TIMEOUT_MS = 20000;

// 長時間の歩行で内部RAM上のJSON文字列が大きくなりすぎないよう、1回の転送枚数を制限する。
// 未転送の画像が残っている場合は、靴を一度持ち上げて置き直すと次のまとまりを転送する。
// 転送完了した画像番号はNVS（不揮発メモリ）へ記録する。
constexpr uint16_t MAX_TRANSFER_IMAGES = 128;

// 以下の値は、3.3V -> FSR -> ADC -> 10kΩ -> GNDの配線を前提とする。
// シリアルモニターの[fsr]ログを見て、実際の中敷きやかかとにかかる荷重に合わせて調整する。
constexpr uint16_t FSR_PRESS_THRESHOLD = 900;
constexpr uint16_t FSR_RELEASE_THRESHOLD = 550;
constexpr uint32_t FSR_PRESS_DEBOUNCE_MS = 25;
constexpr uint32_t FSR_RELEASE_DEBOUNCE_MS = 80;
constexpr float FSR_FILTER_ALPHA = 0.22f;

constexpr uint16_t CAMERA_WIDTH = 320;
constexpr uint16_t CAMERA_HEIGHT = 240;
constexpr size_t BYTES_PER_PIXEL = 2;  // RGB565形式（1画素2バイト）。
constexpr size_t FRAME_BYTES =
    static_cast<size_t>(CAMERA_WIDTH) * CAMERA_HEIGHT * BYTES_PER_PIXEL;

constexpr uint8_t RING_SIZE = 5;
constexpr uint8_t POST_LANDING_FRAMES = 2;
constexpr uint8_t CAPTURE_FPS = 8;
constexpr uint32_t FRAME_INTERVAL_MS = 1000 / CAPTURE_FPS;

// JPEGエンコーダの品質は1～100で指定する（大きいほど高画質になり、ファイルサイズも増える）。
constexpr uint8_t JPEG_QUALITY = 82;

// 明るさを正規化した32×24サムネイル同士の平均絶対差（MAD）の閾値。
// 差が小さいほど似ていると判断する。この閾値を上げると、破棄される画像が増える。
constexpr float DUPLICATE_MAD_THRESHOLD = 10.0f;

constexpr bool CAMERA_VFLIP = false;
constexpr bool CAMERA_HMIRROR = false;

// Sense拡張ボードのSD用CSはGPIO21と記載されているが、一部の新しいピン配置図では
// GPIO3となっているため、mountSdCard()で両方を順に試す。
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

struct TransferImage {
  uint32_t imageNumber = 0;
  uint32_t eventId = 0;
  uint32_t captureMs = 0;
  uint32_t landingMs = 0;
  uint16_t fsrPeak = 0;
  uint32_t sharpness = 0;
  float similarityMad = 0.0f;
  char filename[32] = {};
  char sdPath[48] = {};
};

enum class TransferResult {
  Success,
  NothingToUpload,
  Failed,
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

bool hallRawActive = false;
bool hallStableActive = false;
bool hallHandledForPlacement = false;
bool transferRequested = false;
uint32_t hallRawChangedAtMs = 0;
uint32_t nextTransferRetryAtMs = 0;

Preferences transferPreferences;
bool transferPreferencesReady = false;

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

// SD内の最大画像番号とNVSの転送済み番号より後から採番する。
// 削除で空いた番号を再利用すると、新しい写真が「転送済み」と判定されてしまう。
// 一覧を読めないと既存画像との衝突を避けられないため、その場合は保存を止める。
bool findNextImageNumber() {
  uint32_t highestNumber = transferPreferences.getULong("last_image", 0);
  File directory = SD.open("/shoe", FILE_READ);
  if (!directory || !directory.isDirectory()) {
    Serial.println("[sd] could not list /shoe; image saving disabled");
    return false;
  }

  File entry = directory.openNextFile();
  while (entry) {
    if (!entry.isDirectory()) {
      const char *name = strrchr(entry.name(), '/');
      name = name == nullptr ? entry.name() : name + 1;
      unsigned long number = 0;
      int consumed = 0;
      if (sscanf(name, "step_%lu.jpg%n", &number, &consumed) == 1 &&
          consumed > 0 && name[consumed] == '\0' && number > highestNumber) {
        highestNumber = number;
      }
    }
    entry.close();
    entry = directory.openNextFile();
  }
  directory.close();
  if (highestNumber == UINT32_MAX) {
    Serial.println("[sd] image numbers exhausted; image saving disabled");
    return false;
  }
  nextImageNumber = highestNumber + 1;
  return true;
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

// 次の転送が失敗したときに、前回分を「今回分」として削除しないよう対象を解除する。
// JPEG自体は削除しない。対象の記録を解除できない場合は転送も開始しない。
bool clearDeletionBatch() {
  return !transferPreferences.isKey("delete_batch") ||
         transferPreferences.remove("delete_batch");
}

// 完了番号だけでは対象範囲の未転送画像も消しかねないため、実際に送った番号を記録する。
// NVSへ残すことで、転送後に再起動しても同じバッチをdで削除できる。
bool rememberDeletionBatch(const TransferImage *images, uint16_t count) {
  if (count == 0 || count > MAX_TRANSFER_IMAGES) {
    return false;
  }
  uint32_t numbers[MAX_TRANSFER_IMAGES];
  for (uint16_t i = 0; i < count; ++i) {
    numbers[i] = images[i].imageNumber;
  }
  const size_t bytes = count * sizeof(numbers[0]);
  return transferPreferences.putBytes("delete_batch", numbers, bytes) == bytes;
}

// d対応前の転送にはdelete_batchがないため、従来からNVSに保存している
// last_imageを上限として、index.csvに載っている転送済みJPEGを削除する。
// CSVにない画像は転送済みと確認できないので、番号が古くても削除しない。
void deletePreviouslyTransferredImages(uint32_t lastTransferred) {
  if (lastTransferred == 0) {
    Serial.println(
        "[delete] no transfer history found; upload with 'u' before deleting");
    return;
  }

  File file = SD.open("/shoe/index.csv", FILE_READ);
  if (!file) {
    Serial.println("[delete] /shoe/index.csv not found; nothing deleted");
    return;
  }

  uint16_t deleted = 0;
  uint16_t missing = 0;
  uint16_t failed = 0;
  char line[192];
  while (file.available()) {
    const size_t length = file.readBytesUntil('\n', line, sizeof(line) - 1);
    line[length] = '\0';
    if (length == 0 || strncmp(line, "event_id,", 9) == 0) {
      continue;
    }

    unsigned long eventId = 0;
    unsigned long captureMs = 0;
    unsigned long landingMs = 0;
    unsigned int fsrPeak = 0;
    unsigned long sharpness = 0;
    float similarityMad = 0.0f;
    char sdPath[48] = {};
    const int fields = sscanf(line, "%lu,%47[^,],%lu,%lu,%u,%lu,%f",
                              &eventId, sdPath, &captureMs, &landingMs,
                              &fsrPeak, &sharpness, &similarityMad);
    if (fields != 7) {
      continue;
    }

    const char *basename = strrchr(sdPath, '/');
    basename = basename == nullptr ? sdPath : basename + 1;
    unsigned long imageNumber = 0;
    int consumed = 0;
    if (sscanf(basename, "step_%lu.jpg%n", &imageNumber, &consumed) != 1 ||
        consumed <= 0 || basename[consumed] != '\0' || imageNumber == 0 ||
        imageNumber > lastTransferred) {
      continue;
    }

    if (!SD.exists(sdPath)) {
      ++missing;
    } else if (SD.remove(sdPath)) {
      ++deleted;
      Serial.printf("[delete] removed %s\n", sdPath);
    } else {
      ++failed;
      Serial.printf("[delete] could not remove %s; send 'd' to retry\n", sdPath);
    }
  }
  file.close();
  Serial.printf("[delete] legacy transfer cleanup through step_%05lu: "
                "deleted=%u already_missing=%u failed=%u; "
                "Supabase images and index.csv kept\n",
                static_cast<unsigned long>(lastTransferred), deleted, missing,
                failed);
}

// 全件転送成功後に記録したJPEGだけをSDから削除し、CSVは撮影履歴として残す。
// 撮影・転送と同じloop内で同期実行するため、ファイルの書き込みと競合しない。
// 途中で削除に失敗したら記録を残し、次のdでは既に消えたファイルを飛ばして再試行する。
void deleteLastTransferredBatch() {
  if (!sdReady || !transferPreferencesReady) {
    Serial.println("[delete] SD or transfer records unavailable; nothing deleted");
    return;
  }
  if (transferRequested) {
    Serial.println("[delete] upload pending; send 'd' after transfer completes");
    return;
  }
  if (!transferPreferences.isKey("delete_batch")) {
    Serial.println(
        "[delete] exact batch record not found; checking earlier transfers");
    deletePreviouslyTransferredImages(
        transferPreferences.getULong("last_image", 0));
    return;
  }

  uint32_t numbers[MAX_TRANSFER_IMAGES];
  const size_t bytes = transferPreferences.getBytesLength("delete_batch");
  if (bytes == 0 || bytes > sizeof(numbers) || bytes % sizeof(numbers[0]) != 0 ||
      transferPreferences.getBytes("delete_batch", numbers, bytes) != bytes) {
    Serial.println("[delete] invalid batch record; nothing deleted");
    return;
  }
  const uint16_t count = bytes / sizeof(numbers[0]);
  const uint32_t lastTransferred = transferPreferences.getULong("last_image", 0);
  for (uint16_t i = 0; i < count; ++i) {
    if (numbers[i] == 0 || numbers[i] > lastTransferred ||
        (i > 0 && numbers[i] <= numbers[i - 1])) {
      Serial.println("[delete] inconsistent batch record; nothing deleted");
      return;
    }
  }
  if (numbers[count - 1] != lastTransferred) {
    Serial.println("[delete] batch does not match latest transfer; nothing deleted");
    return;
  }

  uint16_t deleted = 0;
  uint16_t missing = 0;
  uint16_t failed = 0;
  for (uint16_t i = 0; i < count; ++i) {
    char path[40];
    snprintf(path, sizeof(path), "/shoe/step_%05lu.jpg",
             static_cast<unsigned long>(numbers[i]));
    if (!SD.exists(path)) {
      ++missing;
    } else if (SD.remove(path)) {
      ++deleted;
      Serial.printf("[delete] removed %s\n", path);
    } else {
      ++failed;
      Serial.printf("[delete] could not remove %s; send 'd' to retry\n", path);
    }
  }
  if (failed == 0 && !clearDeletionBatch()) {
    Serial.println("[delete] could not clear batch record; send 'd' to retry");
  }
  Serial.printf("[delete] deleted=%u already_missing=%u failed=%u; "
                "Supabase images and index.csv kept\n", deleted, missing, failed);
}

// シリアルから't'で着地、'u'でマット転送、'd'で転送完了分のSD画像削除を実行する。
// センサー入力と、撮影・Supabase側の問題をそれぞれ切り分けるための診断経路。
void pollSerialTrigger() {
  while (Serial.available() > 0) {
    const char command = static_cast<char>(Serial.read());
    if (command == 't' || command == 'T') {
      if (!hallStableActive) {
        startLandingEvent(true);
      }
    } else if (command == 'u' || command == 'U') {
      transferRequested = true;
      Serial.println("[transfer] manual transfer requested");
    } else if (command == 'd' || command == 'D') {
      deleteLastTransferredBatch();
    }
  }
}

// Supabaseへ公開してよい低権限キーが設定されているかだけを確認する。
// プレースホルダーのまま無効な接続を繰り返すと、ホールセンサー自体の検証ログが
// 読みにくくなるため、ネットワークへ出る前に設定漏れを明示する。
bool transferSettingsAreConfigured() {
  return strcmp(WIFI_SSID, "WIFI_SSID") != 0 &&
         strcmp(WIFI_PASSWORD, "WIFI_PASSWORD") != 0 &&
         strcmp(SUPABASE_URL, "SUPABASE_URL") != 0 &&
         strcmp(SUPABASE_KEY, "SUPABASE_KEY") != 0;
}

// SUPABASE_URL末尾のスラッシュ有無に影響されないAPI URLを組み立てる。
// objectPathにはこのスケッチが生成する英数字・ハイフン・スラッシュだけを渡すため、
// ここではURLエンコードを行わない。
String supabaseApiUrl(const String &apiPath) {
  String url(SUPABASE_URL);
  while (url.endsWith("/")) {
    url.remove(url.length() - 1);
  }
  url += apiPath;
  return url;
}

// Supabase REST/Storage APIに共通する認証ヘッダーを付ける。
// 新しいsb_publishableキーはapikeyだけで送り、JWT形式の旧anonキーに限って
// Authorizationにも設定する。secret/service_roleキーは端末へ保存してはいけない。
void addSupabaseHeaders(HTTPClient &http) {
  http.addHeader("apikey", SUPABASE_KEY);
  if (strncmp(SUPABASE_KEY, "eyJ", 3) == 0) {
    http.addHeader("Authorization", String("Bearer ") + SUPABASE_KEY);
  }
}

// HTTP失敗時に、ステータスとSupabaseが返した短い診断だけを表示する。
// 認証キーやWi-Fiパスワードはログへ出さず、レスポンスもシリアルを埋めない長さに
// 制限する。
void printHttpFailure(const char *operation, int statusCode,
                      HTTPClient &http) {
  String response = http.getString();
  if (response.length() > 300) {
    response = response.substring(0, 300);
    response += "...";
  }
  Serial.printf("[transfer] %s failed: HTTP %d", operation, statusCode);
  if (response.length() > 0) {
    Serial.printf("; %s", response.c_str());
  }
  Serial.println();
}

// 家のWi-Fiへ一定時間だけ接続を試し、失敗時は撮影へ戻れるようfalseを返す。
// 転送時以外は無線を切ってカメラ実験への消費電力・ノイズの影響を抑える。
bool connectTransferWifi() {
  if (WiFi.status() == WL_CONNECTED) {
    return true;
  }

  Serial.printf("[wifi] connecting to %s", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  const uint32_t startedAt = millis();
  while (WiFi.status() != WL_CONNECTED &&
         millis() - startedAt < WIFI_CONNECT_TIMEOUT_MS) {
    delay(250);
    Serial.print('.');
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[wifi] connection timed out; files remain on microSD");
    WiFi.disconnect(true, false);
    return false;
  }

  Serial.printf("[wifi] connected, IP=%s, RSSI=%d dBm\n",
                WiFi.localIP().toString().c_str(), WiFi.RSSI());
  return true;
}

// 現在の試作では証明書を端末へまだ組み込んでいないため、TLS暗号化だけを使って
// Supabaseへ接続する。実展示前にはsetInsecure()をSupabase証明書チェーンの検証へ
// 置き換える必要がある。この関数に集約し、その変更箇所を一つにしている。
void configurePrototypeTls(WiFiClientSecure &client) {
  client.setInsecure();
}

// 小さなmanifest文字列をSupabase Storageへ先にアップロードする。
// JPEG到着前にマット側が画像一覧と特徴量を読めることが、段階描画を始める条件に
// なる。保存先はセッションごとに一意なので上書きは許可しない。
bool uploadTextObject(const String &objectPath, const String &content,
                      const char *contentType) {
  const String url = supabaseApiUrl(
      String("/storage/v1/object/") + STORAGE_BUCKET + "/" + objectPath);
  WiFiClientSecure client;
  configurePrototypeTls(client);
  HTTPClient http;
  if (!http.begin(client, url)) {
    Serial.println("[transfer] could not start manifest HTTPS request");
    return false;
  }

  http.setTimeout(HTTP_TIMEOUT_MS);
  addSupabaseHeaders(http);
  http.addHeader("Content-Type", contentType);
  http.addHeader("x-upsert", "false");
  const int statusCode = http.POST(
      reinterpret_cast<uint8_t *>(const_cast<char *>(content.c_str())),
      content.length());
  const bool ok = statusCode >= 200 && statusCode < 300;
  if (!ok) {
    printHttpFailure("manifest upload", statusCode, http);
  }
  http.end();
  return ok;
}

// microSD上のJPEGを全体メモリへ読み込まず、HTTPリクエストへ直接流す。
// 1枚ずつ完了を確定してから次へ進むことで、失敗位置をuploaded_countとして
// マットへ通知できる。
bool uploadFileObject(const String &objectPath, const char *sdPath,
                      const char *contentType) {
  File file = SD.open(sdPath, FILE_READ);
  if (!file) {
    Serial.printf("[transfer] could not open %s\n", sdPath);
    return false;
  }

  const String url = supabaseApiUrl(
      String("/storage/v1/object/") + STORAGE_BUCKET + "/" + objectPath);
  WiFiClientSecure client;
  configurePrototypeTls(client);
  HTTPClient http;
  if (!http.begin(client, url)) {
    file.close();
    Serial.println("[transfer] could not start image HTTPS request");
    return false;
  }

  http.setTimeout(HTTP_TIMEOUT_MS);
  addSupabaseHeaders(http);
  http.addHeader("Content-Type", contentType);
  http.addHeader("x-upsert", "false");
  const size_t fileSize = file.size();
  const int statusCode = http.sendRequest("POST", &file, fileSize);
  file.close();

  const bool ok = statusCode >= 200 && statusCode < 300;
  if (!ok) {
    printHttpFailure(sdPath, statusCode, http);
  }
  http.end();
  return ok;
}

// manifestを読める状態にした後、Realtime通知元となるセッション行を作成する。
// このINSERTがマット側の「生成開始」トリガーになるため、JPEGのアップロードより
// 必ず前に行う。
bool insertTransferSession(const String &sessionId,
                           const String &manifestPath, uint16_t imageCount,
                           uint32_t lastImageNumber) {
  const String url = supabaseApiUrl("/rest/v1/transfer_sessions");
  String body;
  body.reserve(320);
  body += "{\"id\":\"" + sessionId + "\",\"manifest_path\":\"";
  body += manifestPath;
  body += "\",\"image_count\":" + String(imageCount);
  body += ",\"uploaded_count\":0,\"last_image_number\":";
  body += String(lastImageNumber);
  body += ",\"status\":\"uploading\"}";

  WiFiClientSecure client;
  configurePrototypeTls(client);
  HTTPClient http;
  if (!http.begin(client, url)) {
    return false;
  }
  http.setTimeout(HTTP_TIMEOUT_MS);
  addSupabaseHeaders(http);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Prefer", "return=minimal");
  const int statusCode = http.POST(
      reinterpret_cast<uint8_t *>(const_cast<char *>(body.c_str())),
      body.length());
  const bool ok = statusCode >= 200 && statusCode < 300;
  if (!ok) {
    printHttpFailure("session insert", statusCode, http);
  }
  http.end();
  return ok;
}

// 画像1枚の完了ごとにuploaded_countを更新し、最後はcompleteへ遷移させる。
// ブラウザはこのUPDATEを購読し、利用可能になった枚数まで順番に取得する。
bool updateTransferSession(const String &sessionId, uint16_t uploadedCount,
                           const char *status) {
  String url = supabaseApiUrl("/rest/v1/transfer_sessions?id=eq.");
  url += sessionId;
  String body = "{\"uploaded_count\":" + String(uploadedCount) +
                ",\"status\":\"" + status + "\"}";

  WiFiClientSecure client;
  configurePrototypeTls(client);
  HTTPClient http;
  if (!http.begin(client, url)) {
    return false;
  }
  http.setTimeout(HTTP_TIMEOUT_MS);
  addSupabaseHeaders(http);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Prefer", "return=minimal");
  const int statusCode = http.sendRequest(
      "PATCH", reinterpret_cast<uint8_t *>(const_cast<char *>(body.c_str())),
      body.length());
  const bool ok = statusCode >= 200 && statusCode < 300;
  if (!ok) {
    printHttpFailure("session update", statusCode, http);
  }
  http.end();
  return ok;
}

// /shoe/index.csvから未転送の画像を撮影順に読み込む。
// JPEG保存成功後にCSV追記が失敗した画像はここに現れないため、警告ログで気づける
// ようにする。1回の上限を超えた分は次の設置時に送る。
uint16_t loadPendingTransferImages(uint32_t lastTransferredImage,
                                   TransferImage *images, bool &hasMore) {
  hasMore = false;
  File file = SD.open("/shoe/index.csv", FILE_READ);
  if (!file) {
    Serial.println("[transfer] /shoe/index.csv was not found");
    return 0;
  }

  uint16_t count = 0;
  char line[192];
  while (file.available()) {
    const size_t length = file.readBytesUntil('\n', line, sizeof(line) - 1);
    line[length] = '\0';
    if (length == 0 || strncmp(line, "event_id,", 9) == 0) {
      continue;
    }

    unsigned long eventId = 0;
    unsigned long captureMs = 0;
    unsigned long landingMs = 0;
    unsigned int fsrPeak = 0;
    unsigned long sharpness = 0;
    float similarityMad = 0.0f;
    char sdPath[48] = {};
    const int fields = sscanf(line, "%lu,%47[^,],%lu,%lu,%u,%lu,%f",
                              &eventId, sdPath, &captureMs, &landingMs,
                              &fsrPeak, &sharpness, &similarityMad);
    if (fields != 7) {
      Serial.println("[transfer] warning: skipped malformed index.csv row");
      continue;
    }

    const char *basename = strrchr(sdPath, '/');
    basename = basename == nullptr ? sdPath : basename + 1;
    unsigned long imageNumber = 0;
    if (sscanf(basename, "step_%lu.jpg", &imageNumber) != 1 ||
        imageNumber <= lastTransferredImage) {
      continue;
    }
    if (!SD.exists(sdPath)) {
      Serial.printf("[transfer] warning: %s is listed but missing\n", sdPath);
      continue;
    }
    if (count >= MAX_TRANSFER_IMAGES) {
      hasMore = true;
      continue;
    }

    TransferImage &image = images[count++];
    image.imageNumber = imageNumber;
    image.eventId = eventId;
    image.captureMs = captureMs;
    image.landingMs = landingMs;
    image.fsrPeak = static_cast<uint16_t>(fsrPeak);
    image.sharpness = sharpness;
    image.similarityMad = similarityMad;
    strlcpy(image.filename, basename, sizeof(image.filename));
    strlcpy(image.sdPath, sdPath, sizeof(image.sdPath));
  }
  file.close();
  return count;
}

// RTCがなくても衝突しにくいセッションIDを、端末固有MAC・乱数・起動時間から作る。
// IDはStorageのパスとDB主キーの両方に使うため、URLで安全な文字だけに限定する。
String createTransferSessionId() {
  const uint64_t chipId = ESP.getEfuseMac();
  char id[40];
  snprintf(id, sizeof(id), "%04lx%08lx-%08lx-%08lx",
           static_cast<unsigned long>((chipId >> 32) & 0xFFFF),
           static_cast<unsigned long>(chipId & 0xFFFFFFFF),
           static_cast<unsigned long>(esp_random()),
           static_cast<unsigned long>(millis()));
  return String(id);
}

// index.csvの対象行を、マットが最初に受け取る小さなJSONへ変換する。
// sharpness等から仮の模様を描けるため、JPEG本体の転送を待たずに画面を変えられる。
String buildTransferManifest(const String &sessionId,
                             const TransferImage *images, uint16_t count,
                             bool hasMore) {
  String manifest;
  manifest.reserve(320 + static_cast<size_t>(count) * 220);
  manifest += "{\"version\":1,\"session_id\":\"" + sessionId;
  manifest += "\",\"image_count\":" + String(count);
  manifest += ",\"has_more\":";
  manifest += hasMore ? "true" : "false";
  manifest += ",\"images\":[";

  for (uint16_t i = 0; i < count; ++i) {
    const TransferImage &image = images[i];
    if (i > 0) {
      manifest += ',';
    }
    manifest += "{\"order\":" + String(i);
    manifest += ",\"filename\":\"" + String(image.filename) + "\"";
    manifest += ",\"storage_path\":\"sessions/" + sessionId +
                "/images/" + image.filename + "\"";
    manifest += ",\"event_id\":" + String(image.eventId);
    manifest += ",\"capture_ms\":" + String(image.captureMs);
    manifest += ",\"landing_ms\":" + String(image.landingMs);
    manifest += ",\"fsr_peak\":" + String(image.fsrPeak);
    manifest += ",\"sharpness\":" + String(image.sharpness);
    manifest += ",\"similarity_mad\":" +
                String(image.similarityMad, 2) + "}";
  }
  manifest += "]}";
  return manifest;
}

// 1バッチについて、manifest公開→セッション通知→JPEG順次転送を実行する。
// 途中で失敗した場合はNVS上の完了番号を進めないため、靴を置き直す
// と同じ画像を別セッションで再送できる。
TransferResult uploadTransferBatch(const TransferImage *images,
                                   uint16_t count, bool hasMore) {
  if (!clearDeletionBatch()) {
    Serial.println("[transfer] could not clear previous deletion record; cancelled");
    return TransferResult::Failed;
  }
  if (!connectTransferWifi()) {
    return TransferResult::Failed;
  }

  const String sessionId = createTransferSessionId();
  const String manifestPath =
      "sessions/" + sessionId + "/manifest.json";
  const String manifest =
      buildTransferManifest(sessionId, images, count, hasMore);
  const uint32_t lastImageNumber = images[count - 1].imageNumber;

  Serial.printf("[transfer %s] uploading metadata for %u image(s)\n",
                sessionId.c_str(), count);
  if (!uploadTextObject(manifestPath, manifest, "application/json") ||
      !insertTransferSession(sessionId, manifestPath, count,
                             lastImageNumber)) {
    WiFi.disconnect(true, false);
    return TransferResult::Failed;
  }

  for (uint16_t i = 0; i < count; ++i) {
    const TransferImage &image = images[i];
    const String objectPath = "sessions/" + sessionId + "/images/" +
                              String(image.filename);
    Serial.printf("[transfer %s] image %u/%u: %s\n", sessionId.c_str(),
                  i + 1, count, image.filename);
    if (!uploadFileObject(objectPath, image.sdPath, "image/jpeg")) {
      updateTransferSession(sessionId, i, "failed");
      WiFi.disconnect(true, false);
      return TransferResult::Failed;
    }
    if (!updateTransferSession(sessionId, i + 1, "uploading")) {
      WiFi.disconnect(true, false);
      return TransferResult::Failed;
    }
  }

  if (!updateTransferSession(sessionId, count, "complete")) {
    WiFi.disconnect(true, false);
    return TransferResult::Failed;
  }

  if (transferPreferences.putULong("last_image", lastImageNumber) !=
      sizeof(lastImageNumber)) {
    Serial.println("[transfer] could not persist completion; SD images kept");
    WiFi.disconnect(true, false);
    return TransferResult::Failed;
  }
  if (rememberDeletionBatch(images, count)) {
    Serial.printf("[delete] %u uploaded image(s) recorded; send 'd' to delete "
                  "them from microSD\n", count);
  } else {
    Serial.println("[delete] could not record uploaded images; SD images kept, "
                   "'d' unavailable for this batch");
  }
  WiFi.disconnect(true, false);
  Serial.printf("[transfer %s] complete; last image=%lu%s\n",
                sessionId.c_str(),
                static_cast<unsigned long>(lastImageNumber),
                hasMore ? "; another batch remains" : "");
  return TransferResult::Success;
}

// 未転送一覧用の領域をPSRAMに一時確保し、完了後は必ず解放する。
// 写真そのものはFileから直接送るため、この領域の大きさは枚数に比例する
// メタデータ分だけである。
TransferResult performShoeTransfer() {
  if (!transferPreferencesReady) {
    Serial.println("[transfer] NVS unavailable; transfer cancelled");
    return TransferResult::Failed;
  }
  if (!sdReady) {
    Serial.println("[transfer] SD is unavailable; transfer cancelled");
    return TransferResult::Failed;
  }
  if (!transferSettingsAreConfigured()) {
    Serial.println(
        "[transfer] set WIFI_SSID, WIFI_PASSWORD, SUPABASE_URL and "
        "SUPABASE_KEY first");
    return TransferResult::Failed;
  }

  TransferImage *images = static_cast<TransferImage *>(heap_caps_calloc(
      MAX_TRANSFER_IMAGES, sizeof(TransferImage),
      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (images == nullptr) {
    Serial.println("[transfer] could not allocate transfer metadata in PSRAM");
    return TransferResult::Failed;
  }

  const uint32_t lastTransferredImage =
      transferPreferences.getULong("last_image", 0);
  bool hasMore = false;
  const uint16_t count =
      loadPendingTransferImages(lastTransferredImage, images, hasMore);
  TransferResult result = TransferResult::NothingToUpload;
  if (count == 0) {
    Serial.printf("[transfer] no image newer than step_%05lu.jpg\n",
                  static_cast<unsigned long>(lastTransferredImage));
  } else {
    result = uploadTransferBatch(images, count, hasMore);
  }

  heap_caps_free(images);
  return result;
}

// デジタルホール出力をデバウンスし、磁石へ近づいた1回につき転送を1回だけ予約する。
// 失敗時は靴を置いたままでも一定間隔で再試行し、成功後は磁石から離れるまで
// 再送しない。マット設置の立ち上がりでは、FSR由来の未完了着地を破棄する。
void updateHallSensor() {
  const uint32_t now = millis();
  const bool active = digitalRead(HALL_PIN) == HALL_ACTIVE_LEVEL;
  if (active != hallRawActive) {
    hallRawActive = active;
    hallRawChangedAtMs = now;
  }

  if (hallRawActive != hallStableActive &&
      now - hallRawChangedAtMs >= HALL_DEBOUNCE_MS) {
    hallStableActive = hallRawActive;
    if (hallStableActive) {
      landingCaptureActive = false;
      postFramesRemaining = 0;
      hallHandledForPlacement = false;
      nextTransferRetryAtMs = now;
      Serial.println("[hall] mat magnet detected; capture paused");
    } else {
      hallHandledForPlacement = false;
      transferRequested = false;
      fsrFilterInitialized = false;
      fsrContact = false;
      fsrAboveSinceMs = 0;
      fsrBelowSinceMs = 0;
      nextFrameAtMs = now;
      Serial.println("[hall] shoe removed; capture resumed");
    }
  }

  if (hallStableActive && !hallHandledForPlacement && !transferRequested &&
      static_cast<int32_t>(now - nextTransferRetryAtMs) >= 0) {
    transferRequested = true;
  }
}

// 予約された転送を同期的に完了させ、結果に応じて同じ設置中の再試行を制御する。
// 転送中はカメラとSD保存を止めるので、同じSPI/メモリ資源を同時に操作しない。
void handleTransferRequest() {
  if (!transferRequested || landingCaptureActive) {
    return;
  }

  transferRequested = false;
  const TransferResult result = performShoeTransfer();
  if (result == TransferResult::Success ||
      result == TransferResult::NothingToUpload) {
    hallHandledForPlacement = hallStableActive;
  } else if (hallStableActive) {
    nextTransferRetryAtMs = millis() + TRANSFER_RETRY_INTERVAL_MS;
    Serial.printf("[transfer] retry in %lu seconds while shoe remains\n",
                  static_cast<unsigned long>(
                      TRANSFER_RETRY_INTERVAL_MS / 1000));
  }
  nextFrameAtMs = millis() + FRAME_INTERVAL_MS;
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
  pinMode(HALL_PIN, INPUT_PULLUP);
  analogReadResolution(12);

  transferPreferencesReady = transferPreferences.begin("shoe-xfer", false);
  if (!transferPreferencesReady) {
    Serial.println("[warning] NVS unavailable; transfer and deletion disabled");
  }

  ringReady = allocateRingBuffer();
  cameraReady = ringReady && initializeCamera();
  sdReady = mountSdCard();
  if (sdReady) {
    sdReady = findNextImageNumber();
  }

  nextFrameAtMs = millis();
  Serial.printf(
      "[ready] camera=%d ring=%d sd=%d, press=%u release=%u, hall=GPIO%d. "
      "Send 't' for landing, 'u' for upload, 'd' to delete the uploaded batch.\n",
      cameraReady, ringReady, sdReady, FSR_PRESS_THRESHOLD,
      FSR_RELEASE_THRESHOLD, HALL_PIN);
}

// FSR監視を優先しながら、millis()ベースでカメラを目標fpsにスケジュールする。
// JPEG変換やSD保存で遅れた後にフレームを連続取得すると「直近」の意味が崩れるため、
// 遅延分を取り戻すバースト撮影は行わず、現在時刻から次の1枚を予約し直す。
void loop() {
  updateHallSensor();
  if (!hallStableActive) {
    updateFsr();
  }
  // マット上で早期returnする前に読むことで、転送後もdやuを受け付ける。
  pollSerialTrigger();
  handleTransferRequest();

  // マット上では撮影を止める。転送完了後も同じ床を着地画像として増やさないため。
  if (hallStableActive) {
    delay(10);
    return;
  }

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
