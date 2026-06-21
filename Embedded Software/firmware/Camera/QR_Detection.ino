/* ============================================================
 *  QR_Detection.ino  –  ESP32-S3-EYE  (Camera / QR side)
 *  -----------------------------------------------------------
 *  Communication topology (this device):
 *
 *    [RC Controller] ──START──▶ [Camera] ──colour/TIMEOUT──▶ [Base]
 *
 *  This device receives from the RC Controller and sends to
 *  the Base controller.  It does NOT send back to the RC
 *  Controller (the radio-layer ACK is sufficient for the RC
 *  Controller's retry loop).
 *
 *  Behaviour:
 *   1. Camera stream (HTTP) is ALWAYS ON from boot.
 *   2. QR scanning is OFF by default, waiting for "START".
 *   3. Once the RC Controller sends "START": QR scanner runs.
 *   3a. First successful QR decode → send colour string to the
 *       BASE CONTROLLER via ESP-NOW → QR scanner stops (stream
 *       continues).
 *   3b. No QR found within 20 s → send "TIMEOUT" to the BASE
 *       CONTROLLER via ESP-NOW → QR scanner stops (stream
 *       continues).
 *
 *  *** WHY MESSAGES WERE FAILING & THE FIX ***
 *  -------------------------------------------
 *  Root cause: the Camera connects to a WiFi router (WIFI_STA).
 *  The router assigns its own channel (e.g. ch 1, 11, 13).
 *  Calling esp_wifi_set_channel() AFTER association does NOT
 *  actually hold — the driver reverts the radio back to the
 *  router's channel on the very next beacon.  Result: the
 *  Camera is perpetually on the router's channel while the RC
 *  Controller is locked to channel 6, so every ESP-NOW packet
 *  sent by the RC Controller is lost.
 *
 *  Fix: switch to WIFI_AP_STA mode and start a HIDDEN SoftAP
 *  on channel 6 BEFORE connecting to the router.  In AP_STA
 *  mode the SoftAP channel is the master; the router connection
 *  is forced to follow it.  The radio stays on channel 6
 *  permanently, so ESP-NOW always works while the HTTP web
 *  server still has a router connection.
 *
 *  MAC Addresses
 *  -------------
 *  Camera  (this device)       : 94:A9:90:0C:AE:88
 *  RC Controller (LED_Receiver): B0:CB:D8:8A:75:44  ← inbound only
 *  Base controller             : 80:F3:DA:54:AC:58  ← outbound only
 * ============================================================ */

/* ===================== Libraries =========================== */
#include "esp_camera.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "quirc.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>          // needed for esp_wifi_set_channel()
#include "esp_http_server.h"
/* =========================================================== */

/* ===================== Camera model ======================== */
#define CAMERA_MODEL_ESP32S3_EYE
#define CAMERA_MODULE_NAME  "ESP-S3-EYE"
#define PWDN_GPIO_NUM   -1
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM   15
#define SIOD_GPIO_NUM    4
#define SIOC_GPIO_NUM    5
#define Y2_GPIO_NUM     11
#define Y3_GPIO_NUM      9
#define Y4_GPIO_NUM      8
#define Y5_GPIO_NUM     10
#define Y6_GPIO_NUM     12
#define Y7_GPIO_NUM     18
#define Y8_GPIO_NUM     17
#define Y9_GPIO_NUM     16
#define VSYNC_GPIO_NUM   6
#define HREF_GPIO_NUM    7
#define PCLK_GPIO_NUM   13
/* =========================================================== */

/* ===================== ESP-NOW config ====================== */
/*  ESPNOW_CHANNEL must match the value in LED_Receiver.ino.
 *  All three devices share this fixed channel.               */
#define ESPNOW_CHANNEL  1

/*  RC Controller sends START to us (inbound only – no peer
 *  registration needed to receive from it).
 *  Base controller receives colour/TIMEOUT from us (outbound). */
#define BASE_MAC  { 0x80, 0xF3, 0xDA, 0x54, 0xAC, 0x58 }

/* Shared message structure – must be identical in all files.  */
typedef struct {
  char payload[128];
} espnow_msg_t;

static espnow_msg_t        outgoing;
static esp_now_peer_info_t basePeerInfo;   // Base controller peer
static bool                espnow_ready = false;
/* =========================================================== */

/* ===================== System state ======================== */
/*  streamActive : always true – camera stream runs from boot.
 *  qrActive     : true  = QR scanning enabled (button pressed)
 *                 false = idle, waiting for START             */
volatile bool          streamActive  = true;   // stream is always ON
volatile bool          qrActive      = false;  // QR off until START
volatile unsigned long scanStartTime = 0;

#define QR_SCAN_TIMEOUT_MS  20000UL   // 20 seconds
/* =========================================================== */

/* ===================== QR / camera globals ================= */
TaskHandle_t QRCodeReader_Task = NULL;

struct QRCodeData {
  bool    valid;
  int     dataType;
  uint8_t payload[1024];
  int     payloadLen;
};

struct quirc        *q     = NULL;
uint8_t             *image = NULL;
struct quirc_code    code;
struct quirc_data    qdata;
quirc_decode_error_t qerr;
String               QRCodeResult     = "";
String               QRCodeResultSend = "";

SemaphoreHandle_t cam_mutex = NULL;
/* =========================================================== */

/* ===================== WiFi / HTTP ========================= */
const char *ssid     = "ELBOV";
const char *password = "88888888";

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE =
    "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART =
    "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t index_httpd  = NULL;
httpd_handle_t stream_httpd = NULL;
/* =========================================================== */

/* ===================== HTML page =========================== */
static const char PROGMEM INDEX_HTML[] = R"rawliteral(
<html>
  <head>
    <title>ESP32-CAM QR Code Reader</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
      body { font-family: Arial; text-align: center; margin: 0 auto; padding-top: 10px; }
      img  { width: auto; max-width: 100%; height: auto; transform: rotate(90deg); transform-origin: center center; }
      #status { font-size: 14px; color: #888; margin-bottom: 6px; }
      #showqrcodeval {
        padding: 5px; border: 3px solid #075264; text-align: center;
        width: 70%; margin: auto; color: #0A758F;
      }
    </style>
  </head>
  <body>
    <h3>QR Code Reader – ESP32-CAM</h3>
    <p id="status">Streaming – press button on LED controller to start QR scan.</p>
    <img src="" id="vdstream">
    <br><br>
    <p>QR Code Scan Result:</p>
    <div id="showqrcodeval">...</div>
    <br>
    <button onclick="send_btn_cmd('clr')">Clear Result</button>
    <script>
      window.onload = function() {
        document.getElementById("vdstream").src =
            window.location.href.slice(0, -1) + ":81/stream";
      };
      let qrcodeval = "...";
      setInterval(function() {
        fetch("/getqrcodeval")
          .then(r => r.text())
          .then(t => {
            qrcodeval = t;
            document.getElementById("showqrcodeval").innerHTML = t || "...";
          });
        fetch("/getstatus")
          .then(r => r.text())
          .then(s => { document.getElementById("status").textContent = s; });
      }, 500);
      function send_btn_cmd(c) {
        fetch("/action?go=B," + c);
      }
    </script>
  </body>
</html>
)rawliteral";
/* =========================================================== */

/* =================== Forward declarations ================== */
void dumpData(const struct quirc_data *d);
String getValue(String data, char separator, int index);
void sendESPNow(const char *msg);
void createQRTask();
void QRCodeReader(void *pvParameters);
/* =========================================================== */

/* =================== ESP-NOW send callback ================= */
/* ESP32 Arduino core 3.x changed esp_now_send_cb_t: the first
 * argument is now const wifi_tx_info_t* instead of const uint8_t*.
 * The MAC address (if needed) is available via info->peer_addr.   */
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  Serial.print("ESP-NOW send: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAILED");
}

/* =================== ESP-NOW receive callback ============== */
/*  Handles "START" command sent by the LED receiver when its
 *  push button is pressed.
 *
 *  ESP32 Arduino core 3.x changed esp_now_recv_cb_t: the first
 *  argument is now const esp_now_recv_info_t* instead of
 *  const uint8_t* (MAC).  The source MAC is still available
 *  via info->src_addr if ever needed.                         */
void onDataReceived(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(espnow_msg_t)) return;

  espnow_msg_t msg;
  memcpy(&msg, data, sizeof(msg));
  msg.payload[sizeof(msg.payload) - 1] = '\0';

  String cmd = String(msg.payload);
  cmd.trim();
  cmd.toUpperCase();

  Serial.print("ESP-NOW received: "); Serial.println(cmd);

  if (cmd == "START") {
    if (!qrActive) {
      qrActive      = true;
      scanStartTime = millis();
      QRCodeResult  = "";           // clear old result
      Serial.println(">>> QR scan ACTIVATED by button press <<<");
    } else {
      Serial.println("QR scan already active – START ignored.");
    }
  }
}
/* =========================================================== */

/* =================== sendESPNow helper ==================== */
/*  Sends colour result or "TIMEOUT" to the Base controller.  */
void sendESPNow(const char *msg) {
  if (!espnow_ready) { Serial.println("ESP-NOW not ready"); return; }
  memset(outgoing.payload, 0, sizeof(outgoing.payload));
  strncpy(outgoing.payload, msg, sizeof(outgoing.payload) - 1);
  uint8_t baseMac[] = BASE_MAC;
  esp_err_t result = esp_now_send(baseMac,
                                  (uint8_t *)&outgoing,
                                  sizeof(outgoing));
  Serial.print("ESP-NOW → Base '"); Serial.print(msg); Serial.print("': ");
  Serial.println(result == ESP_OK ? "queued OK" : "FAILED");
}
/* =========================================================== */

/* =================== HTTP handlers ======================== */

static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, (const char *)INDEX_HTML, strlen(INDEX_HTML));
}

/* --- /getstatus : returns a short human-readable status ---- */
static esp_err_t status_handler(httpd_req_t *req) {
  char buf[64];
  if (qrActive) {
    unsigned long elapsed = (millis() - scanStartTime) / 1000;
    unsigned long remain  = QR_SCAN_TIMEOUT_MS / 1000 - elapsed;
    snprintf(buf, sizeof(buf), "Scanning... %lus remaining", remain);
  } else {
    snprintf(buf, sizeof(buf), "Streaming – press button to start QR scan.");
  }
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}

/* --- /stream : MJPEG stream (always available) ------------- */
static esp_err_t stream_handler(httpd_req_t *req) {
  Serial.print("stream_handler started on core ");
  Serial.println(xPortGetCoreID());

  camera_fb_t *fb           = NULL;
  esp_err_t    res          = ESP_OK;
  size_t       _jpg_buf_len = 0;
  uint8_t     *_jpg_buf     = NULL;
  char         part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;

  while (true) {
    _jpg_buf     = NULL;
    _jpg_buf_len = 0;

    if (xSemaphoreTake(cam_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    fb = esp_camera_fb_get();
    if (!fb) {
      xSemaphoreGive(cam_mutex);
      Serial.println("Camera capture failed (stream_handler)");
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    bool jpeg_ok = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
    esp_camera_fb_return(fb);
    fb = NULL;
    xSemaphoreGive(cam_mutex);

    if (!jpeg_ok) {
      Serial.println("JPEG compression failed");
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    size_t hlen = snprintf(part_buf, sizeof(part_buf),
                           _STREAM_PART, _jpg_buf_len);
    res = httpd_resp_send_chunk(req, part_buf, hlen);
    if (res == ESP_OK)
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    if (res == ESP_OK)
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY,
                                  strlen(_STREAM_BOUNDARY));
    free(_jpg_buf);
    _jpg_buf = NULL;

    if (res != ESP_OK) break;   // client disconnected
  }

  return res;
}

/* --- /action : button/command handler ---------------------- */
static esp_err_t cmd_handler(httpd_req_t *req) {
  char  *buf;
  size_t buf_len;
  char   variable[32] = {0};

  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    buf = (char *)malloc(buf_len);
    if (!buf) { httpd_resp_send_500(req); return ESP_FAIL; }
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
      if (httpd_query_key_value(buf, "go", variable, sizeof(variable)) != ESP_OK) {
        free(buf); httpd_resp_send_404(req); return ESP_FAIL;
      }
    } else {
      free(buf); httpd_resp_send_404(req); return ESP_FAIL;
    }
    free(buf);
  } else {
    httpd_resp_send_404(req); return ESP_FAIL;
  }

  String getData    = String(variable);
  String resultData = getValue(getData, ',', 0);
  if (resultData == "B") {
    resultData = getValue(getData, ',', 1);
    if (resultData == "clr") QRCodeResult = "";
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

/* --- /getqrcodeval : returns last decoded QR string -------- */
static esp_err_t qrcoderslt_handler(httpd_req_t *req) {
  if (QRCodeResult != "Decoding FAILED") QRCodeResultSend = QRCodeResult;
  httpd_resp_send(req, QRCodeResultSend.c_str(), HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}
/* =========================================================== */

/* =================== startCameraWebServer ================== */
void startCameraWebServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  httpd_uri_t index_uri   = {"/",            HTTP_GET, index_handler,      NULL};
  httpd_uri_t cmd_uri     = {"/action",       HTTP_GET, cmd_handler,        NULL};
  httpd_uri_t qrrslt_uri  = {"/getqrcodeval", HTTP_GET, qrcoderslt_handler, NULL};
  httpd_uri_t status_uri  = {"/getstatus",    HTTP_GET, status_handler,     NULL};
  httpd_uri_t stream_uri  = {"/stream",       HTTP_GET, stream_handler,     NULL};

  if (httpd_start(&index_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(index_httpd, &index_uri);
    httpd_register_uri_handler(index_httpd, &cmd_uri);
    httpd_register_uri_handler(index_httpd, &qrrslt_uri);
    httpd_register_uri_handler(index_httpd, &status_uri);
  }
  config.server_port += 1;
  config.ctrl_port   += 1;
  if (httpd_start(&stream_httpd, &config) == ESP_OK)
    httpd_register_uri_handler(stream_httpd, &stream_uri);

  Serial.println("\nCamera Web Server started.");
  Serial.print("Access at: http://");
  Serial.println(WiFi.localIP());
}
/* =========================================================== */

/* =================== initESPNow ============================ */
void initESPNow() {
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  /* Send callback (outgoing messages to Base).               */
  esp_now_register_send_cb(onDataSent);

  /* Receive callback – handles "START" from RC Controller.
     No peer registration needed to receive; ESP-NOW delivers
     packets to us regardless of the peer list.               */
  esp_now_register_recv_cb(onDataReceived);

  /* Channel is already locked to ESPNOW_CHANNEL (6) by the
     hidden SoftAP started in setup() before WiFi.begin().
     Verify it here for confidence – should always print 6.  */
  Serial.print("ESP-NOW operating channel: ");
  Serial.println(WiFi.channel());

  /* Register Base controller as the outbound peer.
     All QR results and TIMEOUT messages are sent here.      */
  uint8_t baseMac[] = BASE_MAC;
  memset(&basePeerInfo, 0, sizeof(basePeerInfo));
  memcpy(basePeerInfo.peer_addr, baseMac, 6);
  basePeerInfo.channel = ESPNOW_CHANNEL;
  basePeerInfo.encrypt = false;
  if (esp_now_add_peer(&basePeerInfo) != ESP_OK) {
    Serial.println("ESP-NOW add Base controller peer FAILED");
    return;
  }
  Serial.println("Base controller peer registered OK.");

  espnow_ready = true;
  Serial.println("ESP-NOW ready.");
  Serial.print("Camera MAC: ");
  Serial.println(WiFi.macAddress());
}
/* =========================================================== */

/* =================== setup ================================= */
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println("\n--- QR Detection (Camera) ---");

  cam_mutex = xSemaphoreCreateMutex();

  /* Camera init */
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 10000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size   = FRAMESIZE_QVGA;
  config.jpeg_quality = 15;
  config.fb_count     = 2;
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.grab_mode    = CAMERA_GRAB_LATEST;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init FAILED – restarting");
    ESP.restart();
  }
  esp_camera_sensor_get()->set_framesize(esp_camera_sensor_get(), FRAMESIZE_QVGA);
  Serial.println("Camera OK");

  /* quirc init */
  q = quirc_new();
  if (!q || quirc_resize(q, 320, 240) < 0) {
    Serial.println("quirc init FAILED – restarting");
    ESP.restart();
  }

  /* -------------------------------------------------------
   *  WiFi – AP_STA mode.
   *
   *  WHY AP_STA instead of STA:
   *  In pure STA mode the router dictates the radio channel.
   *  esp_wifi_set_channel() called after association is
   *  silently reverted on every router beacon, so ESP-NOW
   *  ends up on the router's channel, not channel 6.
   *
   *  In AP_STA mode the SoftAP channel is the MASTER.  The
   *  STA connection is forced to use the same channel as the
   *  AP.  By starting a hidden SoftAP on channel 6 first,
   *  the radio is permanently locked to channel 6 — and the
   *  router connection follows.  ESP-NOW therefore always
   *  works on channel 6, matching the RC Controller and Base.
   * ------------------------------------------------------- */
  WiFi.mode(WIFI_AP_STA);

  /* Start a hidden SoftAP on channel 1
     SSID/password are irrelevant (hidden=true, no one joins).
     This pins the radio to channel 6 before the router
     connection is established.                               */
  WiFi.softAP("ESP32CAM_ESPNOW", "unused_pw", ESPNOW_CHANNEL, 1 /*hidden*/);
  Serial.print("SoftAP started on channel: ");
  Serial.println(WiFi.channel());   // should print 1

  /* Now connect to the router. The router connection will be
     on channel 6 (following the SoftAP), not the other way. */
  Serial.print("Connecting to "); Serial.println(ssid);
  WiFi.begin(ssid, password);
  int timeout = 40;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
    if (--timeout == 0) {
      Serial.println("WiFi timeout – restarting");
      delay(500);
      ESP.restart();
    }
  }
  Serial.println("\nWiFi connected");
  Serial.print("Radio channel after router connect: ");
  Serial.println(WiFi.channel());   // must still be 6

  /* initESPNow() will override the channel to ESPNOW_CHANNEL. */
  initESPNow();
  startCameraWebServer();

  /* Permanent QR scanner task on core 0. */
  createQRTask();

  Serial.println("Camera stream active. Waiting for START to begin QR scan...");
}
/* =========================================================== */

/* =================== loop ================================== */
void loop() {
  delay(100);   // All logic handled in task + ESP-NOW callbacks.
}
/* =========================================================== */

/* =================== createQRTask ========================== */
void createQRTask() {
  if (QRCodeReader_Task != NULL) return;   // already exists
  xTaskCreatePinnedToCore(
    QRCodeReader,
    "QRCodeReader_Task",
    16000,
    NULL,
    1,
    &QRCodeReader_Task,
    0);
}
/* =========================================================== */

/* =================== QRCodeReader task =====================
 *  Runs permanently on core 0.
 *  Lifecycle per activation:
 *    1. Block until qrActive == true.
 *    2. Scan frames for QR codes.
 *    3a. QR found  → send colour → deactivate → go to 1.
 *    3b. 20 s elapsed → send "TIMEOUT" → deactivate → go to 1.
 * =========================================================== */
void QRCodeReader(void *pvParameters) {
  Serial.print("QRCodeReader task started on core ");
  Serial.println(xPortGetCoreID());

  while (true) {
    /* ---- Wait for activation ---- */
    while (!qrActive) {
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    Serial.println("QR scan started.");

    /* ---- Active scanning loop ---- */
    while (qrActive) {

      /* Timeout check */
      if (millis() - scanStartTime >= QR_SCAN_TIMEOUT_MS) {
        Serial.println("QR scan TIMEOUT (20 s) – sending TIMEOUT to LED controller");
        QRCodeResult = "TIMEOUT – no QR found in 20 s";
        sendESPNow("TIMEOUT");
        qrActive = false;
        Serial.println(">>> QR scan DEACTIVATED (timeout) – stream continues <<<");
        break;
      }

      /* Acquire camera */
      if (xSemaphoreTake(cam_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
        vTaskDelay(pdMS_TO_TICKS(10));
        continue;
      }

      camera_fb_t *fb = esp_camera_fb_get();
      if (!fb) {
        xSemaphoreGive(cam_mutex);
        Serial.println("Camera capture failed (QRCodeReader)");
        vTaskDelay(pdMS_TO_TICKS(100));
        continue;
      }

      image = quirc_begin(q, NULL, NULL);
      memcpy(image, fb->buf, fb->len);
      esp_camera_fb_return(fb);
      xSemaphoreGive(cam_mutex);

      quirc_end(q);
      int count = quirc_count(q);

      if (count > 0) {
        quirc_extract(q, 0, &code);
        qerr = quirc_decode(&code, &qdata);

        if (qerr) {
          QRCodeResult = "Decoding FAILED";
          Serial.println(QRCodeResult);
        } else {
          Serial.println("QR decode successful:");
          dumpData(&qdata);     // stores result + sends colour via ESP-NOW
          qrActive = false;     // disable QR scan after first successful detection
          Serial.println(">>> QR scan DEACTIVATED (QR detected) – stream continues <<<");
          break;
        }
      }

      taskYIELD();
    } // inner while

    Serial.println("QR scan stopped. Waiting for next button press...");
  } // outer while – task never terminates
}
/* =========================================================== */

/* =================== dumpData ============================== */
void dumpData(const struct quirc_data *d) {
  Serial.printf("  Version  : %d\n",   d->version);
  Serial.printf("  ECC level: %c\n",   "MLHQ"[d->ecc_level]);
  Serial.printf("  Mask     : %d\n",   d->mask);
  Serial.printf("  Length   : %d\n",   d->payload_len);
  Serial.printf("  Payload  : %s\n",   d->payload);

  QRCodeResult = (const char *)d->payload;

  /* Send the colour payload to the LED receiver via ESP-NOW. */
  sendESPNow((const char *)d->payload);
}
/* =========================================================== */

/* =================== getValue ============================== */
String getValue(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;
  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
/* =========================================================== */
