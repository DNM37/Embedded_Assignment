#include "esp_camera.h"
#include <WiFi.h>
#include "esp_timer.h"
#include "img_converters.h"

// Select camera model - AI_THINKER is the most common ESP32-CAM module
#define CAMERA_MODEL_AI_THINKER

// Camera pin definitions for AI-THINKER ESP32-CAM
#if defined(CAMERA_MODEL_AI_THINKER)
  #define PWDN_GPIO_NUM     32
  #define RESET_GPIO_NUM    -1
  #define XCLK_GPIO_NUM      0
  #define SIOD_GPIO_NUM     26
  #define SIOC_GPIO_NUM     27
  
  #define Y9_GPIO_NUM       35
  #define Y8_GPIO_NUM       34
  #define Y7_GPIO_NUM       39
  #define Y6_GPIO_NUM       36
  #define Y5_GPIO_NUM       21
  #define Y4_GPIO_NUM       19
  #define Y3_GPIO_NUM       18
  #define Y2_GPIO_NUM        5
  #define VSYNC_GPIO_NUM    25
  #define HREF_GPIO_NUM     23
  #define PCLK_GPIO_NUM     22
#else
  #error "Camera model not selected"
#endif

// Replace with your network credentials
const char* ssid = "Georges_iPhone";
const char* password = "yessirski";

// Web server running on port 80
WiFiServer server(80);

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  camera_config_t config;
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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_VGA;
  config.pixel_format = PIXFORMAT_JPEG;
  config.jpeg_quality = 20;
  config.fb_count = 2;

  // Camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    delay(1000);
    ESP.restart();
  }

  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);
  
  int attempt = 0;
  while (WiFi.status() != WL_CONNECTED && attempt < 60) { // 10s timeout
    delay(500);
    Serial.print(".");
    attempt++;
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nFailed to connect to WiFi!");
    return;
  }

  Serial.println("\nWiFi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Start the web server
  startCameraServer();
}

void loop() {
  WiFiClient client = server.available();
  if (client) {
    Serial.println("New Client Connected.");
    handleClient(client);
  }
}

// Function to start the web server
void startCameraServer() {
  server.begin();
  Serial.println("Web server started successfully!");
  Serial.print("Visit: http://");
  Serial.print(WiFi.localIP());
  Serial.println("/stream to view the camera feed.");
}
void handleClient(WiFiClient client) {
  String request = "";
  String currentLine = "";

  while (client.connected() && client.available()) {
    char c = client.read();
    request += c;
    if (c == '\n') break;
  }

  // Handle /offset?x=...&y=...
  if (request.indexOf("GET /offset?") >= 0) {
    int xIndex = request.indexOf("x=");
    int yIndex = request.indexOf("y=");
    if (xIndex > 0 && yIndex > 0) {
      int xEnd = request.indexOf('&', xIndex);
      String xVal = request.substring(xIndex + 2, xEnd);
      String yVal = request.substring(yIndex + 2, request.indexOf(' ', yIndex));
      Serial.printf("Offset received -> X: %s, Y: %s\n", xVal.c_str(), yVal.c_str());
    }

    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/plain");
    client.println("Connection: close");
    client.println();
    client.println("Offset received.");
    client.stop();
    return;
  }

  // MJPEG Stream handler
  if (request.indexOf("GET /stream") >= 0) {
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
    client.println();

    while (client.connected()) {
      camera_fb_t *fb = esp_camera_fb_get();
      if (!fb) {
        Serial.println("Camera capture failed");
        break;
      }

      client.println("--frame");
      client.println("Content-Type: image/jpeg");
      client.print("Content-Length: ");
      client.println(fb->len);
      client.println();

      client.write(fb->buf, fb->len);
      client.println();

      esp_camera_fb_return(fb);
      delay(50);
    }
  } else {
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println();
    client.println("<html><body><h2>ESP32-CAM</h2>");
    client.println("<p><a href=\"/stream\">Click here to view stream</a></p>");
    client.println("</body></html>");
  }

  client.stop();
  Serial.println("Client Disconnected.");
}

