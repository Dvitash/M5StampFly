/*
 * MIT License
 *
 * Copyright (c) 2024 Kouhei Ito
 * Copyright (c) 2024 M5Stack
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "rc.hpp"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WebServer.h>
#include "flight_control.hpp"
#include <buzzer.h>
#include "imu.hpp"
#include "serial_logger.hpp"

extern volatile float acc_x, acc_y, acc_z;
extern volatile float gyro_x, gyro_y, gyro_z;
extern volatile float current_x, current_y;

WebServer server(80);

// esp_now_peer_info_t slave;

volatile uint16_t Connect_flag = 0;

// Telemetry相手のMAC ADDRESS 4C:75:25:AD:B6:6C
// ATOM Lite (C): 4C:75:25:AE:27:FC
// 4C:75:25:AD:8B:20
// 4C:75:25:AF:4E:84
// 4C:75:25:AD:8B:20
// 4C:75:25:AD:8B:20 赤水玉テープ　ATOM lite
uint8_t TelemAddr[6] = {0};
// uint8_t TelemAddr[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
volatile uint8_t MyMacAddr[6];
volatile uint8_t peer_command[4] = {0xaa, 0x55, 0x16, 0x88};
volatile uint8_t Rc_err_flag     = 0;
esp_now_peer_info_t peerInfo;

// RC
volatile float Stick[16];
volatile uint8_t Recv_MAC[3];

void on_esp_now_sent(const uint8_t *mac_addr, esp_now_send_status_t status);

// 受信コールバック
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *recv_data, int data_len) {
    Connect_flag = 0;

    uint8_t *d_int;
    // int16_t d_short;
    float d_float;

    if (!TelemAddr[0] && !TelemAddr[1] && !TelemAddr[2] && !TelemAddr[3] && !TelemAddr[4] && !TelemAddr[5]) {
        memcpy(TelemAddr, mac_addr, 6);
        memcpy(peerInfo.peer_addr, TelemAddr, 6);
        peerInfo.channel = CHANNEL;
        peerInfo.encrypt = false;
        if (esp_now_add_peer(&peerInfo) != ESP_OK) {
            serial_logger_usb_println("Failed to add peer2");
            memset(TelemAddr, 0, 6);
        } else {
            esp_now_register_send_cb(on_esp_now_sent);
        }
    }

    Recv_MAC[0] = recv_data[0];
    Recv_MAC[1] = recv_data[1];
    Recv_MAC[2] = recv_data[2];

    if ((recv_data[0] == MyMacAddr[3]) && (recv_data[1] == MyMacAddr[4]) && (recv_data[2] == MyMacAddr[5])) {
        Rc_err_flag = 0;
    } else {
        Rc_err_flag = 1;
        return;
    }

    // checksum
    uint8_t check_sum = 0;
    for (uint8_t i = 0; i < 24; i++) check_sum = check_sum + recv_data[i];
    // if (check_sum!=recv_data[23])USBSerial.printf("checksum=%03d recv_sum=%03d\n\r", check_sum, recv_data[23]);
    if (check_sum != recv_data[24]) {
        Rc_err_flag = 1;
        return;
    }

    d_int         = (uint8_t *)&d_float;
    d_int[0]      = recv_data[3];
    d_int[1]      = recv_data[4];
    d_int[2]      = recv_data[5];
    d_int[3]      = recv_data[6];
    Stick[RUDDER] = d_float;

    d_int[0]        = recv_data[7];
    d_int[1]        = recv_data[8];
    d_int[2]        = recv_data[9];
    d_int[3]        = recv_data[10];
    Stick[THROTTLE] = d_float;

    d_int[0]       = recv_data[11];
    d_int[1]       = recv_data[12];
    d_int[2]       = recv_data[13];
    d_int[3]       = recv_data[14];
    Stick[AILERON] = d_float;

    d_int[0]        = recv_data[15];
    d_int[1]        = recv_data[16];
    d_int[2]        = recv_data[17];
    d_int[3]        = recv_data[18];
    Stick[ELEVATOR] = d_float;

    Stick[BUTTON_ARM]     = recv_data[19];  // auto_up_down_status
    Stick[BUTTON_FLIP]    = recv_data[20];
    Stick[CONTROLMODE]    = recv_data[21];  // Mode:rate or angle control
    Stick[ALTCONTROLMODE] = recv_data[22];  // 高度制御

    ahrs_reset_flag = recv_data[23];

    Stick[LOG] = 0.0;
    // if (check_sum!=recv_data[23])USBSerial.printf("checksum=%03d recv_sum=%03d\n\r", check_sum, recv_data[23]);

#if 0
  USBSerial.printf("%6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f  %6.3f\n\r", 
                                            Stick[THROTTLE],
                                            Stick[AILERON],
                                            Stick[ELEVATOR],
                                            Stick[RUDDER],
                                            Stick[BUTTON_ARM],
                                            Stick[BUTTON_FLIP],
                                            Stick[CONTROLMODE],
                                            Stick[ALTCONTROLMODE],
                                            Stick[LOG]);
#endif
}

// 送信コールバック
uint8_t esp_now_send_status;
void on_esp_now_sent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    esp_now_send_status = status;
}

static uint32_t g_activeFreq = 0;

static void handle_options() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Access-Control-Allow-Methods", "GET,POST,OPTIONS");
    server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
    server.send(204);
}

// start/stop buzz
void startBuzz() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    uint32_t freq = server.hasArg("freq") ? server.arg("freq").toInt() : 1000;
    if (freq < 50) freq = 50;
    if (freq > 10000) freq = 10000;
    tone(freq);
    g_activeFreq = freq;
    server.send(200, "text/plain", "OK");
}

void stopBuzz() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    if (server.hasArg("freq")) {
        uint32_t f = server.arg("freq").toInt();
        if (f == g_activeFreq) {
            g_activeFreq = 0;
            stopTone();
        }
    }
    server.send(200, "text/plain", "OK");
}

void do_hover() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    float alt = server.hasArg("altitude") ? server.arg("altitude").toFloat() : 0.5f;
    auto_takeoff_and_hover(alt);
    server.send(200, "text/plain", "OK");
}

void auto_land() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    request_mode_change(AUTO_LANDING_MODE);
    server.send(200, "text/plain", "OK");
}

void handle_movement() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    const String dir  = server.arg("direction");
    const float speed = server.hasArg("speed") ? server.arg("speed").toFloat() : 1.f;
    if (dir == "left")
        Stick[AILERON] = -speed;
    else if (dir == "right")
        Stick[AILERON] = speed;
    else if (dir == "forward")
        Stick[ELEVATOR] = speed;
    else if (dir == "backward")
        Stick[ELEVATOR] = -speed;
    else {
        server.send(400, "text/plain", "bad direction");
        return;
    }
    Stick[CONTROLMODE] = 1.f;
    ahrs_reset_flag    = 0;
    server.send(200, "text/plain", "OK");
}

void handle_stop() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    Stick[AILERON]  = 0;
    Stick[ELEVATOR] = 0;
    auto_takeoff_and_hover(0.5f);
    server.send(200, "text/plain", "OK");
}

void handle_telemetry() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Cache-Control", "no-cache");

    imu_update();

    String mac = String(MyMacAddr[0], 16) + ":" + String(MyMacAddr[1], 16) + ":" + String(MyMacAddr[2], 16) + ":" +
                 String(MyMacAddr[3], 16) + ":" + String(MyMacAddr[4], 16) + ":" + String(MyMacAddr[5], 16);
    String j;
    j.reserve(256);
    j += '{';
    j += "\"t_us\":";
    j += String((uint32_t)micros());
    j += ",\"acc\":[";
    j += String(acc_x, 6);
    j += ',';
    j += String(acc_y, 6);
    j += ',';
    j += String(acc_z, 6);
    j += ']';
    j += ",\"gyro\":[";
    j += String(gyro_x, 6);
    j += ',';
    j += String(gyro_y, 6);
    j += ',';
    j += String(gyro_z, 6);
    j += ']';
    j += ",\"rc_ok\":";
    j += rc_isconnected() ? "true" : "false";
    j += ",\"flow_pos\":[";
    j += String(current_x, 2);
    j += ',';
    j += String(current_y, 2);
    j += ']';
    j += ",\"mac\":\"";
    j += mac;
    j += "\"}";
    server.send(200, "application/json", j);
}

void handle_logs() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Cache-Control", "no-cache");

    uint32_t last_id = 0;
    if (server.hasArg("last_id")) {
        last_id = server.arg("last_id").toInt();
    }

    String json;
    uint32_t new_last_id = last_id;
    serial_logger_get_json(json, new_last_id);

    server.send(200, "application/json", json);
}

void handle_ping() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "text/plain", "pong");
}

void handle_reset_position() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    current_x = 0.0f;
    current_y = 0.0f;
    server.send(200, "text/plain", "OK");
}

static bool wait_for_sta_ip(uint32_t timeout_ms) {
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED) {
        if (millis() - t0 > timeout_ms) return false;
        delay(50);
    }
    return true;
}

// Proxies the MJPEG stream from camera to client using chunked transfer encoding
// This allows other server operations to continue while streaming
void handle_camera_stream() {
    // Send headers for chunked response
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Cache-Control", "no-cache");
    server.sendHeader("Connection", "close");
    server.setContentLength(CONTENT_LENGTH_UNKNOWN); // Chunked transfer
    server.send(200, "multipart/x-mixed-replace; boundary=--frame", "");

    WiFiClient client = server.client();

    // Ensure STA is connected to camera AP
    if (!wait_for_sta_ip(5000)) {
        server.sendContent("--frame\r\nContent-Type: text/plain\r\n\r\nCamera not available\r\n--frame--\r\n");
        return;
    }

    // Connect to camera
    WiFiClient camera_client;
    if (!camera_client.connect("192.168.4.1", 80)) {
        server.sendContent("--frame\r\nContent-Type: text/plain\r\n\r\nCamera connection failed\r\n--frame--\r\n");
        return;
    }

    // Send request to camera
    camera_client.print("GET /v1/stream HTTP/1.1\r\n");
    camera_client.print("Host: 192.168.4.1\r\n");
    camera_client.print("Connection: close\r\n");
    camera_client.print("\r\n");

    // Skip camera headers
    uint32_t start_time = millis();
    bool headers_done = false;
    while (camera_client.connected() && !headers_done && millis() - start_time < 3000) {
        if (camera_client.available()) {
            String line = camera_client.readStringUntil('\n');
            if (line == "\r" || line == "\n") {
                headers_done = true;
            }
        }
        // Yield to allow other server operations
        server.handleClient();
        delay(1);
    }

    // Stream data in chunks, yielding control periodically
    uint8_t buf[512]; // Smaller buffer for better responsiveness
    uint32_t last_yield = millis();
    start_time = millis();

    while (client.connected() && camera_client.connected() && millis() - start_time < 30000) {
        // Read available data from camera
        int available = camera_client.available();
        if (available > 0) {
            int to_read = min(available, (int)sizeof(buf));
            int n = camera_client.read(buf, to_read);

            if (n > 0) {
                // Send in smaller chunks to allow yielding
                int sent = 0;
                while (sent < n && client.connected()) {
                    int chunk_size = min(128, n - sent); // Small chunks
                    size_t written = client.write(&buf[sent], chunk_size);
                    if (written == 0) break;
                    sent += written;

                    // Yield every 5ms to allow other server operations
                    if (millis() - last_yield > 5) {
                        server.handleClient();
                        last_yield = millis();
                    }
                }
            }
        } else {
            // No data available, yield control
            server.handleClient();
            delay(1);
        }
    }

    // Send closing boundary
    if (client.connected()) {
        client.print("\r\n--frame--\r\n");
    }

    camera_client.stop();
}

void rc_init(void) {
    // Initialize Stick list
    for (uint8_t i = 0; i < 16; i++) Stick[i] = 0.0;

    // ESP-NOW初期化
    // WiFi.mode(WIFI_STA);
    // WiFi.disconnect();

    // WiFi.macAddress((uint8_t *)MyMacAddr);
    // USBSerial.printf("MAC ADDRESS: %02X:%02X:%02X:%02X:%02X:%02X\r\n", MyMacAddr[0], MyMacAddr[1], MyMacAddr[2],
    //                  MyMacAddr[3], MyMacAddr[4], MyMacAddr[5]);

    WiFi.mode(WIFI_AP_STA);
    WiFi.begin("UnitCamS3-WiFi", "");
    WiFi.softAP(WIFI_SSID, WIFI_PASSWORD, 1);

    serial_logger_usb_println("Access Point started!");
    serial_logger_usb_print("IP address: ");
    serial_logger_usb_println(WiFi.softAPIP().toString());

    WiFi.macAddress((uint8_t *)MyMacAddr);
    print("MAC ADDRESS: %02X:%02X:%02X:%02X:%02X:%02X\r\n", MyMacAddr[0], MyMacAddr[1], MyMacAddr[2], MyMacAddr[3],
          MyMacAddr[4], MyMacAddr[5]);

    if (esp_now_init() == ESP_OK) {
        serial_logger_usb_println("ESPNow Init Success");
    } else {
        serial_logger_usb_println("ESPNow Init Failed");
        ESP.restart();
    }

    // MACアドレスブロードキャスト
    uint8_t addr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    memcpy(peerInfo.peer_addr, addr, 6);
    peerInfo.channel = CHANNEL;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        serial_logger_usb_println("Failed to add peer");
        return;
    }
    esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);

    // Send my MAC address
    for (uint16_t i = 0; i < 50; i++) {
        send_peer_info();
        delay(50);
        print("%d\n", i);
    }

    // ESP-NOW再初期化
    // WiFi.mode(WIFI_STA);
    // WiFi.disconnect();
    if (esp_now_init() == ESP_OK) {
        serial_logger_usb_println("ESPNow Init Success2");
    } else {
        serial_logger_usb_println("ESPNow Init Failed2");
        ESP.restart();
    }

    // ESP-NOWコールバック登録
    esp_now_register_recv_cb(OnDataRecv);
    serial_logger_usb_println("ESP-NOW Ready.");

    serial_logger_init();

    server.on("/buzz/start", HTTP_GET, startBuzz);
    server.on("/buzz/stop", HTTP_GET, stopBuzz);
    server.on("/land", HTTP_GET, auto_land);
    server.on("/hover", HTTP_GET, do_hover);
    server.on("/move", HTTP_GET, handle_movement);
    server.on("/move/stop", HTTP_GET, handle_stop);
    server.on("/telemetry", HTTP_GET, handle_telemetry);
    server.on("/logs", HTTP_GET, handle_logs);
    server.on("/ping", HTTP_GET, handle_ping);
    server.on("/reset/position", HTTP_GET, handle_reset_position);
    server.on("/camera/stream", HTTP_GET, handle_camera_stream);

    server.on("/buzz/start", HTTP_OPTIONS, handle_options);
    server.on("/buzz/stop", HTTP_OPTIONS, handle_options);
    server.on("/land", HTTP_OPTIONS, handle_options);
    server.on("/hover", HTTP_OPTIONS, handle_options);
    server.on("/move", HTTP_OPTIONS, handle_options);
    server.on("/move/stop", HTTP_OPTIONS, handle_options);
    server.on("/logs", HTTP_OPTIONS, handle_options);
    server.on("/ping", HTTP_OPTIONS, handle_options);
    server.on("/reset/position", HTTP_OPTIONS, handle_options);

    // server.on("/move", handle_movement);
    // server.on("/move/stop", handle_stop);
    server.begin();
}

void send_peer_info(void) {
    uint8_t data[11];
    data[0] = CHANNEL;
    memcpy(&data[1], (uint8_t *)MyMacAddr, 6);
    memcpy(&data[1 + 6], (uint8_t *)peer_command, 4);
    esp_now_send(peerInfo.peer_addr, data, 11);
}

uint8_t telemetry_send(uint8_t *data, uint16_t datalen) {
    static uint32_t cnt       = 0;
    static uint8_t error_flag = 0;
    static uint8_t state      = 0;

    esp_err_t result;

    if ((error_flag == 0) && (state == 0)) {
        result = esp_now_send(peerInfo.peer_addr, data, datalen);
        cnt    = 0;
    } else
        cnt++;

    if (esp_now_send_status == 0) {
        error_flag = 0;
        // state = 0;
    } else {
        error_flag = 1;
        // state = 1;
    }
    // 一度送信エラーを検知してもしばらくしたら復帰する
    if (cnt > 500) {
        error_flag = 0;
        cnt        = 0;
    }
    cnt++;
    // USBSerial.printf("%6d %d %d\r\n", cnt, error_flag, esp_now_send_status);

    return error_flag;
}

void rc_end(void) {
    // Ps3.end();
}

uint8_t rc_isconnected(void) {
    bool status;
    // Connect_flag++;
    if (Connect_flag < 40)
        status = 1;
    else
        status = 0;
    // USBSerial.printf("%d \n\r", Connect_flag);
    return status;
}

void handleClient() {
    server.handleClient();
}

void rc_demo() {
}
