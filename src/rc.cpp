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

// webserver task to handle http requests independently
static void webserver_task(void *param) {
    for (;;) {
        server.handleClient();
        // small delay to yield but keep responsiveness high
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static WiFiServer cameraServer(81);

static void camera_stream_task(void *param);

void start_camera_stream_server() {
    // start listening on port 81
    cameraServer.begin();
    cameraServer.setNoDelay(true);

    // spawn a task that accepts clients and spawns a worker per client
    // lower priority (0) to avoid interfering with main webserver
    xTaskCreatePinnedToCore(camera_stream_task,  // task function
                            "cam_stream_task",   // name
                            6144,                // stack
                            nullptr,             // param
                            0,                   // priority (lower than main loop)
                            nullptr,             // handle
                            0                    // core 0 (leave core 1 for wifi/arduino)
    );
}

static bool wait_for_sta_ip(uint32_t timeout_ms) {
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED) {
        if (millis() - t0 > timeout_ms) return false;
        delay(50);
        yield();
    }
    return true;
}

static void handle_one_camera_client_task(void *param) {
    WiFiClient client_out = *static_cast<WiFiClient *>(param);
    delete static_cast<WiFiClient *>(param);

    client_out.setTimeout(2000);

    // Parse request line to preserve query string (and to drain headers)
    String reqLine = client_out.readStringUntil('\n');
    if (reqLine.length() == 0) {
        client_out.stop();
        vTaskDelete(nullptr);
        return;
    }
    // Drain the rest of the headers quickly
    unsigned long tdrain = millis();
    while (millis() - tdrain < 500) {
        String l = client_out.readStringUntil('\n');
        if (l.length() == 0 || l == "\r") break;
        vTaskDelay(1);
    }

    // Extract query (optional)
    String qs;
    int sp1 = reqLine.indexOf(' ');
    int sp2 = reqLine.indexOf(' ', sp1 + 1);
    if (sp1 > 0 && sp2 > sp1) {
        String path = reqLine.substring(sp1 + 1, sp2);
        int q       = path.indexOf('?');
        if (q >= 0) qs = path.substring(q);
    }

    // reduce timeout to avoid blocking main server - check quickly and fail fast
    if (!wait_for_sta_ip(500)) {
        client_out.print(
            "HTTP/1.1 503 Service Unavailable\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\nsta not "
            "connected");
        client_out.stop();
        vTaskDelete(nullptr);
        return;
    }

    WiFiClient client_in;
    client_in.setTimeout(2000);
    if (!client_in.connect(IPAddress(192, 168, 4, 1), 80)) {
        client_out.print(
            "HTTP/1.1 502 Bad Gateway\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\nupstream connect failed");
        client_out.stop();
        vTaskDelete(nullptr);
        return;
    }

    String upstream = "/api/v1/stream";
    upstream += qs;

    client_in.print("GET ");
    client_in.print(upstream);
    client_in.print(" HTTP/1.1\r\n");
    client_in.print("Host: 192.168.4.1\r\n");
    client_in.print("Accept: multipart/x-mixed-replace,image/*,*/*;q=0.8\r\n");
    client_in.print("Connection: close\r\n\r\n");

    String status = client_in.readStringUntil('\n');
    status.trim();
    if (!status.startsWith("HTTP/1.1 200") && !status.startsWith("HTTP/1.0 200")) {
        client_out.print("HTTP/1.1 502 Bad Gateway\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\n");
        client_out.print("bad camera status: ");
        client_out.print(status);
        client_in.stop();
        client_out.stop();
        vTaskDelete(nullptr);
        return;
    }

    String content_type = "multipart/x-mixed-replace; boundary=--frame";
    unsigned long th    = millis();
    for (;;) {
        String h = client_in.readStringUntil('\n');
        if (h.length() == 0) break;
        h.trim();
        if (h.length() == 0) break;
        if (h.startsWith("Content-Type:")) {
            int c = h.indexOf(':');
            if (c >= 0) {
                content_type = h.substring(c + 1);
                content_type.trim();
            }
        }
        if (millis() - th > 2000) break;
        vTaskDelay(1);
    }

    client_out.print("HTTP/1.1 200 OK\r\n");
    client_out.print("Content-Type: ");
    client_out.print(content_type);
    client_out.print("\r\n");
    client_out.print("Access-Control-Allow-Origin: *\r\n");
    client_out.print("Cache-Control: no-cache\r\n");
    client_out.print("Connection: close\r\n\r\n");

    // Pump bytes with frequent yields to allow main server to process requests
    uint8_t buf[1460];
    uint32_t lastYield = millis();
    for (;;) {
        int n = client_in.read(buf, sizeof(buf));
        if (n > 0) {
            if (client_out.write(buf, n) == 0) break;
        } else {
            if (!client_in.connected() && !client_in.available()) break;
            vTaskDelay(1);
        }
        if (!client_out.connected()) break;

        // yield every 10ms to allow main server to handle requests
        if (millis() - lastYield > 10) {
            vTaskDelay(1);
            lastYield = millis();
        }
    }

    client_in.stop();
    client_out.stop();
    vTaskDelete(nullptr);
}

static void handle_one_camera_client(WiFiClient client_out) {
    // spawn each client in its own task to avoid blocking the main camera task
    WiFiClient *client_copy = new WiFiClient(std::move(client_out));
    if (xTaskCreatePinnedToCore(handle_one_camera_client_task, "cam_client", 8192, client_copy, 0, nullptr, 0) !=
        pdPASS) {
        // task creation failed, clean up
        client_copy->stop();
        delete client_copy;
    }
}

static void camera_stream_task(void *param) {
    for (;;) {
        WiFiClient client = cameraServer.available();
        if (client) {
            handle_one_camera_client(std::move(client));
        } else {
            vTaskDelay(10);  // yield more frequently to allow main server
        }
    }
}

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

    // use cached imu values updated in main loop - no need to read spi here
    char json[320];
    snprintf(json, sizeof(json),
             "{\"t_us\":%u,\"acc\":[%.6f,%.6f,%.6f],\"gyro\":[%.6f,%.6f,%.6f],\"rc_ok\":%s,\"flow_pos\":[%.2f,%.2f],"
             "\"mac\":\"%02x:%02x:%02x:%02x:%02x:%02x\"}",
             (uint32_t)micros(), acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, rc_isconnected() ? "true" : "false",
             current_x, current_y, MyMacAddr[0], MyMacAddr[1], MyMacAddr[2], MyMacAddr[3], MyMacAddr[4], MyMacAddr[5]);
    server.send(200, "application/json", json);
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

void rc_init(void) {
    for (uint8_t i = 0; i < 16; i++) Stick[i] = 0.0;

    WiFi.mode(WIFI_AP_STA);
    WiFi.begin("UnitCamS3-WiFi", "");
    WiFi.softAPConfig(IPAddress(10, 0, 0, 1), IPAddress(10, 0, 0, 1), IPAddress(255, 255, 255, 0));
    WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);

    serial_logger_usb_println("Access Point started!");
    serial_logger_usb_print("AP IP: ");
    serial_logger_usb_println(WiFi.softAPIP().toString());

    WiFi.macAddress((uint8_t *)MyMacAddr);
    print("MAC ADDRESS: %02X:%02X:%02X:%02X:%02X:%02X\r\n", MyMacAddr[0], MyMacAddr[1], MyMacAddr[2], MyMacAddr[3],
          MyMacAddr[4], MyMacAddr[5]);

    if (esp_now_init() != ESP_OK) {
        serial_logger_usb_println("ESPNow Init Failed");
        ESP.restart();
    }

    uint8_t addr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    memcpy(peerInfo.peer_addr, addr, 6);
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        serial_logger_usb_println("Failed to add peer");
        return;
    }

    for (uint16_t i = 0; i < 50; i++) {
        send_peer_info();
        delay(50);
        print("%d\n", i);
    }

    if (esp_now_init() != ESP_OK) {
        serial_logger_usb_println("ESPNow Init Failed2");
        ESP.restart();
    }
    esp_now_register_recv_cb(OnDataRecv);
    serial_logger_usb_println("ESP-NOW Ready.");

    // sync channel with joined AP if available
    if (wait_for_sta_ip(8000)) {
        wifi_ap_record_t apinfo{};
        if (esp_wifi_sta_get_ap_info(&apinfo) == ESP_OK) {
            esp_wifi_set_channel(apinfo.primary, WIFI_SECOND_CHAN_NONE);
            serial_logger_usb_print("Synced WiFi channel to ");
            serial_logger_usb_println(String(apinfo.primary));
        }
    }

    serial_logger_init();

    // normal api continues on port 80, non-blocking
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

    server.on("/buzz/start", HTTP_OPTIONS, handle_options);
    server.on("/buzz/stop", HTTP_OPTIONS, handle_options);
    server.on("/land", HTTP_OPTIONS, handle_options);
    server.on("/hover", HTTP_OPTIONS, handle_options);
    server.on("/move", HTTP_OPTIONS, handle_options);
    server.on("/move/stop", HTTP_OPTIONS, handle_options);
    server.on("/logs", HTTP_OPTIONS, handle_options);
    server.on("/ping", HTTP_OPTIONS, handle_options);
    server.on("/reset/position", HTTP_OPTIONS, handle_options);

    server.on("/camera/stream", HTTP_GET, []() {
        server.sendHeader("Access-Control-Allow-Origin", "*");
        String loc = "http://" + WiFi.softAPIP().toString() + ":81/stream";
        server.sendHeader("Location", loc);
        server.send(302);
    });

    server.begin();

    // start webserver in separate task so it doesn't block control loop
    xTaskCreatePinnedToCore(webserver_task, "webserver", 8192, nullptr, 1, nullptr, 1);

    // // start the separate camera server on port 81 so webserver never blocks
    start_camera_stream_server();
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
