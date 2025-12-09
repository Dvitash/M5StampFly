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
#include <cmath>

extern volatile float acc_x, acc_y, acc_z;
extern volatile float gyro_x, gyro_y, gyro_z;
extern volatile float current_x, current_y;
extern volatile float current_x, current_y;
extern volatile float target_x, target_y;
extern volatile uint8_t PositionHold_flag;
extern volatile uint8_t Mode;

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
    const String dir = server.arg("direction");

    // step size in meters; we default to 0.5 m and clamp if a 'step' arg is provided
    float step = server.hasArg("step") ? server.arg("step").toFloat() : 0.5f;
    if (step <= 0.0f) step = 0.5f;
    if (step > 2.0f) step = 2.0f;

    float old_tx = target_x;
    float old_ty = target_y;

    if (dir == "forward") {
        // X axis is forward/backward (positive forward)
        target_y -= step;
    } else if (dir == "backward") {
        target_y += step;
    } else if (dir == "right") {
        // Y axis is left/right (positive right)
        target_x += step;
    } else if (dir == "left") {
        target_x -= step;
    } else {
        server.send(400, "text/plain", "bad direction");
        return;
    }

    // Ensure angle control + position hold are active
    Stick[CONTROLMODE] = 1.0f;  // ANGLECONTROL
    PositionHold_flag  = 1;     // enable position controller
    ahrs_reset_flag    = 0;

    // Debug: log the move request
    print("MOVE CMD: dir=%s step=%.2f m | target: (%.3f, %.3f) -> (%.3f, %.3f) | current: (%.3f, %.3f)\r\n",
          dir.c_str(), step, old_tx, old_ty, target_x, target_y, current_x, current_y);

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

static void handle_target_get() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    char buf[160];
    snprintf(buf, sizeof(buf), "{\"target_x\":%.3f,\"target_y\":%.3f,\"current_x\":%.3f,\"current_y\":%.3f}", target_x,
             target_y, current_x, current_y);
    server.send(200, "application/json", buf);
}

static void handle_target_set() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    if (!server.hasArg("x") || !server.hasArg("y")) {
        server.send(400, "text/plain", "missing x or y");
        return;
    }

    float x = server.arg("x").toFloat();
    float y = server.arg("y").toFloat();

    if (!std::isfinite(x) || !std::isfinite(y)) {
        server.send(400, "text/plain", "bad coords");
        return;
    }

    const float limit_m = 10.0f;
    if (x < -limit_m) x = -limit_m;
    if (x > limit_m) x = limit_m;
    if (y < -limit_m) y = -limit_m;
    if (y > limit_m) y = limit_m;

    target_x           = x;
    target_y           = y;
    PositionHold_flag  = 1;
    Stick[CONTROLMODE] = ANGLECONTROL;
    ahrs_reset_flag    = 0;

    char buf[160];
    snprintf(buf, sizeof(buf), "{\"target_x\":%.3f,\"target_y\":%.3f,\"current_x\":%.3f,\"current_y\":%.3f}", target_x,
             target_y, current_x, current_y);
    server.send(200, "application/json", buf);
}

static void handle_pos_gains_get() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    float kp, ki, kd;
    get_position_hold_gains(kp, ki, kd);
    char buf[96];
    snprintf(buf, sizeof(buf), "{\"kp\":%.5f,\"ki\":%.5f,\"kd\":%.5f}", kp, ki, kd);
    server.send(200, "application/json", buf);
}

static void handle_pos_gains_set() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    float kp, ki, kd;
    get_position_hold_gains(kp, ki, kd);

    if (server.hasArg("kp")) kp = server.arg("kp").toFloat();
    if (server.hasArg("ki")) ki = server.arg("ki").toFloat();
    if (server.hasArg("kd")) kd = server.arg("kd").toFloat();

    set_position_hold_gains(kp, ki, kd);
    get_position_hold_gains(kp, ki, kd);

    char buf[96];
    snprintf(buf, sizeof(buf), "{\"kp\":%.5f,\"ki\":%.5f,\"kd\":%.5f}", kp, ki, kd);
    server.send(200, "application/json", buf);
}

// -------- action sequence support --------
enum class SeqActionType : uint8_t { MOVE, WAIT, ALT, LAND };

struct SeqAction {
    SeqActionType type;
    char axis;    // 'x' or 'y' for move
    int8_t sign;  // +1 or -1 for move
    float speed_mps;
    float duration_s;
    float alt_target;
};

static SeqAction g_seq[16];
static uint8_t g_seq_len        = 0;
static uint8_t g_seq_index      = 0;
static float g_seq_elapsed      = 0.0f;
static volatile bool g_seq_live = false;

static void sequence_reset_state() {
    g_seq_len     = 0;
    g_seq_index   = 0;
    g_seq_elapsed = 0.0f;
    g_seq_live    = false;
}

static bool parse_action_token(const String &token, SeqAction &out) {
    String t = token;
    t.trim();
    if (t.length() == 0) return false;

    int first = t.indexOf(':');
    String kind;
    String durStr;
    String speedStr;
    if (first < 0) {
        kind = t;
    } else {
        kind       = t.substring(0, first);
        int second = t.indexOf(':', first + 1);
        if (second < 0) {
            durStr = t.substring(first + 1);
        } else {
            durStr   = t.substring(first + 1, second);
            speedStr = t.substring(second + 1);
        }
    }
    kind.toLowerCase();

    float duration = durStr.length() ? durStr.toFloat() : 0.0f;
    float speed    = speedStr.length() ? speedStr.toFloat() : 0.35f;

    // default clamps
    if (speed <= 0.0f) speed = 0.35f;
    if (speed > 1.5f) speed = 1.5f;

    // duration handling per kind
    if (kind == "wait" || kind == "pause" || kind == "move" || kind == "forward" || kind == "backward" ||
        kind == "left" || kind == "right") {
        if (duration <= 0.0f) return false;
        if (duration < 0.01f) duration = 0.01f;
    } else {
        // for alt/land allow zero duration (instant)
        if (duration < 0.0f) duration = 0.0f;
    }

    if (kind == "wait" || kind == "pause") {
        out.type       = SeqActionType::WAIT;
        out.axis       = 'x';
        out.sign       = 0;
        out.speed_mps  = 0.0f;
        out.duration_s = duration;
        out.alt_target = 0.0f;
        return true;
    }

    if (kind == "alt" || kind == "altitude") {
        float alt = duration;
        if (alt <= 0.0f && speedStr.length()) alt = speedStr.toFloat();  // fallback
        if (alt < 0.2f) alt = 0.2f;
        if (alt > 1.8f) alt = 1.8f;
        out.type       = SeqActionType::ALT;
        out.axis       = 'x';
        out.sign       = 0;
        out.speed_mps  = 0.0f;
        out.duration_s = 0.0f;
        out.alt_target = alt;
        return true;
    }

    if (kind == "land" || kind == "landing") {
        out.type       = SeqActionType::LAND;
        out.axis       = 'x';
        out.sign       = 0;
        out.speed_mps  = 0.0f;
        out.duration_s = 0.0f;
        out.alt_target = 0.0f;
        return true;
    }

    out.type       = SeqActionType::MOVE;
    out.duration_s = duration;
    out.speed_mps  = speed;
    out.alt_target = 0.0f;
    if (kind == "forward") {
        out.axis = 'y';
        out.sign = -1;
    } else if (kind == "backward") {
        out.axis = 'y';
        out.sign = 1;
    } else if (kind == "right") {
        out.axis = 'x';
        out.sign = 1;
    } else if (kind == "left") {
        out.axis = 'x';
        out.sign = -1;
    } else {
        return false;
    }
    return true;
}

static void handle_sequence() {
    server.sendHeader("Access-Control-Allow-Origin", "*");

    String actions = server.arg("actions");
    if (!actions.length()) {
        server.send(400, "text/plain", "missing actions");
        return;
    }

    float alt = server.hasArg("altitude") ? server.arg("altitude").toFloat() : 0.5f;
    if (alt < 0.2f) alt = 0.2f;
    if (alt > 1.8f) alt = 1.8f;

    SeqAction parsed[16];
    uint8_t count = 0;

    int start = 0;
    while (start < actions.length() && count < 16) {
        int comma = actions.indexOf(',', start);
        if (comma < 0) comma = actions.length();
        String token = actions.substring(start, comma);
        if (parse_action_token(token, parsed[count])) {
            count++;
        } else {
            server.send(400, "text/plain", "bad action: " + token);
            return;
        }
        start = comma + 1;
    }

    if (count == 0) {
        server.send(400, "text/plain", "no valid actions");
        return;
    }

    sequence_reset_state();
    for (uint8_t i = 0; i < count; i++) g_seq[i] = parsed[i];
    g_seq_len  = count;
    g_seq_live = true;

    reset_position_state();
    auto_takeoff_and_hover(alt);
    Stick[CONTROLMODE]    = ANGLECONTROL;
    Stick[ALTCONTROLMODE] = AUTO_ALT;
    // free-fly laterally during sequence; no position hold corrections
    PositionHold_flag = 0;
    ahrs_reset_flag   = 0;

    print("SEQ start: %u actions, alt=%.2f\r\n", g_seq_len, alt);
    server.send(200, "text/plain", "OK");
}

static void handle_sequence_stop() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    sequence_stop();
    server.send(200, "text/plain", "stopped");
}

void sequence_stop(void) {
    g_seq_live        = false;
    g_seq_len         = 0;
    g_seq_index       = 0;
    g_seq_elapsed     = 0.0f;
    Stick[AILERON]    = 0.0f;
    Stick[ELEVATOR]   = 0.0f;
    PositionHold_flag = 0;
    print("SEQ stop\r\n");
}

void sequence_tick(float dt_sec) {
    if (!g_seq_live || g_seq_len == 0) return;
    if (dt_sec <= 0.0f) return;
    if (Mode != FLIGHT_MODE && Mode != AUTO_LANDING_MODE && Mode != PARKING_MODE) return;

    if (g_seq_index >= g_seq_len) {
        sequence_stop();
        return;
    }

    SeqAction &act = g_seq[g_seq_index];

    // default to zero stick unless the step commands motion
    float ail_cmd = 0.0f;
    float ele_cmd = 0.0f;

    if (act.type == SeqActionType::MOVE) {
        // map speed (m/s) into a normalized stick deflection (0..1)
        float mag = act.speed_mps / 1.0f;  // 1 m/s -> full stick
        if (mag > 1.0f) mag = 1.0f;
        if (mag < 0.0f) mag = 0.0f;

        if (act.axis == 'x') {
            ail_cmd = mag * (float)act.sign;
        } else if (act.axis == 'y') {
            ele_cmd = mag * (float)act.sign;
        }
    } else if (act.type == SeqActionType::ALT) {
        Alt_ref = act.alt_target;
        g_seq_index++;
        g_seq_elapsed = 0.0f;
        if (g_seq_index >= g_seq_len) sequence_stop();
        return;
    } else if (act.type == SeqActionType::LAND) {
        request_mode_change(AUTO_LANDING_MODE);
        sequence_stop();
        return;
    }

    // apply virtual sticks directly
    Stick[AILERON]  = ail_cmd;
    Stick[ELEVATOR] = ele_cmd;

    g_seq_elapsed += dt_sec;
    if (g_seq_elapsed + 1e-5f >= act.duration_s) {
        g_seq_index++;
        g_seq_elapsed = 0.0f;
        if (g_seq_index >= g_seq_len) {
            sequence_stop();
        }
    }
}

void rc_init(void) {
    for (uint8_t i = 0; i < 16; i++) Stick[i] = 0.0;

    WiFi.mode(WIFI_AP_STA);
    // WiFi.begin("UnitCamS3-WiFi", "");
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
    server.on("/target", HTTP_GET, handle_target_get);
    server.on("/target", HTTP_POST, handle_target_set);
    server.on("/pos/gains", HTTP_GET, handle_pos_gains_get);
    server.on("/pos/gains", HTTP_POST, handle_pos_gains_set);
    server.on("/sequence", HTTP_POST, handle_sequence);
    server.on("/sequence/stop", HTTP_GET, handle_sequence_stop);

    server.on("/buzz/start", HTTP_OPTIONS, handle_options);
    server.on("/buzz/stop", HTTP_OPTIONS, handle_options);
    server.on("/land", HTTP_OPTIONS, handle_options);
    server.on("/hover", HTTP_OPTIONS, handle_options);
    server.on("/move", HTTP_OPTIONS, handle_options);
    server.on("/move/stop", HTTP_OPTIONS, handle_options);
    server.on("/logs", HTTP_OPTIONS, handle_options);
    server.on("/ping", HTTP_OPTIONS, handle_options);
    server.on("/reset/position", HTTP_OPTIONS, handle_options);
    server.on("/target", HTTP_OPTIONS, handle_options);
    server.on("/pos/gains", HTTP_OPTIONS, handle_options);
    server.on("/sequence", HTTP_OPTIONS, handle_options);
    server.on("/sequence/stop", HTTP_OPTIONS, handle_options);

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
