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
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WebServer.h>
#include "flight_control.hpp"
#include "sensor.hpp"
#include <buzzer.h>
#include "imu.hpp"
#include "serial_logger.hpp"
#include <cmath>

#define SUPABASE_PROJECT_ID "ezdenxjlzxtpdjzebfjo"
#define SUPABASE_ANON_KEY                                                                                              \
    "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9."                                                                            \
    "eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6ImV6ZGVueGpsenh0cGRqemViZmpvIiwicm9sZSI6ImFub24iLCJpYXQiOjE3Njk4ODkyMzYsImV4cCI6" \
    "MjA4NTQ2NTIzNn0.Sp7Ib3ysZS4vm1XfZ-Yr4-_odwPK2U91I1MOiXeZV18"
#define SUPABASE_PUBLISHABLE_KEY "sb_publishable_TtEga7NJ4QtXjAI4qQAhhw_2DihyA2T"

extern volatile float acc_x, acc_y, acc_z;
extern volatile float gyro_x, gyro_y, gyro_z;
extern volatile float current_x, current_y;
extern volatile float target_x, target_y;
extern volatile uint8_t PositionHold_flag;
extern volatile uint8_t Mode;

WebServer server(80);

namespace {
constexpr uint32_t kBackendPollIntervalMs       = 2000;
constexpr float kBackendAutoHoverAltitudeMeters = 0.5f;

struct BackendCommandState {
    bool sweep_now             = false;
    bool force_scan            = false;
    bool sweep_intervals       = false;
    bool temporary_disable_off = false;
};
}  // namespace

// webserver task to handle http requests independently
static void webserver_task(void *param) {
    for (;;) {
        server.handleClient();
        // small delay to yield but keep responsiveness high
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// org_id from drone_org_registry_map, -1 = not yet fetched
static int8_t g_org_id = -1;
static volatile uint8_t g_backend_sweep_now = 0;
static volatile uint8_t g_backend_force_scan = 0;
static volatile uint8_t g_backend_sweep_intervals = 0;
static volatile uint8_t g_backend_temporary_disable_off = 0;

static int battery_voltage_to_percent(float voltage) {
    // Clamp the pack reading into a simple dashboard percentage.
    constexpr float kBatteryEmptyV = 3.30f;
    constexpr float kBatteryFullV  = 4.20f;

    if (voltage <= kBatteryEmptyV) return 0;
    if (voltage >= kBatteryFullV) return 100;

    float pct = ((voltage - kBatteryEmptyV) / (kBatteryFullV - kBatteryEmptyV)) * 100.0f;
    return (int)(pct + 0.5f);
}

static int supabase_get_json(const char *url, String &body) {
    WiFiClientSecure client;
    client.setInsecure();
    HTTPClient http;
    http.begin(client, url);
    http.addHeader("apikey", SUPABASE_ANON_KEY);
    http.addHeader("Authorization", "Bearer " SUPABASE_ANON_KEY);
    http.addHeader("Accept", "application/json");
    http.setTimeout(5000);
    int code = http.GET();
    body     = http.getString();
    http.end();
    return code;
}

static bool json_field_is_true(const String &body, const char *field_name) {
    String token = String("\"") + field_name + "\":";
    int idx      = body.indexOf(token);
    if (idx < 0) return false;

    idx += token.length();
    while (idx < body.length() && (body[idx] == ' ' || body[idx] == '\t' || body[idx] == '\r' || body[idx] == '\n')) {
        idx++;
    }

    return body.substring(idx).startsWith("true");
}

static void sync_backend_command_state(const BackendCommandState &state) {
    g_backend_sweep_now             = state.sweep_now ? 1 : 0;
    g_backend_force_scan            = state.force_scan ? 1 : 0;
    g_backend_sweep_intervals       = state.sweep_intervals ? 1 : 0;
    g_backend_temporary_disable_off = state.temporary_disable_off ? 1 : 0;
}

static bool backend_airborne_mode(uint8_t mode) {
    return mode == FLIGHT_MODE || mode == AUTO_LANDING_MODE || mode == FLIP_MODE;
}

static void apply_backend_command_state(const BackendCommandState &state) {
    static bool last_takeoff_request = false;
    static bool last_disable_request = false;

    const bool takeoff_request = !state.temporary_disable_off && (state.sweep_now || state.force_scan || state.sweep_intervals);
    const bool disable_request = state.temporary_disable_off;

    if (disable_request && backend_airborne_mode(Mode) && Mode != AUTO_LANDING_MODE) {
        if (!last_disable_request) {
            serial_logger_usb_print("backend: temporary_disable_off=true -> auto landing from mode ");
            serial_logger_usb_println(String((unsigned int)Mode));
        }
        request_mode_change(AUTO_LANDING_MODE);
    } else if (disable_request && !last_disable_request) {
        if (Mode == PARKING_MODE) {
            serial_logger_usb_println("backend: temporary_disable_off=true while already parked");
        } else {
            serial_logger_usb_print("backend: temporary_disable_off=true while mode=");
            serial_logger_usb_println(String((unsigned int)Mode));
        }
    }

    if (disable_request && last_takeoff_request) {
        serial_logger_usb_println("backend: disable active, clearing pending sweep/scan latch");
        last_takeoff_request = false;
    }

    if (disable_request && !last_disable_request && backend_airborne_mode(Mode)) {
        // keep the disable request dominant over any concurrent scan/takeoff command
        sequence_stop();
        PositionHold_flag = 0;
    }

    if (!disable_request && takeoff_request && !last_takeoff_request) {
        if (Mode == PARKING_MODE) {
            serial_logger_usb_println("backend: sweep/scan request -> auto takeoff");
            auto_takeoff_and_hover(kBackendAutoHoverAltitudeMeters);
        } else if (backend_airborne_mode(Mode)) {
            serial_logger_usb_print("backend: sweep/scan request ignored because drone is airborne in mode ");
            serial_logger_usb_println(String((unsigned int)Mode));
        } else {
            serial_logger_usb_print("backend: sweep/scan request ignored because drone is not ready (mode=");
            serial_logger_usb_print(String((unsigned int)Mode));
            serial_logger_usb_println(")");
        }
    }

    last_takeoff_request = takeoff_request;
    last_disable_request = disable_request;
}

static bool fetch_backend_command_state_from_table(const char *table_name, const char *select_clause,
                                                   int8_t org_id, String &body, int &code) {
    char url_buf[320];
    snprintf(url_buf, sizeof(url_buf), "https://" SUPABASE_PROJECT_ID ".supabase.co/rest/v1/%s?select=%s&org_id=eq.%d",
             table_name, select_clause, (int)org_id);
    code = supabase_get_json(url_buf, body);
    if (code == 200) {
        serial_logger_usb_print("backend table: ");
        serial_logger_usb_println(table_name);
        return true;
    }
    return false;
}

static bool fetch_backend_command_state(BackendCommandState &state, int8_t org_id, String &body, int &code) {
    if (fetch_backend_command_state_from_table("org_drone_priority_state",
                                               "force_scan,sweep_intervals,disable_temporarily", org_id, body, code)) {
        state.sweep_now             = false;
        state.force_scan            = json_field_is_true(body, "force_scan");
        state.sweep_intervals       = json_field_is_true(body, "sweep_intervals");
        state.temporary_disable_off = json_field_is_true(body, "disable_temporarily");
        return true;
    }

    if (fetch_backend_command_state_from_table("sweep_now", "force_scan,sweep_intervals,disable_temporarily", org_id,
                                               body, code)) {
        state.sweep_now             = false;
        state.force_scan            = json_field_is_true(body, "force_scan");
        state.sweep_intervals       = json_field_is_true(body, "sweep_intervals");
        state.temporary_disable_off = json_field_is_true(body, "disable_temporarily");
        return true;
    }

    if (fetch_backend_command_state_from_table("sweep_now", "sweep_now,force_scan,sweep_intervals,temporary_disable_off",
                                               org_id, body, code)) {
        state.sweep_now             = json_field_is_true(body, "sweep_now");
        state.force_scan            = json_field_is_true(body, "force_scan");
        state.sweep_intervals       = json_field_is_true(body, "sweep_intervals");
        state.temporary_disable_off = json_field_is_true(body, "temporary_disable_off");
        return true;
    }

    if (!fetch_backend_command_state_from_table("sweep_now", "sweep_now", org_id, body, code)) return false;

    state.sweep_now             = json_field_is_true(body, "sweep_now");
    state.force_scan            = false;
    state.sweep_intervals       = false;
    state.temporary_disable_off = false;
    return true;
}

// Post only battery voltage and WiFi RSSI after retrieving data.
static void post_drone_dashboard_state() {
    if (WiFi.status() != WL_CONNECTED || g_org_id < 0) return;

    const char *url = "https://" SUPABASE_PROJECT_ID ".supabase.co/rest/v1/drone_dashboard_state";
    long wifi_rssi  = WiFi.RSSI();
    char hardware_id[18];
    snprintf(hardware_id, sizeof(hardware_id), "%02X:%02X:%02X:%02X:%02X:%02X", MyMacAddr[0], MyMacAddr[1],
             MyMacAddr[2], MyMacAddr[3], MyMacAddr[4], MyMacAddr[5]);
    const char *status = (Mode == PARKING_MODE) ? "Docked" : "Active";
    int battery_pct = battery_voltage_to_percent(Voltage);
    int wifi_pct    = (int)lroundf((float)(wifi_rssi + 100) * (100.0f / 50.0f));
    if (wifi_pct < 0) wifi_pct = 0;
    if (wifi_pct > 100) wifi_pct = 100;
    String payload  = "{";
    payload += "\"org_id\":";
    payload += String((int)g_org_id);
    payload += ",\"hardware_id\":\"";
    payload += hardware_id;
    payload += "\"";
    payload += ",\"status\":\"";
    payload += status;
    payload += "\"";
    payload += ",\"battery_level\":";
    payload += String(battery_pct);
    payload += ",\"wifi_strength\":";
    payload += String(wifi_pct);
    payload += "}";

    WiFiClientSecure client;
    client.setInsecure();
    HTTPClient http;
    http.begin(client, url);
    http.addHeader("apikey", SUPABASE_ANON_KEY);
    http.addHeader("Authorization", "Bearer " SUPABASE_ANON_KEY);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("Prefer", "resolution=merge-duplicates,return=minimal");
    http.setTimeout(5000);

    int post_code = http.POST(payload);
    if (post_code < 200 || post_code >= 300) {
        serial_logger_usb_print("sample POST failed: ");
        serial_logger_usb_print(String(post_code).c_str());
        String post_body = http.getString();
        if (post_body.length() > 0) {
            serial_logger_usb_print(" ");
            serial_logger_usb_println(post_body.c_str());
        } else {
            serial_logger_usb_println("");
        }
    }
    http.end();
}

static void supabase_registry_task(void *param) {
    char url_buf[180];

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (WiFi.status() != WL_CONNECTED) continue;

        // fetch org_id from drone_org_registry_map if not yet known
        if (g_org_id < 0) {
            // colons are reserved - use in.(%%22...%%22) which properly handles quoted values
            snprintf(url_buf, sizeof(url_buf),
                     "https://" SUPABASE_PROJECT_ID
                     ".supabase.co/rest/v1/"
                     "drone_org_registry_map?select=org_id&hardware_id=in.(%%22%02X%%3A%02X%%3A%02X%%3A%02X%%3A%02X%%"
                     "3A%02X%%22)",
                     MyMacAddr[0], MyMacAddr[1], MyMacAddr[2], MyMacAddr[3], MyMacAddr[4], MyMacAddr[5]);

            WiFiClientSecure client;
            client.setInsecure();
            HTTPClient http;
            http.begin(client, url_buf);
            http.addHeader("apikey", SUPABASE_ANON_KEY);
            http.addHeader("Authorization", "Bearer " SUPABASE_ANON_KEY);
            http.addHeader("Accept", "application/json");
            http.setTimeout(5000);
            int code    = http.GET();
            String body = http.getString();
            http.end();

            if (code != 200 && body.length() > 0) {
                serial_logger_usb_print("drone_org_registry_map body: ");
                serial_logger_usb_println(body.c_str());
            }
            if (code != 200) {
                serial_logger_usb_print("drone_org_registry_map url: ");
                serial_logger_usb_println(url_buf);
            }

            if (code == 200 && body.length() > 0) {
                // parse [{"org_id":N}] - find "org_id": and read number
                int idx = body.indexOf("\"org_id\":");
                if (idx >= 0) {
                    idx += 9;  // skip "org_id":
                    int end = body.indexOf(',', idx);
                    if (end < 0) end = body.indexOf('}', idx);
                    if (end > idx) {
                        g_org_id = (int8_t)body.substring(idx, end).toInt();
                        serial_logger_usb_print("org_id: ");
                        serial_logger_usb_println(String(g_org_id).c_str());
                    }
                }
                if (g_org_id < 0) {
                    serial_logger_usb_print("drone_org_registry_map: no match. querying MAC ");
                    char mac_dbg[18];
                    snprintf(mac_dbg, sizeof(mac_dbg), "%02X:%02X:%02X:%02X:%02X:%02X", MyMacAddr[0], MyMacAddr[1],
                             MyMacAddr[2], MyMacAddr[3], MyMacAddr[4], MyMacAddr[5]);
                    serial_logger_usb_print(mac_dbg);
                    serial_logger_usb_println(" - add this to drone_org_registry_map");
                }
            } else {
                serial_logger_usb_print("drone_org_registry_map: ");
                serial_logger_usb_println(String(code).c_str());
            }
            continue;
        }

        // poll backend command state every 2s
        vTaskDelay(pdMS_TO_TICKS(kBackendPollIntervalMs - 1000));
        if (WiFi.status() != WL_CONNECTED) continue;

        BackendCommandState state;
        String body;
        int code = 0;
        if (fetch_backend_command_state(state, g_org_id, body, code)) {
            sync_backend_command_state(state);
            serial_logger_usb_print("backend body: ");
            serial_logger_usb_println(body.c_str());
            serial_logger_usb_print("backend flags sweep_now=");
            serial_logger_usb_print(state.sweep_now ? "true" : "false");
            serial_logger_usb_print(" force_scan=");
            serial_logger_usb_print(state.force_scan ? "true" : "false");
            serial_logger_usb_print(" sweep_intervals=");
            serial_logger_usb_print(state.sweep_intervals ? "true" : "false");
            serial_logger_usb_print(" temporary_disable_off=");
            serial_logger_usb_println(state.temporary_disable_off ? "true" : "false");
            apply_backend_command_state(state);
            post_drone_dashboard_state();
        } else {
            serial_logger_usb_print("backend command poll: ");
            serial_logger_usb_print(String(code).c_str());
            if (body.length() > 0) {
                serial_logger_usb_print(" ");
                serial_logger_usb_println(body.c_str());
            } else {
                serial_logger_usb_println("");
            }
        }
    }
}

uint8_t rc_backend_force_scan_active(void) {
    return g_backend_force_scan;
}

uint8_t rc_backend_sweep_now_active(void) {
    return g_backend_sweep_now;
}

uint8_t rc_backend_sweep_intervals_active(void) {
    return g_backend_sweep_intervals;
}

uint8_t rc_backend_temporary_disable_off_active(void) {
    return g_backend_temporary_disable_off;
}

static WiFiServer cameraServer(81);

static void camera_stream_task(void *param);

// Upstream camera (the device that actually produces MJPEG).
// Defaults to the common ESP32-CAM softAP IP, but can be changed at runtime via HTTP.
static IPAddress g_camera_upstream_ip(10, 135, 55, 152);
static uint16_t g_camera_upstream_port = 80;
static String g_camera_upstream_path   = "/api/v1/stream";

static bool parse_ip_or_host(const String &s, IPAddress &out) {
    // Accept dotted-quad IP or hostname (resolved via DNS)
    if (out.fromString(s)) return true;
    IPAddress resolved;
    if (WiFi.hostByName(s.c_str(), resolved)) {
        out = resolved;
        return true;
    }
    return false;
}

static void send_json(const String &json) {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Cache-Control", "no-cache");
    server.send(200, "application/json", json);
}

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

    // Allow overriding upstream per-request: /stream?up=1.2.3.4&port=80&path=/api/v1/stream
    // We strip these params from the forwarded query string so the camera endpoint doesn't see them.
    IPAddress upstream_ip = g_camera_upstream_ip;
    uint16_t upstream_port = g_camera_upstream_port;
    String upstream_path = g_camera_upstream_path;
    if (qs.length() > 1 && qs[0] == '?') {
        // very small/fast query parser
        auto getParam = [&](const char *key) -> String {
            String k = String(key) + "=";
            int i = qs.indexOf(k);
            if (i < 0) return String();
            int v0 = i + k.length();
            int v1 = qs.indexOf('&', v0);
            if (v1 < 0) v1 = qs.length();
            String v = qs.substring(v0, v1);
            v.replace("%2F", "/");
            v.replace("%3A", ":");
            return v;
        };
        String up = getParam("up");
        if (up.length() > 0) {
            IPAddress tmp;
            if (parse_ip_or_host(up, tmp)) upstream_ip = tmp;
        }
        String portS = getParam("port");
        if (portS.length() > 0) {
            int p = portS.toInt();
            if (p > 0 && p < 65536) upstream_port = (uint16_t)p;
        }
        String pathS = getParam("path");
        if (pathS.length() > 0) upstream_path = pathS;

        // Remove our control params from forwarded qs (best-effort; ok if we leave it)
        // This keeps the upstream camera API clean if it doesn't expect these params.
        // We keep any other params (e.g., resolution settings) intact.
        auto stripParam = [&](const char *key) {
            String k1 = String("?") + key + "=";
            String k2 = String("&") + key + "=";
            int i = qs.indexOf(k1);
            if (i < 0) i = qs.indexOf(k2);
            if (i < 0) return;
            int amp = qs.indexOf('&', i + 1);
            if (amp < 0) {
                // truncate to before this param
                qs = qs.substring(0, i);
            } else {
                qs = qs.substring(0, i) + qs.substring(amp);
                // if we removed the first param after '?', we may now have '?&' -> '?'
                qs.replace("?&", "?");
            }
            // if only '?' remains, drop it
            if (qs == "?") qs = "";
        };
        stripParam("up");
        stripParam("port");
        stripParam("path");
        // clean trailing '?' (if any)
        if (qs.endsWith("?")) qs = qs.substring(0, qs.length() - 1);
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
    if (!client_in.connect(upstream_ip, upstream_port)) {
        client_out.print(
            "HTTP/1.1 502 Bad Gateway\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\nupstream connect failed");
        client_out.stop();
        vTaskDelete(nullptr);
        return;
    }

    String upstream = upstream_path;
    upstream += qs;

    client_in.print("GET ");
    client_in.print(upstream);
    client_in.print(" HTTP/1.1\r\n");
    client_in.print("Host: ");
    client_in.print(upstream_ip.toString());
    client_in.print("\r\n");
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
static constexpr int RC_PACKET_SIZE = 25;
static constexpr uint8_t RC_BAD_PACKET_LIMIT = 3;
static volatile uint8_t g_bad_packet_count = 0;

// RC
volatile float Stick[16];
volatile uint8_t Recv_MAC[3];

void on_esp_now_sent(const uint8_t *mac_addr, esp_now_send_status_t status);

static void apply_bad_packet_failsafe() {
    Rc_err_flag = 1;
    Connect_flag = 40;
    PositionHold_flag = 0;

    Stick[RUDDER] = 0.0f;
    Stick[THROTTLE] = 0.0f;
    Stick[AILERON] = 0.0f;
    Stick[ELEVATOR] = 0.0f;
    Stick[BUTTON_ARM] = 0.0f;
    Stick[BUTTON_FLIP] = 0.0f;
    Stick[CONTROLMODE] = ANGLECONTROL;
    Stick[ALTCONTROLMODE] = AUTO_ALT;

    ahrs_reset_flag = 0;
}

// 受信コールバック
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *recv_data, int data_len) {
    uint8_t *d_int;
    // int16_t d_short;
    float d_float;

    if (data_len != RC_PACKET_SIZE) {
        if (g_bad_packet_count < RC_BAD_PACKET_LIMIT) g_bad_packet_count++;
        if (g_bad_packet_count >= RC_BAD_PACKET_LIMIT) apply_bad_packet_failsafe();
        return;
    }

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
        if (g_bad_packet_count < RC_BAD_PACKET_LIMIT) g_bad_packet_count++;
        if (g_bad_packet_count >= RC_BAD_PACKET_LIMIT) apply_bad_packet_failsafe();
        else Rc_err_flag = 1;
        return;
    }

    // checksum
    uint8_t check_sum = 0;
    for (uint8_t i = 0; i < 24; i++) check_sum = check_sum + recv_data[i];
    // if (check_sum!=recv_data[23])USBSerial.printf("checksum=%03d recv_sum=%03d\n\r", check_sum, recv_data[23]);
    if (check_sum != recv_data[24]) {
        if (g_bad_packet_count < RC_BAD_PACKET_LIMIT) g_bad_packet_count++;
        if (g_bad_packet_count >= RC_BAD_PACKET_LIMIT) apply_bad_packet_failsafe();
        else Rc_err_flag = 1;
        return;
    }

    g_bad_packet_count = 0;
    Rc_err_flag = 0;
    Connect_flag = 0;

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

    if (Position_estimate_valid == 0) {
        server.send(409, "text/plain", "position estimate unavailable");
        return;
    }

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
    char json[640];
    snprintf(json, sizeof(json),
             "{\"t_us\":%u,\"acc\":[%.6f,%.6f,%.6f],\"gyro\":[%.6f,%.6f,%.6f],\"rc_ok\":%s,"
             "\"flow_valid\":%u,\"pos_hold\":%u,\"flow_pos\":[%.3f,%.3f],\"flow_vel\":[%.3f,%.3f],"
             "\"target\":[%.3f,%.3f],\"mode\":%u,\"motor\":[%.3f,%.3f,%.3f,%.3f],"
             "\"mac\":\"%02x:%02x:%02x:%02x:%02x:%02x\"}",
             (uint32_t)micros(), acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, rc_isconnected() ? "true" : "false",
             (unsigned int)Position_estimate_valid, (unsigned int)PositionHold_flag, current_x, current_y, Vel_x,
             Vel_y, target_x, target_y, (unsigned int)Mode, FrontRight_motor_duty, FrontLeft_motor_duty,
             RearRight_motor_duty, RearLeft_motor_duty, MyMacAddr[0], MyMacAddr[1], MyMacAddr[2], MyMacAddr[3],
             MyMacAddr[4], MyMacAddr[5]);
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

    if (Position_estimate_valid == 0) {
        server.send(409, "text/plain", "position estimate unavailable");
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
    WiFi.begin("QU-Device", "1234567!");

    WiFi.softAPConfig(IPAddress(10, 0, 0, 1), IPAddress(10, 0, 0, 1), IPAddress(255, 255, 255, 0));
    WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);

    serial_logger_usb_println("Access Point started!");

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
        serial_logger_usb_print("STA IP: ");
        serial_logger_usb_println(WiFi.localIP().toString());
        serial_logger_usb_print("Gateway: ");
        serial_logger_usb_println(WiFi.gatewayIP().toString());
        serial_logger_usb_print("Subnet: ");
        serial_logger_usb_println(WiFi.subnetMask().toString());
        serial_logger_usb_print("DNS: ");
        serial_logger_usb_println(WiFi.dnsIP().toString());
        wifi_ap_record_t apinfo{};
        if (esp_wifi_sta_get_ap_info(&apinfo) == ESP_OK) {
            esp_wifi_set_channel(apinfo.primary, WIFI_SECOND_CHAN_NONE);
            serial_logger_usb_print("Synced WiFi channel to ");
            serial_logger_usb_println(String(apinfo.primary));
        }
    } else {
        serial_logger_usb_println("STA: no IP (not connected to QU-Device)");
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
        // Prefer the STA/LAN IP (QU-Device network) so you can view the stream without joining the drone AP.
        // Fallback to softAP IP when STA isn't connected.
        IPAddress ip = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP() : IPAddress(0, 0, 0, 0);
        if (ip == IPAddress(0, 0, 0, 0)) ip = WiFi.softAPIP();
        String loc = "http://" + ip.toString() + ":81/stream";

        // Preserve query string so callers can pass overrides like ?up=<camera_ip>
        if (server.args() > 0) {
            loc += "?";
            for (uint8_t i = 0; i < server.args(); i++) {
                if (i) loc += "&";
                loc += server.argName(i);
                loc += "=";
                loc += server.arg(i);
            }
        }
        server.sendHeader("Location", loc);
        server.send(302);
    });

    // Configure upstream camera (where the drone proxies MJPEG from)
    server.on("/camera/upstream", HTTP_GET, []() {
        String j = "{";
        j += "\"ip\":\"" + g_camera_upstream_ip.toString() + "\",";
        j += "\"port\":" + String((int)g_camera_upstream_port) + ",";
        j += "\"path\":\"" + g_camera_upstream_path + "\"";
        j += "}";
        send_json(j);
    });
    server.on("/camera/upstream", HTTP_POST, []() {
        server.sendHeader("Access-Control-Allow-Origin", "*");
        String ipS = server.hasArg("ip") ? server.arg("ip") : "";
        String portS = server.hasArg("port") ? server.arg("port") : "";
        String pathS = server.hasArg("path") ? server.arg("path") : "";

        if (ipS.length() > 0) {
            IPAddress tmp;
            if (parse_ip_or_host(ipS, tmp)) {
                g_camera_upstream_ip = tmp;
            } else {
                server.send(400, "text/plain", "bad ip/host");
                return;
            }
        }
        if (portS.length() > 0) {
            int p = portS.toInt();
            if (p <= 0 || p >= 65536) {
                server.send(400, "text/plain", "bad port");
                return;
            }
            g_camera_upstream_port = (uint16_t)p;
        }
        if (pathS.length() > 0) {
            if (!pathS.startsWith("/")) pathS = "/" + pathS;
            g_camera_upstream_path = pathS;
        }

        String j = "{";
        j += "\"ok\":true,";
        j += "\"ip\":\"" + g_camera_upstream_ip.toString() + "\",";
        j += "\"port\":" + String((int)g_camera_upstream_port) + ",";
        j += "\"path\":\"" + g_camera_upstream_path + "\"";
        j += "}";
        send_json(j);
    });
    server.on("/camera/upstream", HTTP_OPTIONS, handle_options);

    server.begin();

    // start webserver in separate task so it doesn't block control loop
    xTaskCreatePinnedToCore(webserver_task, "webserver", 8192, nullptr, 1, nullptr, 1);

    xTaskCreatePinnedToCore(supabase_registry_task, "supabase_registry", 8192, nullptr, 0, nullptr, 1);

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

int8_t rc_get_org_id(void) {
    return g_org_id;
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
