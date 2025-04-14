// websocket.cpp
#include "webSocketServer.h"

volatile CommandType pendingCommand = NONE;
AsyncWebSocket ws("/ws");

void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                      AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.println("[WS] Client connected");
    client->text("Connected to ESP32 WebSocket");
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.println("[WS] Client disconnected");
  } else if (type == WS_EVT_DATA) {
    String msg;
    for (size_t i = 0; i < len; i++) msg += (char)data[i];
    msg.trim();

    Serial.printf("[WS] Received: %s\n", msg.c_str());

    if (msg == "forward") {
      pendingCommand = FORWARD;
    } else if (msg == "backward") {
      pendingCommand = BACKWARD;
    }

    client->text("Command queued: " + msg);
  }
}

void initWebSocket(AsyncWebServer &server) {
    ws.onEvent(onWebSocketEvent);
    server.addHandler(&ws);
  
    // Optional: debug unhandled HTTP requests
    server.onNotFound([](AsyncWebServerRequest *request) {
      Serial.printf("[404] URI: %s\n", request->url().c_str());
      request->send(404, "text/plain", "Not found");
    });
  }
