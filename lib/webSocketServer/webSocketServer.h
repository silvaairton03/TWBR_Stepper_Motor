#ifndef WEBSOCKET_SERVER_H
#define WEBSOCKET_SERVER_H

#include <ESPAsyncWebServer.h>

// Enum to represent control commands
enum CommandType {
  NONE,
  FORWARD,
  BACKWARD
};

// Declare external shared variable for command tracking
extern volatile CommandType pendingCommand;

// Declare the WebSocket instance
extern AsyncWebSocket ws;

// Function to initialize the WebSocket server and bind event handling
void initWebSocket(AsyncWebServer &server);

#endif

