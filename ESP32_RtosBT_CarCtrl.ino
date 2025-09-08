#include "BluetoothSerial.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// Debug flag
bool debug = true;
void debugPrintln(const String &msg) {
  if (debug) {
    Serial.println(msg);
  }
}

// Bluetooth
BluetoothSerial SerialBT;

// Motor Pins
const int IN1 = 12;
const int IN2 = 14;
const int IN3 = 27;
const int IN4 = 26;

// PWM Settings
const int freq = 500;
const int resolution = 8;

// Task Communication
QueueHandle_t commandQueue;

// Control Variables
int duration = 500; // milli seconds
bool motorRunning = false;
unsigned long motorStartTime = 0;

// Heartbeat timestamps for watchdog
volatile unsigned long lastBTHeartbeat = 0;
volatile unsigned long lastMotorHeartbeat = 0;
const unsigned long watchdogInterval = 2000; // 2 sec

// Stop all motors
void stopMotors() {
  debugPrintln("Auto Stop");
  ledcWrite(IN1, 0);
  ledcWrite(IN2, 0);
  ledcWrite(IN3, 0);
  ledcWrite(IN4, 0);
  motorRunning = false;
}

// Bluetooth Task (Core 1)
void vBluetoothTask(void *pvParameters) {
  String rx_data;
  char cmd;

  while (1) {
    lastBTHeartbeat = millis(); // heartbeat

    if (SerialBT.available()) {
      rx_data = SerialBT.readString();
      if (rx_data.length() > 0) {
        cmd = rx_data.charAt(0);
        debugPrintln("Received: " + String(cmd));
        xQueueSend(commandQueue, &cmd, portMAX_DELAY);
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// Motor Control Task (Core 2)
void vMotorTask(void *pvParameters) {
  char command;

  while (1) {
    lastMotorHeartbeat = millis(); // heartbeat

    // Check for new command
    if (xQueueReceive(commandQueue, &command, 10 / portTICK_PERIOD_MS) == pdPASS) {
      switch (command) {
        case 'F':
          debugPrintln("Forward");
          ledcWrite(IN1, 255);
          ledcWrite(IN2, 0);
          ledcWrite(IN3, 0);
          ledcWrite(IN4, 0);
          motorRunning = true;
          motorStartTime = millis();
          break;

        case 'B':
          debugPrintln("Backward");
          ledcWrite(IN1, 0);
          ledcWrite(IN2, 150);
          ledcWrite(IN3, 0);
          ledcWrite(IN4, 0);
          motorRunning = true;
          motorStartTime = millis();
          break;

        case 'L':
          debugPrintln("Left");
          ledcWrite(IN1, 150);
          ledcWrite(IN2, 0);
          ledcWrite(IN3, 255);
          ledcWrite(IN4, 0);
          motorRunning = true;
          motorStartTime = millis();
          break;

        case 'R':
          debugPrintln("Right");
          ledcWrite(IN1, 150);
          ledcWrite(IN2, 0);
          ledcWrite(IN3, 0);
          ledcWrite(IN4, 255);
          motorRunning = true;
          motorStartTime = millis();
          break;

        case 'S':
        default:
          debugPrintln("Manual Stop");
          stopMotors();
          break;
      }
    }

    // Auto stop after duration
    if (motorRunning && (millis() - motorStartTime >= duration)) {
      stopMotors();
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// Watchdog Task (Core 1, high priority)
void vWatchdogTask(void *pvParameters) {
  while (1) {
    unsigned long now = millis();

    if ((now - lastBTHeartbeat) > watchdogInterval) {
      debugPrintln("⚠️ Watchdog: Bluetooth task stalled!");
      stopMotors();
    }

    if ((now - lastMotorHeartbeat) > watchdogInterval) {
      debugPrintln("⚠️ Watchdog: Motor task stalled!");
      stopMotors();
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS); // check every second
  }
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ToyCar");
  debugPrintln("ToyCar Ready");

  // Motor PWM Setup
  ledcAttach(IN1, freq, resolution);
  ledcAttach(IN2, freq, resolution);
  ledcAttach(IN3, freq, resolution);
  ledcAttach(IN4, freq, resolution);

  // Create Command Queue
  commandQueue = xQueueCreate(5, sizeof(char));

  // Create Tasks
  xTaskCreatePinnedToCore(vBluetoothTask, "BluetoothTask", 4096, NULL, 1, NULL, 1); // Core 1
  xTaskCreatePinnedToCore(vMotorTask, "MotorTask", 4096, NULL, 2, NULL, 0);         // Core 2 (higher prio)
  xTaskCreatePinnedToCore(vWatchdogTask, "WatchdogTask", 4096, NULL, 3, NULL, 1);   // Core 1, highest prio
}

void loop() {
  // Not used in FreeRTOS
}
