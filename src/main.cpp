#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>

const int LED_PIN = 25; // Default built-in LED pin on Pico

// The FreeRTOS task function
void vBlinkTask(void *pvParameters) {
  pinMode(LED_PIN, OUTPUT);
  for (;;) {
    digitalWrite(LED_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(500)); // Delay for 500 ticks (milliseconds)
    digitalWrite(LED_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(500)); // Delay for 500 ticks (milliseconds)
  }
}

void setup() {
  Serial.begin(115200);

  // Create the blink task
  xTaskCreate(vBlinkTask,   // Task function
              "Blink Task", // Name of task
              256,          // Stack size (words)
              NULL,         // Parameter passed to task
              1,            // Priority
              NULL          // Task handle
  );
}

void loop() {
  // In FreeRTOS, the loop function is not strictly needed for task management.
  // The scheduler handles the tasks. You can leave it empty.
}
