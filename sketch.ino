#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

#define MR_R 4
#define MR_Y 5
#define MR_G 6

#define RT_R 16
#define RT_Y 17
#define RT_G 18

#define LT_R 11
#define LT_Y 12
#define LT_G 13

#define PED_R 8
#define PED_G 9

#define BTN 10
#define BUZZ 19

#define SDA_PIN 41
#define SCL_PIN 40

const TickType_t T_GREEN = pdMS_TO_TICKS(20000);
const TickType_t T_YELLOW = pdMS_TO_TICKS(200);
const uint32_t T_PED_MS = 15000; 

QueueHandle_t eventQueue = nullptr;
QueueHandle_t pedQueue = nullptr;
SemaphoreHandle_t displayMutex = nullptr;
SemaphoreHandle_t stateMutex = nullptr;
SemaphoreHandle_t pedDone = nullptr;

// Event codes
enum Event_t : int {
  EVT_NONE = 0,
  EVT_PED_REQ = 1,
  EVT_PED_START = 2
};

// forward tasks
void trafficTask(void *pvParameters);
void pedTask(void *pvParameters);

// ----------------- Utility ---------------------
void setAllTrafficRed() {
  digitalWrite(MR_R, HIGH);
  digitalWrite(RT_R, HIGH);
  digitalWrite(LT_R, HIGH);

  digitalWrite(MR_Y, LOW);
  digitalWrite(RT_Y, LOW);
  digitalWrite(LT_Y, LOW);

  digitalWrite(MR_G, LOW);
  digitalWrite(RT_G, LOW);
  digitalWrite(LT_G, LOW);
}

void allOffTraffic() {
  digitalWrite(MR_R, LOW); digitalWrite(MR_Y, LOW); digitalWrite(MR_G, LOW);
  digitalWrite(RT_R, LOW); digitalWrite(RT_Y, LOW); digitalWrite(RT_G, LOW);
  digitalWrite(LT_R, LOW); digitalWrite(LT_Y, LOW); digitalWrite(LT_G, LOW);
}

// ----------------- ISR (debounced) --------------------------
static void IRAM_ATTR IRAM_isr_button() {
  static TickType_t lastTick = 0;
  TickType_t now = xTaskGetTickCountFromISR();
  // debounce: 200 ms
  if ((now - lastTick) < pdMS_TO_TICKS(200)) {
    return; // ignore bounce
  }
  lastTick = now;

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  int ev = EVT_PED_REQ;
  xQueueSendFromISR(eventQueue, &ev, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// -------------- Display helpers -----------------
void drawWalkingFrame(int frame) {
  int x = 90;
  int y = 36;
  display.fillCircle(x, y - 16, 6, SSD1306_WHITE);
  display.drawLine(x, y - 10, x, y + 6, SSD1306_WHITE);
  if (frame == 0) {
    display.drawLine(x, y + 6, x - 8, y + 18, SSD1306_WHITE);
    display.drawLine(x, y + 6, x + 6, y + 18, SSD1306_WHITE);
  } else {
    display.drawLine(x, y + 6, x - 4, y + 18, SSD1306_WHITE);
    display.drawLine(x, y + 6, x + 10, y + 18, SSD1306_WHITE);
  }
}

void displayWalkCountdown(int secondsLeft, int frame) {
  // caller must hold displayMutex (or be the only display writer)
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(2);
  display.setCursor(6, 4);
  display.println("Walk");

  display.setTextSize(3);
  display.setCursor(46, 22);
  display.print(secondsLeft);

  drawWalkingFrame(frame);
  display.display();
}

// --------------- Tasks --------------------------

// trafficTask pinned to core 0
void trafficTask(void *pvParameters) {
  (void) pvParameters;
  int evt;

  for (;;) {
    // ---- Main road phase ----
    allOffTraffic();
    digitalWrite(RT_R, HIGH);
    digitalWrite(LT_R, HIGH);

    digitalWrite(MR_R, LOW);
    digitalWrite(MR_Y, HIGH);
    vTaskDelay(T_YELLOW);

    digitalWrite(MR_Y, LOW);
    digitalWrite(MR_G, HIGH);
    vTaskDelay(T_GREEN);

    digitalWrite(MR_G, LOW);
    digitalWrite(MR_R, HIGH);

    // check & handle pedestrian requests (flush duplicates)
    if (xQueueReceive(eventQueue, &evt, 0) == pdPASS) {
      if (evt == EVT_PED_REQ) {
        // flush remaining PED_REQ events so we don't handle duplicates
        while (xQueueReceive(eventQueue, &evt, 0) == pdPASS) { /* discard */ }

        int startEvt = EVT_PED_START;
        xQueueSend(pedQueue, &startEvt, portMAX_DELAY);
        // wait for ped task to finish
        xSemaphoreTake(pedDone, portMAX_DELAY);
      }
    }

    // ---- Right turn phase ----
    allOffTraffic();
    digitalWrite(MR_R, HIGH);
    digitalWrite(LT_R, HIGH);

    digitalWrite(RT_R, LOW);
    digitalWrite(RT_Y, HIGH);
    vTaskDelay(T_YELLOW);

    digitalWrite(RT_Y, LOW);
    digitalWrite(RT_G, HIGH);
    vTaskDelay(T_GREEN);

    digitalWrite(RT_G, LOW);
    digitalWrite(RT_R, HIGH);

    if (xQueueReceive(eventQueue, &evt, 0) == pdPASS) {
      if (evt == EVT_PED_REQ) {
        while (xQueueReceive(eventQueue, &evt, 0) == pdPASS) { }
        int startEvt = EVT_PED_START;
        xQueueSend(pedQueue, &startEvt, portMAX_DELAY);
        xSemaphoreTake(pedDone, portMAX_DELAY);
      }
    }

    // ---- Left turn phase ----
    allOffTraffic();
    digitalWrite(MR_R, HIGH);
    digitalWrite(RT_R, HIGH);

    digitalWrite(LT_R, LOW);
    digitalWrite(LT_Y, HIGH);
    vTaskDelay(T_YELLOW);

    digitalWrite(LT_Y, LOW);
    digitalWrite(LT_G, HIGH);
    vTaskDelay(T_GREEN);

    digitalWrite(LT_G, LOW);
    digitalWrite(LT_R, HIGH);

    if (xQueueReceive(eventQueue, &evt, 0) == pdPASS) {
      if (evt == EVT_PED_REQ) {
        while (xQueueReceive(eventQueue, &evt, 0) == pdPASS) { }
        int startEvt = EVT_PED_START;
        xQueueSend(pedQueue, &startEvt, portMAX_DELAY);
        xSemaphoreTake(pedDone, portMAX_DELAY);
      }
    }
  }
}

// pedTask pinned to core 1
void pedTask(void *pvParameters) {
  (void) pvParameters;
  int evt;
  for (;;) {
    if (xQueueReceive(pedQueue, &evt, portMAX_DELAY) == pdPASS) {
      if (evt == EVT_PED_START) {
        // take display mutex (safe)
        if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(5000)) == pdTRUE) {
          // set all traffic RED
          setAllTrafficRed();

          // ped lights on
          digitalWrite(PED_R, LOW);
          digitalWrite(PED_G, HIGH);

          int totalSec = T_PED_MS / 1000;
          unsigned long start = millis();

          for (int s = totalSec; s >= 0; s--) {
            // buzzer pattern: short beep at start of each second
            tone(BUZZ, 3000, 150);
            displayWalkCountdown(s, s % 2);
            vTaskDelay(pdMS_TO_TICKS(1000));
          }

          noTone(BUZZ);

          // restore ped lights
          digitalWrite(PED_G, LOW);
          digitalWrite(PED_R, HIGH);

          display.clearDisplay();
          display.display();

          xSemaphoreGive(displayMutex);
        } else {
          // couldn't take display mutex, still ensure ped lights and buzzer run
          setAllTrafficRed();
          digitalWrite(PED_R, LOW);
          digitalWrite(PED_G, HIGH);
          int totalSec = T_PED_MS / 1000;
          for (int s = totalSec; s >= 0; s--) {
            tone(BUZZ, 3000, 150);
            vTaskDelay(pdMS_TO_TICKS(1000));
          }
          noTone(BUZZ);
          digitalWrite(PED_G, LOW);
          digitalWrite(PED_R, HIGH);
        }

        // signal traffic that ped is done
        xSemaphoreGive(pedDone);
      }
    }
  }
}

// ---------------- setup / main ------------------
void setup() {
  Serial.begin(115200);

  // pin modes
  pinMode(MR_R, OUTPUT); pinMode(MR_Y, OUTPUT); pinMode(MR_G, OUTPUT);
  pinMode(RT_R, OUTPUT); pinMode(RT_Y, OUTPUT); pinMode(RT_G, OUTPUT);
  pinMode(LT_R, OUTPUT); pinMode(LT_Y, OUTPUT); pinMode(LT_G, OUTPUT);
  pinMode(PED_R, OUTPUT); pinMode(PED_G, OUTPUT);

  pinMode(BUZZ, OUTPUT);
  // IMPORTANT: use INPUT_PULLUP and wire button between pin and GND
  pinMode(BTN, INPUT_PULLUP);

  // I2C & display
  Wire.begin(SDA_PIN, SCL_PIN);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 allocation failed");
  }
  display.clearDisplay();
  display.display();

  // initialize traffic to all red and pedestrian red
  setAllTrafficRed();
  digitalWrite(PED_R, HIGH);
  digitalWrite(PED_G, LOW);

  // create synchronization primitives
  eventQueue = xQueueCreate(8, sizeof(int));  // from ISR -> traffic
  pedQueue = xQueueCreate(4, sizeof(int));    // traffic -> ped
  displayMutex = xSemaphoreCreateMutex();
  stateMutex = xSemaphoreCreateMutex();
  pedDone = xSemaphoreCreateBinary();

  if (!eventQueue || !pedQueue || !displayMutex || !stateMutex || !pedDone) {
    Serial.println("Failed to create RTOS objects");
    while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
  }

  // attach ISR for button (falling edge - because INPUT_PULLUP wiring)
  attachInterrupt(digitalPinToInterrupt(BTN), IRAM_isr_button, FALLING);

  // Create tasks pinned to cores
  BaseType_t ok1 = xTaskCreatePinnedToCore(trafficTask, "Traffic", 4096, NULL, 2, NULL, 0);
  BaseType_t ok2 = xTaskCreatePinnedToCore(pedTask, "Ped", 4096, NULL, 2, NULL, 1);
  if (ok1 != pdPASS || ok2 != pdPASS) {
    Serial.println("Task creation failed");
    while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
  }
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
