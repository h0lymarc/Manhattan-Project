#include "Wire.h"
#include "SPI.h"
#include "TFT_eSPI.h"
#include "AS726X.h"
#include "Free_Fonts.h"
#include "random_forest_model.h"
#include <vector>
#include <String>

#define LINE_HEIGHT 19
#define MAX_LINES 11
#define I2C_SDA 5
#define I2C_SCL 6
#define ADC_MAX 4095.0
#define VREF    3.3
#define BATT_PERCENTAGE 7
#define BATT_INDICATOR 38
#define TRIG_BUTTON 4

const int BAR_X[] = {291, 299, 307};
const int BAR_Y = 4;
const int BAR_W = 6;
const int BAR_H = 15;
const int BAR_COUNT = sizeof(BAR_X) / sizeof(BAR_X[0]);
int current_bar = 0;
int last_button_state = 1;
unsigned long lastTime = 0;
const unsigned long interval = 1000;
bool pressed = false;
float totalReadings = 0;

using Eloquent::ML::Port::RandomForest;
RandomForest rf;

AS726X sensor;
TFT_eSPI tft = TFT_eSPI();
TwoWire I2CSensor = TwoWire(0);
std::vector<String> log_buffer;

void read_battery() {
    int raw = analogRead(BATT_PERCENTAGE);
    float v_adc = (raw / ADC_MAX) * VREF;
    float voltage = v_adc / 0.680;    

    int chIndicator = digitalRead(BATT_INDICATOR);

    if (chIndicator == 0) {
        for (int i = 0; i < BAR_COUNT; i++) {
            tft.fillRect(BAR_X[i], BAR_Y, BAR_W, BAR_H, TFT_BLACK);
            tft.drawRect(BAR_X[i], BAR_Y, BAR_W, BAR_H, TFT_WHITE);
        }

        for (int i = 0; i <= current_bar; i++) {
            tft.fillRect(BAR_X[i], BAR_Y, BAR_W, BAR_H, TFT_GREEN);
        }

        current_bar++;
        if (current_bar >= BAR_COUNT) {
            current_bar = 0;
        }
    }

    else {
        if (voltage < 3.5) {
            tft.fillRect(291, 4, 6, 15, TFT_RED);

            tft.fillRect(299, 4, 6, 15, TFT_BLACK);
            tft.drawRect(299, 4, 6, 15, TFT_WHITE);

            tft.fillRect(307, 4, 6, 15, TFT_BLACK);
            tft.drawRect(307, 4, 6, 15, TFT_WHITE);
        }
        else if (voltage < 3.7) {
            tft.fillRect(291, 4, 6, 15, TFT_YELLOW);
            tft.fillRect(299, 4, 6, 15, TFT_YELLOW);

            tft.fillRect(307, 4, 6, 15, TFT_BLACK);
            tft.drawRect(307, 4, 6, 15, TFT_WHITE);
        }
        else {
            tft.fillRect(291, 4, 6, 15, TFT_GREEN);
            tft.fillRect(299, 4, 6, 15, TFT_GREEN);
            tft.fillRect(307, 4, 6, 15, TFT_GREEN);
        }
    }
}

void append_text(const String& log, bool reset = false) {
    if (reset) {
        tft.fillRect(0, 20, 320, 220, TFT_BLACK);
        log_buffer.clear();
        return;
    }

    int start = 0;
    int end = 0;

    while ((end = log.indexOf('\n', start)) != -1) {
        log_buffer.push_back(log.substring(start, end));
        start = end + 1;
    }

    if (start < log.length()) {
        log_buffer.push_back(log.substring(start));
    }

    while (log_buffer.size() > MAX_LINES) {
        log_buffer.erase(log_buffer.begin());
    }

    tft.fillRect(0, 20, 320, 220, TFT_BLACK);

    for (size_t i = 0; i < log_buffer.size(); i++) {
        int y = (i + 2) * LINE_HEIGHT;
        tft.setCursor(5, y);
        tft.setFreeFont(FF17);     
        tft.setTextColor(TFT_WHITE);
        tft.print(log_buffer[i]);
    }
}

void measurement() {
  sensor.takeMeasurementsWithBulb();
  float R = sensor.getCalibratedR();
  float S = sensor.getCalibratedS();
  float T = sensor.getCalibratedT();
  float U = sensor.getCalibratedU();
  float V = sensor.getCalibratedV();
  float W = sensor.getCalibratedW();

  float totalReadings = R + S + T + U + V + W;
  if (totalReadings > 0) {
    R /= totalReadings;
    S /= totalReadings;
    T /= totalReadings;
    U /= totalReadings;
    V /= totalReadings;
    W /= totalReadings;
  }

  float features[6] = {R, S, T, U, V, W};
  int classLabel = rf.predict(features);
  if (classLabel == 0)
     append_text("Soil nutrient status: deficient\n\n");
  else
     append_text("Soil nutrient status: sufficient\n\n");
}

void setup() {
  Serial.begin(115200);
  I2CSensor.begin(I2C_SDA, I2C_SCL, 100000);

  pinMode(BATT_INDICATOR, INPUT);
  pinMode(TRIG_BUTTON, INPUT);

  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);

  append_text("system boot");
  append_text("Initializing sensor");

  if (!sensor.begin(I2CSensor)) {
    append_text("rst", true);
    append_text("Sensor not found");
    return;
  }

  sensor.setIntegrationTime(50);
  sensor.setGain(64);
  
  delay(5000);
  append_text("Found sensor connected");
  delay(3000);
  append_text("rst", true);
  append_text("No task is running");
}

void loop() {
  if (millis() - lastTime >= interval) {
    lastTime = millis();
    read_battery();
  }

  int trigger = digitalRead(TRIG_BUTTON);
  if (last_button_state == 1 && trigger == 0) {
    pressed = !pressed;

    append_text(pressed ? "starting sensor" : "stopped");
    delay(500);
  }

  if (pressed) {
    measurement();
  }
}