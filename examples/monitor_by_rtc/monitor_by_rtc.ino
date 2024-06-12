/*
 * RTC_PeriodicExample
 *
 * This example demonstrates how to use the periodic callback functionality of the RTC
 * (Real Time Clock) on the Portenta C33.
 *
 * It blinks the built-in LED at progressively faster and slower rates repeatedly.
 *
 * Find the full UNO R4 WiFi RTC documentation here:
 * https://docs.arduino.cc/tutorials/uno-r4-wifi/rtc
 */

// Include the RTC library

#include "RTC.h"
#include "PeripheralMonitor.h"

#undef  DEBUG
#define DEBUG   0

static PeripheralMonitor monitor;
static bool flag;

// This is the callback function to be passed to RTC.setPeriodicCallback()
void periodicCallback() {
  static bool ledState = false;
  flag = true;

  if (ledState == true) {
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else {
    digitalWrite(LED_BUILTIN, LOW);
  }
  ledState = !ledState;
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  while (!Serial);

#ifdef  ARDUINO_UNOR4_WIFI
  // UNO R4 WiFi needs to wait for a while to complete Serial initialization.
  delay(1000); // It requires at least 600 ms.
#endif

  /*
   * PERIPHERAL_PORTS, // PORT0 〜 PORT9 (default)
   * PERIPHERAL_PORT,  // PORT0 〜 PORT9
   * PERIPHERAL_PFS,   // PmnPFS (P000 〜 P915)
   * PERIPHERAL_PINS,  // D0 〜 D19 (A0 〜 A5)
   * PERIPHERAL_AGT,   // AGT0 〜 AGT1
   */
  monitor.begin(230400, PERIPHERAL_PFS, digitalPinToPmn(LED_BUILTIN)); // Baud rate: Mac 230400, Windows 921600

  // Initialize the RTC
  RTC.begin();

  // RTC.setTime() must be called for RTC.setPeriodicCallback to work, but it doesn't matter
  // what date and time it's set to
  RTCTime mytime(1, Month::JUNE, 2024, 0, 0, 0, DayOfWeek::THURSDAY, SaveLight::SAVING_TIME_ACTIVE);
  RTC.setTime(mytime);

  // Recommend: N4_TIMES_EVERY_SEC, N8_TIMES_EVERY_SEC or N16_TIMES_EVERY_SEC
  RTC.setPeriodicCallback(periodicCallback, Period::N8_TIMES_EVERY_SEC);
}

void loop() {
  if (flag) {
    flag = false;
    monitor.scan_command();
  
#if DEBUG
    uint32_t start = micros();
#endif

    monitor.show_register();

#if DEBUG
    Serial.println("time: " + String(micros() - start));
#endif
  }
}
