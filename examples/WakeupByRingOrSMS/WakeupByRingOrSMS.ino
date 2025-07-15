/**
 * @file      WakeupByRingOrSMS.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2025  ShenZhen XinYuan Electronic Technology Co., Ltd
 * @date      2025-04-29
 * @note      Only applicable to A7670X, A7608X series,SIM7600 series modules,
 *            SIM7670G - `SIM7670G-MNGV 2374B04` version supports SMS function,
 *            SIM7000G - RI pin not connected to esp ,this examples can't run
 *            but it requires the operator base station to support SMS Over SGS service to send,
 *            otherwise it will be invalid
 */


#include "utilities.h"
#include <driver/gpio.h>

#ifdef LILYGO_SIM7000G
#error "SIM7000G - RI pin not connected to esp ,this examples can't run"
#endif

#define TINY_GSM_RX_BUFFER          1024 // Set RX buffer to 1Kb

// See all AT commands, if wanted
#define DUMP_AT_COMMANDS

#include <TinyGsmClient.h>

#define uS_TO_S_FACTOR      1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP       30          /* Time ESP32 will go to sleep (in seconds) */


#ifdef DUMP_AT_COMMANDS  // if enabled it requires the streamDebugger lib
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, Serial);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif
void print_wakeup_reason()
{
    esp_sleep_wakeup_cause_t wakeup_reason;

    wakeup_reason = esp_sleep_get_wakeup_cause();

    switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
    }
}

void setup()
{
    Serial.begin(115200); // Set console baud rate

    SerialAT.begin(115200, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);

#ifdef BOARD_POWERON_PIN
    /* Set Power control pin output
    * * @note      Known issues, ESP32 (V1.2) version of T-A7670, T-A7608,
    *            when using battery power supply mode, BOARD_POWERON_PIN (IO12) must be set to high level after esp32 starts, otherwise a reset will occur.
    * */
    pinMode(BOARD_POWERON_PIN, OUTPUT);
    digitalWrite(BOARD_POWERON_PIN, HIGH);
#endif

    print_wakeup_reason();

    if (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_EXT0) {

#ifdef MODEM_RESET_PIN
        // Release reset GPIO hold
        gpio_hold_dis((gpio_num_t)MODEM_RESET_PIN);

        // Set modem reset pin ,reset modem
        // The module will also be started during reset.
        Serial.println("Set Reset Pin.");
        pinMode(MODEM_RESET_PIN, OUTPUT);
        digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL); delay(100);
        digitalWrite(MODEM_RESET_PIN, MODEM_RESET_LEVEL); delay(2600);
        digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL);
#endif

#ifdef MODEM_FLIGHT_PIN
        // If there is an airplane mode control, you need to exit airplane mode
        pinMode(MODEM_FLIGHT_PIN, OUTPUT);
        digitalWrite(MODEM_FLIGHT_PIN, HIGH);
#endif

        // Pull down DTR to ensure the modem is not in sleep state
        pinMode(MODEM_DTR_PIN, OUTPUT);
        digitalWrite(MODEM_DTR_PIN, LOW);

        // Turn on the modem
        pinMode(BOARD_PWRKEY_PIN, OUTPUT);
        digitalWrite(BOARD_PWRKEY_PIN, LOW);
        delay(100);
        digitalWrite(BOARD_PWRKEY_PIN, HIGH);
        //Ton >= 100 <= 500
        delay(100);
        digitalWrite(BOARD_PWRKEY_PIN, LOW);


    } else {
        Serial.println("Released modem hold pin!");

        // Need to cancel GPIO hold if wake from sleep
        gpio_hold_dis((gpio_num_t )MODEM_DTR_PIN);

#ifdef BOARD_POWERON_PIN
        // Need to cancel GPIO hold if wake from sleep
        gpio_hold_dis((gpio_num_t )BOARD_POWERON_PIN);
#endif

#ifdef MODEM_FLIGHT_PIN
        // Need to cancel GPIO hold if wake from sleep
        gpio_hold_dis((gpio_num_t )MODEM_FLIGHT_PIN);
#endif
    }

    Serial.println("Check modem online .");
    while (!modem.testAT()) {
        Serial.print("."); delay(500);
    }
    Serial.println("Modem is online !");

    delay(5000);

    if (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_EXT0) {
        // Wait PB DONE
        Serial.println("Wait SMS Done.");
        if (!modem.waitResponse(100000UL, "SMS DONE")) {
            Serial.println("Can't wait from sms register ....");
            return;
        }
        Serial.println("Modem has registered to network");
        delay(1000);
    }


#ifdef MODEM_FLIGHT_PIN
    // Hold on flight pin
    gpio_hold_en((gpio_num_t )MODEM_FLIGHT_PIN);
    gpio_deep_sleep_hold_en();
#endif

#ifdef BOARD_POWERON_PIN
    Serial.println("Set modem power pin to hold on");
    // Hold on power pin
    gpio_hold_en((gpio_num_t )BOARD_POWERON_PIN);
    gpio_deep_sleep_hold_en();
#endif

#ifdef MODEM_RESET_PIN
    // Keep it low during the sleep period. If the module uses GPIO5 as reset,
    // there will be a pulse when waking up from sleep that will cause the module to start directly.
    // https://github.com/Xinyuan-LilyGO/LilyGO-T-A76XX/issues/85
    Serial.println("Set modem reset pin to hold on");
    digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL);
    gpio_hold_en((gpio_num_t)MODEM_RESET_PIN);
    gpio_deep_sleep_hold_en();

#endif

    pinMode(MODEM_RING_PIN, INPUT_PULLUP);
    gpio_pullup_en((gpio_num_t)MODEM_RING_PIN);

    Serial.println("Ring coming...wait ring stop...");
    while (digitalRead(MODEM_RING_PIN) == LOW) {
        Serial.println("Wait rinng to stop..."); delay(5000);
    }

    Serial.println("Now your can send message or call this modem phone number wake up ESP32");
    Serial.println("Enter esp32 goto deepsleep!");
    Serial.flush();
    delay(1000);
    esp_sleep_enable_ext0_wakeup(((gpio_num_t)MODEM_RING_PIN), 0);
    delay(200);
    esp_deep_sleep_start();
    Serial.println("This will never be printed");
}

void loop()
{
}

#ifndef TINY_GSM_FORK_LIBRARY
#error "No correct definition detected, Please copy all the [lib directories](https://github.com/Xinyuan-LilyGO/LilyGO-T-A76XX/tree/main/lib) to the arduino libraries directory , See README"
#endif
