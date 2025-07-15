/**
 * @file      GPS_Acceleration.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2024 Shenzhen Xin Yuan Electronic Technology Co., Ltd
 * @date      2024-09-09
 * @note      
 *            GPS acceleration only supports A7670X/A7608X (excluding A7670G and other versions that do not support positioning). 
 *            SIM7670G does not support GPS acceleration function
 */

#include "utilities.h"
#define TINY_GSM_MODEM_SIM7600
#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial

// Define the serial console for debug prints, if needed
#define TINY_GSM_DEBUG SerialMon

#include <TinyGsmClient.h>

TinyGsm modem(SerialAT);

// Define modem pins (ajuste conforme necessário)
#define MODEM_RX_PIN 16 // Exemplo
#define MODEM_TX_PIN 17 // Exemplo
#define BOARD_POWERON_PIN 5 // Exemplo
#define MODEM_RESET_PIN 4 // Exemplo
#define BOARD_PWRKEY_PIN 23 // Exemplo

void setup()
{
    Serial.begin(115200);
    // Turn on DC boost to power on the modem
    pinMode(BOARD_POWERON_PIN, OUTPUT);
    digitalWrite(BOARD_POWERON_PIN, HIGH);

    // Set modem reset pin, reset modem
    pinMode(MODEM_RESET_PIN, OUTPUT);
    digitalWrite(MODEM_RESET_PIN, LOW); delay(100);
    digitalWrite(MODEM_RESET_PIN, HIGH); delay(2600);
    digitalWrite(MODEM_RESET_PIN, LOW);

    // Turn on modem
    pinMode(BOARD_PWRKEY_PIN, OUTPUT);
    digitalWrite(BOARD_PWRKEY_PIN, LOW);
    delay(100);
    digitalWrite(BOARD_PWRKEY_PIN, HIGH);
    delay(1000);
    digitalWrite(BOARD_PWRKEY_PIN, LOW);

    // Set modem baud
    SerialAT.begin(115200, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);

    Serial.println("Start modem...");
    delay(3000);

    int retry = 0;
    while (!modem.testAT(1000)) {
        Serial.println("Testing modem...");
        if (retry++ > 10) {
            digitalWrite(BOARD_PWRKEY_PIN, LOW);
            delay(100);
            digitalWrite(BOARD_PWRKEY_PIN, HIGH);
            delay(1000);
            digitalWrite(BOARD_PWRKEY_PIN, LOW);
            retry = 0;
        }
    }
    Serial.println("Modem ready");
    delay(200);

    // Check if SIM card is online
    SimStatus sim = SIM_ERROR;
    while (sim != SIM_READY) {
        sim = modem.getSimStatus();
        switch (sim) {
            case SIM_READY:
                Serial.println("SIM card online");
                break;
            case SIM_LOCKED:
                Serial.println("The SIM card is locked. Please unlock the SIM card first.");
                break;
            default:
                break;
        }
        delay(1000);
    }

    // Configuração automática de rede
    modem.sendAT("+CREG?");
    modem.waitResponse();

    // Check network registration status
    Serial.print("Wait for the modem to register with the network.");
    RegStatus status = REG_NO_RESULT;
    while (status == REG_NO_RESULT || status == REG_SEARCHING || status == REG_UNREGISTERED) {
        status = modem.getRegistrationStatus();
        switch (status) {
            case REG_UNREGISTERED:
            case REG_SEARCHING:
                Serial.printf("Signal Quality: %d\n", modem.getSignalQuality());
                delay(1000);
                break;
            case REG_OK_HOME:
            case REG_OK_ROAMING:
                Serial.println("Online registration successful");
                break;
            default:
                Serial.printf("Registration Status: %d\n", status);
                delay(1000);
                break;
        }
    }
    Serial.println();

    // Enable GPS
    Serial.println("Enabling GPS/GNSS");
    while (!modem.enableGPS(MODEM_GPS_ENABLE_GPIO)) {
        Serial.print(".");
    }
    Serial.println("GPS Enabled");

    // Enable GPS acceleration
    Serial.println("GPS acceleration is enabled");
    if (!modem.enableAGPS()) {
        Serial.println("GPS acceleration failed!!!");
    } else {
        Serial.println("GPS acceleration success!!!");
    }
}

void loop()
{
    float lat2 = 0;
    float lon2 = 0;
    float speed2 = 0;
    float alt2 = 0;
    int vsat2 = 0;
    int usat2 = 0;
    float accuracy2 = 0;
    int year2 = 0;
    int month2 = 0;
    int day2 = 0;
    int hour2 = 0;
    int min2 = 0;
    int sec2 = 0;
    uint8_t fixMode = 0;

    Serial.println("Requesting current GPS/GNSS location");
    if (modem.getGPS(&fixMode, &lat2, &lon2, &speed2, &alt2, &vsat2, &usat2, &accuracy2,
                     &year2, &month2, &day2, &hour2, &min2, &sec2)) {
        Serial.print("FixMode: "); Serial.println(fixMode);
        Serial.print("Latitude: "); Serial.print(lat2, 6); Serial.print("\tLongitude: "); Serial.println(lon2, 6);
        Serial.print("Speed: "); Serial.print(speed2); Serial.print("\tAltitude: "); Serial.println(alt2);
        Serial.print("Visible Satellites: "); Serial.print(vsat2); Serial.print("\tUsed Satellites: "); Serial.println(usat2);
        Serial.print("Accuracy: "); Serial.println(accuracy2);
        Serial.print("Date: "); Serial.print(year2); Serial.print("/"); Serial.print(month2); Serial.print("/"); Serial.print(day2);
        Serial.print(" Time: "); Serial.print(hour2); Serial.print(":"); Serial.print(min2); Serial.print(":"); Serial.println(sec2);
    } else {
        Serial.println("Couldn't get GPS location, retrying in 15s.");
        delay(15000UL);
    }

    Serial.println("Disabling GPS");
    modem.disableGPS();

    while (1) {
        if (SerialAT.available()) {
            Serial.write(SerialAT.read());
        }
        if (Serial.available()) {
            SerialAT.write(Serial.read());
        }
        delay(1);
    }
}
