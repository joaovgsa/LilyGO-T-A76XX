#define TINY_GSM_MODEM_SIM7600
#include "utilities.h"

#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb

#define SerialMon Serial
#define TINY_GSM_DEBUG SerialMon

#include <TinyGsmClient.h>

TinyGsm modem(SerialAT);

void setup()
{
    Serial.begin(115200);
    
    // Set modem reset pin and power on the modem
    pinMode(MODEM_RESET_PIN, OUTPUT);
    digitalWrite(MODEM_RESET_PIN, LOW);
    delay(100);
    digitalWrite(MODEM_RESET_PIN, HIGH);
    delay(2000);

    pinMode(BOARD_PWRKEY_PIN, OUTPUT);
    digitalWrite(BOARD_PWRKEY_PIN, LOW);
    delay(100);
    digitalWrite(BOARD_PWRKEY_PIN, HIGH);
    delay(1000);
    digitalWrite(BOARD_PWRKEY_PIN, LOW);

    // Set modem baud rate
    SerialAT.begin(115200, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
    Serial.println("Starting modem...");
    delay(3000);

    // Test modem connection
    for (int i = 0; i < 10; i++) {
        if (modem.testAT(1000)) {
            Serial.println("Modem connected successfully.");
            break;
        } else {
            Serial.print("Attempt "); Serial.print(i + 1); Serial.println(": Failed to connect.");
            delay(1000);
        }
    }

    // Enable GPS
    Serial.println("Enabling GPS...");
    if (!modem.enableGPS()) {
        Serial.println("Failed to enable GPS.");
        return;
    }
    Serial.println("GPS Enabled");
}

void loop()
{
    float lat = 0, lon = 0, speed = 0, alt = 0;
    int vsat = 0, usat = 0;
    float accuracy = 0;
    int year = 0, month = 0, day = 0, hour = 0, minute = 0, second = 0;

    Serial.println("Requesting GPS location...");
    if (modem.getGPS(&lat, &lon, &speed, &alt, &vsat, &usat, &accuracy,
                     &year, &month, &day, &hour, &minute, &second)) {

        Serial.print("Latitude: "); Serial.print(lat, 6); 
        Serial.print(" Longitude: "); Serial.println(lon, 6);
        Serial.print("Speed: "); Serial.print(speed); 
        Serial.print(" Altitude: "); Serial.println(alt);
        Serial.print("Visible Satellites: "); Serial.print(vsat); 
        Serial.print(" Used Satellites: "); Serial.println(usat);
        Serial.print("Accuracy: "); Serial.println(accuracy);
        Serial.print("Date: "); Serial.print(year); 
        Serial.print("-"); Serial.print(month); 
        Serial.print("-"); Serial.print(day);
        Serial.print(" Time: "); Serial.print(hour); 
        Serial.print(":"); Serial.print(minute); 
        Serial.print(":"); Serial.println(second);
    } else {
        Serial.println("Couldn't get GPS location.");
    }

    delay(15000); // Wait for 15 seconds before next request
}
