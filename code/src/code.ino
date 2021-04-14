/*

Moteino Weather Station

Copyright (C) 2016 by Xose Pérez <xose dot perez at gmail dot com>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include "settings.h"
#include <RFM69Manager.h>
#include <LowPower.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SPIFlash.h>

// -----------------------------------------------------------------------------
// Globals
// -----------------------------------------------------------------------------

RFM69Manager radio;
#if HAS_FLASH
    SPIFlash flash(FLASH_SS, 0xEF30);
#endif
Adafruit_BME280 bme;

// -----------------------------------------------------------------------------
// Hardware
// -----------------------------------------------------------------------------

void blink(byte times, byte mseconds) {
    pinMode(LED_PIN, OUTPUT);
    for (byte i=0; i<times; i++) {
        if (i>0) delay(mseconds);
        digitalWrite(LED_PIN, HIGH);
        delay(mseconds);
        digitalWrite(LED_PIN, LOW);
    }
}

void hardwareSetup() {
    Serial.begin(SERIAL_BAUD);
    pinMode(LED_PIN, OUTPUT);
    pinMode(BATTERY_PIN, INPUT);
    #if USE_MOSFET
        pinMode(BATTERY_ENABLE_PIN, INPUT);
    #endif
    delay(1);
}

// -----------------------------------------------------------------------------
// BME280
// -----------------------------------------------------------------------------

#ifdef DEBUG
unsigned char i2cScan() {

    Wire.begin();

    unsigned char nDevices = 0;
    for (unsigned char address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        unsigned char error = Wire.endTransmission();
        if (error == 0) {
            Serial.print("[I2C] Device found at address 0x");
            Serial.println(address, HEX);
            nDevices++;
        }
    }

    if (nDevices == 0) Serial.println("[I2C] No devices found\n");

    return nDevices;

}
#endif

// -----------------------------------------------------------------------------
// BME280
// -----------------------------------------------------------------------------

void bmeSetup() {

    #ifdef DEBUG
    //i2cScan();
    #endif

    // Make sure sensor had enough time to turn on. BME280 requires 2ms to start up
    delay(10);

    bool status = bme.begin(BME280_ADDRESS);  
    #ifdef DEBUG
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while (1) delay(10);
    }
    #endif

}

// -----------------------------------------------------------------------------
// RFM69
// -----------------------------------------------------------------------------

void radioSetup() {
    radio.initialize(FREQUENCY, NODEID, NETWORKID, ENCRYPTKEY, GATEWAYID, ATC_RSSI);
    radio.sleep();
}

// -----------------------------------------------------------------------------
// Flash
// -----------------------------------------------------------------------------

#if HAS_FLASH
void flashSetup() {
    if (flash.initialize()) {
        flash.sleep();
    }
}
#endif

// -----------------------------------------------------------------------------
// Messages
// -----------------------------------------------------------------------------

void sendSensor() {

    char buffer[8];

    // Temperature
    #if SEND_TEMPERATURE
        dtostrf(bme.readTemperature(), 4, 1, buffer);
        radio.send((char *) "TMP", buffer, (uint8_t) 2);
    #endif

    // Humidity
    #if SEND_HUMIDITY
        itoa((int) bme.readHumidity(), buffer, 10);
        radio.send((char *) "HUM", buffer, (uint8_t) 2);
    #endif

    // Pressure
    #if SEND_PRESSURE
        dtostrf(bme.readPressure() / 100.0F, 6, 1, buffer);
        radio.send((char *) "PRS", buffer, (uint8_t) 2);
    #endif

}

void sendBattery() {

    unsigned int voltage;

    // LowPowerLabs WeatherShield v2 can use p-mosfet to enable
    // power monitoring, but it ships without this feature. If you
    // add the required components (a p-mosfet plus 2 resistors)
    // change the USE_MOSFET to 1 in the settings.h file
    #if USE_MOSFET
        pinMode(BATTERY_ENABLE_PIN, OUTPUT);
        digitalWrite(BATTERY_ENABLE_PIN, LOW);
    #endif
    voltage = analogRead(BATTERY_PIN);
    #if USE_MOSFET
        pinMode(BATTERY_ENABLE_PIN, INPUT);
    #endif

    // Map analog reading to VIN value
    voltage = BATTERY_RATIO * voltage;

    char buffer[6];
    sprintf(buffer, "%d", voltage);
    radio.send((char *) "BAT", buffer, (uint8_t) 2);

}

void send()  {

    // Send current sensor readings
    sendSensor();

    // Send battery status once every 10 messages, starting with the first one
    #if SEND_BATTERY
        static unsigned char batteryCountdown = 0;
        if (batteryCountdown == 0) sendBattery();
        batteryCountdown = (batteryCountdown + 1) % 10;
    #endif

    // Radio back to sleep
    radio.sleep();

    // Show visual notification
    blink(1, NOTIFICATION_TIME);

}

// -----------------------------------------------------------------------------
// Common methods
// -----------------------------------------------------------------------------

void setup() {
    hardwareSetup();
    bmeSetup();
    #if HAS_FLASH
        flashSetup();
    #endif
    radioSetup();
}

void loop() {

    // We got here for three possible reasons:
    // - it's the first time (so we report status and battery)
    // - after ~300 seconds (we report status and maybe battery)
    send();

    // Sleep loop
    for (byte i = 0; i < SLEEP_COUNT; i++) {

        // Sleep for 8 seconds (the maximum the WDT accepts)
        LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);

    }

}
