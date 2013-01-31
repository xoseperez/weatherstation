/*

  Outdoor weather station
  Copyright (C) 2013 by Xose Pérez <xose dot perez at gmail dot com>

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

#include <LowPower.h>
#include <DHT22.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>

// DHT22 connections:
// Connect pin 1 (on the left) of the sensor to 3.3V
// Connect pin 2 of the sensor to whatever your DHT_PIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

// BMP085 connections:
// Connect VCC of the sensor to 3.3V (NOT 5.0V!)
// Connect GND to Ground
// Connect SCL to i2c clock - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 5
// Connect SDA to i2c data - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 4
// EOC is not used, it signifies an end of conversion
// XCLR is a reset pin, also not used here

// ===========================================
// Configuration
// ===========================================

#define BATT_PIN 0
#define PANEL_PIN 1
#define XBEE_SLEEP_PIN 4
#define DHT_PIN 12

#define DHT_TYPE DHT22
#define VOLTAGE_REFERENCE 1100
#define BATT_VOLTAGE_FACTOR 4.06
#define PANEL_VOLTAGE_FACTOR 4.06
#define XBEE_DELAY 20

#define SLEEP_INTERVAL SLEEP_2S
#define WARMUP_DELAY 2000
#define MEASURE_EVERY 1
#define SEND_EVERY 5

struct measure {
    int count;
    float sum;
    float minimum;
    float maximum;
};

// ===========================================
// Globals
// ===========================================

DHT22 dht(DHT_PIN);
Adafruit_BMP085 bmp;

boolean bmp_ready = false;
unsigned long interval = 0;

struct measure dht22_temperature = {0,0,0,0};
struct measure dht22_humidity = {0,0,0,0};
struct measure bmp085_temperature = {0,0,0,0};
struct measure bmp085_pressure = {0,0,0,0};
struct measure fio_temperature = {0,0,0,0};
struct measure fio_voltage = {0,0,0,0};

// ===========================================
// Methods
// ===========================================

void xbeeSleep() {
    delay(XBEE_DELAY);
    digitalWrite(XBEE_SLEEP_PIN, HIGH);
}

void xbeeWake() {
    digitalWrite(XBEE_SLEEP_PIN, LOW);
    delay(XBEE_DELAY);
}

long readVoltage(byte pin, float factor) {
    int reading = analogRead(pin);
    return (long) map(reading, 0, 1023, 0, VOLTAGE_REFERENCE) * factor;
}

long readVoltage() {
    long result;
    // Read 1.1V reference against AVcc
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    delay(2); // Wait for Vref to settle
    ADCSRA |= _BV(ADSC); // Convert
    while (bit_is_set(ADCSRA,ADSC));
    result = ADCL;
    result |= ADCH<<8;
    result = 1126400L / result; // Back-calculate AVcc in mV
    return result;
}

float readTemperature() {
    signed long resultTemp;
    // Read temperature sensor against 1.1V reference
    ADMUX = _BV(REFS1) | _BV(REFS0) | _BV(MUX3);
    delay(20);                           // Wait for Vref to settle
    ADCSRA |= _BV(ADSC);                 // Convert
    while (bit_is_set(ADCSRA,ADSC));
    resultTemp = ADCL;
    resultTemp |= ADCH<<8;
    return (float) resultTemp;
}

// dewPoint function NOAA
// reference: http://wahiduddin.net/calc/density_algorithms.htm 
float dewPoint(float celsius, float humidity) {
    float A0= 373.15/(273.15 + celsius);
    float SUM = -7.90298 * (A0-1);
    SUM += 5.02808 * log10(A0);
    SUM += -1.3816e-7 * (pow(10, (11.344*(1-1/A0)))-1) ;
    SUM += 8.1328e-3 * (pow(10,(-3.49149*(A0-1)))-1) ;
    SUM += log10(1013.246);
    float VP = pow(10, SUM-3) * humidity;
    float T = log(VP/0.61078); // temp var
    return (241.88 * T) / (17.558 - T);
}

// delta max = 0.6544 wrt dewPoint()
// 5x faster than dewPoint()
// reference: http://en.wikipedia.org/wiki/Dew_point
/*
float dewPoint(float celsius, float humidity) {
    float a = 17.271;
    float b = 237.7;
    float temp = (a * celsius) / (b + celsius) + log(humidity/100);
    float Td = (b * temp) / (a - temp);
    return Td;
}
*/

void reset(measure &magnitude) {
    magnitude.count = 0;
    magnitude.sum = 0;
}

void record(measure &magnitude, float value) {
    magnitude.sum = magnitude.sum + value;
    magnitude.count = magnitude.count + 1;
    if (magnitude.count == 1) {
        magnitude.minimum = value;
        magnitude.maximum = value;
    } else {
        magnitude.minimum = min(magnitude.minimum, value);
        magnitude.maximum = max(magnitude.maximum, value);
    }
}

float get_average(measure magnitude) {
    if (magnitude.count > 2) {
        return (magnitude.sum - magnitude.minimum - magnitude.maximum) / (magnitude.count - 2);
    } else {
        return 0.0;
    }
}

void readAll() {

    delay(WARMUP_DELAY);

    DHT22_ERROR_t errorCode = dht.readData();
    if (errorCode == DHT_ERROR_NONE) {
        record(dht22_temperature, dht.getTemperatureC());
        record(dht22_humidity, dht.getHumidity());
    }

    if (bmp_ready) {
        record(bmp085_temperature, bmp.readTemperature());
        record(bmp085_pressure, bmp.readPressure());
    }
    record(fio_temperature, readTemperature());
    record(fio_voltage, (float) readVoltage());

    //long vb = readVoltage(BATT_PIN, BATT_VOLTAGE_FACTOR);
    //long vp = readVoltage(PANEL_PIN, PANEL_VOLTAGE_FACTOR);

}

void sendAll() {

    xbeeWake();

    Serial.println("");

    Serial.print(F("dht22_temperature:"));
    float temperature = get_average(dht22_temperature);
    Serial.println(temperature);
    reset(dht22_temperature);
    delay(20);

    Serial.print(F("dht22_humidity:"));
    float humidity = get_average(dht22_humidity);
    Serial.println(humidity);
    reset(dht22_humidity);
    delay(20);

    Serial.print(F("dht22_dew_point:"));
    Serial.println(dewPoint(temperature, humidity));
    delay(20);

    Serial.print(F("bmp085_temperature:"));
    Serial.println(get_average(bmp085_temperature));
    reset(bmp085_temperature);
    delay(20);

    Serial.print(F("bmp085_pressure:"));
    Serial.println(get_average(bmp085_pressure));
    reset(bmp085_pressure);
    delay(20);

    Serial.print(F("fio_temperature:"));
    Serial.println(get_average(fio_temperature));
    reset(fio_temperature);
    delay(20);

    Serial.print(F("fio_voltage:"));
    Serial.println(get_average(fio_voltage));
    reset(fio_voltage);
    delay(20);

    Serial.println("");
    delay(20);

    xbeeSleep();

}

void setup() {

    // Configure control PIN for XBee
    pinMode(XBEE_SLEEP_PIN, OUTPUT);

    // Using the ADC against internal 1V1 reference for battery monitoring
    analogReference(INTERNAL);

    // Initialize UART
    Serial.begin(9600);

    // Initialize BMP085
    bmp_ready = (bool) bmp.begin();

}

void loop() {
    ++interval;
    Serial.print(".");
    if (interval % MEASURE_EVERY == 0) {
        readAll();
    }
    if (interval % SEND_EVERY == 0) {
        sendAll();
        interval = 0;
    }
    LowPower.powerDown(SLEEP_INTERVAL, ADC_OFF, BOD_OFF);
}
