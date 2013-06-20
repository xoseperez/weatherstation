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

#define BAUD_RATE 57600

#define USE_NOAA

#define BATT_PIN 0
#define PANEL_PIN 1
#define XBEE_SLEEP_PIN 4
#define DHT_PIN 12
#define NOTIFICATION_PIN 13

#define DHT_TYPE DHT22
#define VOLTAGE_REFERENCE 3300
#define BATT_VOLTAGE_FACTOR 4.24 // 100KOhm + 324KOhm
#define PANEL_VOLTAGE_FACTOR 6.61 // 100kOhm + 561kOhm
#define XBEE_DELAY 40
#define XBEE_LINK 4000

#define SLEEP_INTERVAL SLEEP_2S
#define MEASURE_EVERY 28 // each measurement takes roughly 4 seconds
#define SEND_EVERY 5
#define WARMUP_DELAY 2000

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
unsigned long measures = 0;

struct measure dht22_temperatures = {0,0,0,0};
struct measure dht22_humidities = {0,0,0,0};
struct measure bmp085_temperatures = {0,0,0,0};
struct measure bmp085_pressures = {0,0,0,0};
struct measure fio_temperatures = {0,0,0,0};
struct measure fio_voltages = {0,0,0,0};
struct measure battery_voltages = {0,0,0,0};
struct measure panel_voltages = {0,0,0,0};

// ===========================================
// Methods
// ===========================================

/*
 * xbeeSleep
 * Sets the XBee radio to sleep
 *
 * @return void
 */
void xbeeSleep() {
    delay(XBEE_DELAY);
    pinMode(XBEE_SLEEP_PIN, HIGH);
}

/*
 * xbeeWake
 * Wakes up the XBee radio to sleep
 *
 * @return void
 */
void xbeeWake() {
    digitalWrite(XBEE_SLEEP_PIN, LOW);
    delay(XBEE_DELAY);
}

/*
 * readVoltage
 * Reads the analog PIN and performs the conversion to get mV
 *
 * @param byte pin PIN number
 * @param float factor conversion factor based on voltage divider
 * @return long
 */
long readVoltage(byte pin, float factor) {
    int reading = analogRead(pin);
    return (long) map(reading, 0, 1023, 0, VOLTAGE_REFERENCE) * factor;
}

/*
 * readVoltage
 * Reads the internal 328 voltage register
 *
 * @return long
 */
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

/*
 * readTemperature
 * Reads the internal 328 temperature register
 *
 * @return float
 */
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

#ifdef USE_NOAA

/*
 * dewPoint
 * Calculates dew point
 *
 * reference: http://wahiduddin.net/calc/density_algorithms.htm 
 *
 * @param temperature float temperature in celsius
 * @param humidity float humidity in %
 * @return float
 */
float dewPoint(float temperature, float humidity) {
    float A0= 373.15/(273.15 + temperature);
    float SUM = -7.90298 * (A0-1);
    SUM += 5.02808 * log10(A0);
    SUM += -1.3816e-7 * (pow(10, (11.344*(1-1/A0)))-1) ;
    SUM += 8.1328e-3 * (pow(10,(-3.49149*(A0-1)))-1) ;
    SUM += log10(1013.246);
    float VP = pow(10, SUM-3) * humidity;
    float T = log(VP/0.61078); // temp var
    return (241.88 * T) / (17.558 - T);
}

#else

/*
 * dewPoint
 * Calculates dew point
 * delta max = 0.6544 wrt dewPoint()
 * 5x faster than dewPoint()
 * reference: http://en.wikipedia.org/wiki/Dew_point
 *
 * @param temperature float temperature in celsius
 * @param humidity float humidity in %
 * @return float
 */
float dewPoint(float temperature, float humidity) {
    float a = 17.271;
    float b = 237.7;
    float temp = (a * temperature) / (b + temperature) + log(humidity/100);
    float Td = (b * temp) / (a - temp);
    return Td;
}

#endif

/*
 * reset
 * Resets the structure
 *
 * @param measure magnitude
 * @return void
 */
void reset(measure &magnitude) {
    magnitude.count = 0;
    magnitude.sum = 0;
}

/*
 * record
 * Records the value into a measure structure
 *
 * @param measure magnitude
 * @param float value
 * @return void
 */
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

/*
 * calculateAverage
 * Calculates the average for the stored values substracting the minimum and maximum first
 *
 * @param measure magnitude
 * @return float
 */
float calculateAverage(measure magnitude) {
    if (magnitude.count > 2) {
        return (magnitude.sum - magnitude.minimum - magnitude.maximum) / (magnitude.count - 2);
    } else {
        return 0.0;
    }
}

/*
 * readAll
 * Reads all magnitudes and stores them in metric structures
 *
 * @return void
 */
void readAll() {

    // Allowing the DHT22 to warm up
    delay(WARMUP_DELAY);

    DHT22_ERROR_t errorCode = dht.readData();
    if (errorCode == DHT_ERROR_NONE) {
        record(dht22_temperatures, dht.getTemperatureC());
        record(dht22_humidities, dht.getHumidity());
    }

    if (bmp_ready) {
        record(bmp085_temperatures, bmp.readTemperature());
        record(bmp085_pressures, bmp.readPressure());
    }
    record(fio_temperatures, readTemperature());
    record(fio_voltages, (float) readVoltage());
    record(battery_voltages, (float) readVoltage(BATT_PIN, BATT_VOLTAGE_FACTOR));
    record(panel_voltages, (float) readVoltage(PANEL_PIN, PANEL_VOLTAGE_FACTOR));

}

/*
 * resetAll
 * Resets all averages
 *
 * @return void
 */
void resetAll() {
    reset(dht22_humidities);
    reset(dht22_temperatures);
    reset(bmp085_temperatures);
    reset(bmp085_pressures);
    reset(fio_temperatures);
    reset(fio_voltages);
    reset(battery_voltages);
    reset(panel_voltages);
}

/*
 * sendAll
 * Gets averages and sends all data through UART link
 *
 * @return void
 */
void sendAll() {

    float dht22_temperature = calculateAverage(dht22_temperatures);
    float dht22_humidity = calculateAverage(dht22_humidities);
    float dht22_dew_point = dewPoint(dht22_temperature, dht22_humidity);
    float bmp085_temperature = calculateAverage(bmp085_temperatures);
    float bmp085_pressure = calculateAverage(bmp085_pressures);
    float fio_temperature = calculateAverage(fio_temperatures);
    float fio_voltage = calculateAverage(fio_voltages);
    float battery_voltage = calculateAverage(battery_voltages);
    float panel_voltage = calculateAverage(panel_voltages);

    xbeeWake();

    Serial.print(F("T1:"));
    Serial.println(dht22_temperature);

    Serial.print(F("H1:"));
    Serial.println(dht22_humidity);

    Serial.print(F("D1:"));
    Serial.println(dht22_dew_point);

    Serial.print(F("T2:"));
    Serial.println(bmp085_temperature);

    Serial.print(F("P2:"));
    Serial.println(bmp085_pressure);

    Serial.print(F("T3:"));
    Serial.println(fio_temperature);

    Serial.print(F("V3:"));
    Serial.println(fio_voltage);

    Serial.print(F("V4:"));
    Serial.println(battery_voltage);

    Serial.print(F("V5:"));
    Serial.println(panel_voltage);

    xbeeSleep();

}

/*
 * setup
 *
 * @return void
 */
void setup() {

    // Using the ADC against internal 1V1 reference for battery monitoring
    analogReference(DEFAULT);

    // Initialize UART
    Serial.begin(BAUD_RATE);

    // Initialize BMP085
    bmp_ready = (bool) bmp.begin();

    // Link radio
    pinMode(XBEE_SLEEP_PIN, OUTPUT);
    pinMode(NOTIFICATION_PIN, OUTPUT);
    xbeeWake();
    digitalWrite(NOTIFICATION_PIN, HIGH);
    delay(XBEE_LINK);
    Serial.println(F("STATUS:1"));
    digitalWrite(NOTIFICATION_PIN, LOW);
    xbeeSleep();

}

/*
 * loop
 *
 * @return void
 */
void loop() {
    ++interval;
    if (interval % MEASURE_EVERY == 0) {
        digitalWrite(NOTIFICATION_PIN, HIGH);
        readAll();
        interval = 0;
        ++measures;
        if (measures % SEND_EVERY == 0) {
            sendAll();
            resetAll();
            measures = 0;
        }
        digitalWrite(NOTIFICATION_PIN, LOW);
    }
    LowPower.powerDown(SLEEP_INTERVAL, ADC_OFF, BOD_OFF);
}
