/*

Radio

RFM69 Radio Manager for ESP8266
Based on sample code by Felix Rusu - http://LowPowerLab.com/contact
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

Requires encapsulating any reference to SPCR and SPSR in SPIFlash.cpp
in an #if clause like this:

#if defined(SPCR) & defined(SPSR)
...
#endif

*/

#include "RFM69Manager.h"

bool RFM69Manager::initialize(uint8_t frequency, uint8_t nodeID, uint8_t networkID, const char* key, uint8_t gatewayID, int16_t targetRSSI) {

    bool ret = RFM69_CLASS::initialize(frequency, nodeID, networkID);
    encrypt(key);
    _gatewayID = gatewayID;
    #if ENABLE_ATC
        if (_gatewayID > 0) enableAutoPower(targetRSSI);
    #else
        if (_gatewayID > 0) setPowerLevel(31);
    #endif
    if (_isRFM69HW) setHighPower();

    #if RADIO_DEBUG
        char buff[50];
        sprintf(buff, "[RADIO] Working at %d Mhz", frequency == RF69_433MHZ ? 433 : frequency == RF69_868MHZ ? 868 : 915);
        Serial.println(buff);
        Serial.print(F("[RADIO] Node: "));
        Serial.println(nodeID);
        Serial.print(F("[RADIO] Network: "));
        Serial.println(networkID);
        if (gatewayID == 0) {
            Serial.println("[RADIO] This node is a gateway");
        } else {
            Serial.print(F("[RADIO] Gateway: "));
            Serial.println(gatewayID);
        }
        #if ENABLE_ATC
            Serial.println(F("[RADIO] Auto Transmission Control (ATC) enabled"));
        #else
            Serial.println(F("[RADIO] Auto Transmission Control (ATC) disabled"));
        #endif
    #endif

    return ret;

}

void RFM69Manager::promiscuous(bool promiscuous) {
    RFM69_CLASS::spyMode(promiscuous);
    _promiscuousMode = promiscuous;
    #if RADIO_DEBUG
        if (_promiscuousMode) {
            Serial.println(F("[RADIO] Promiscuous mode ON"));
        } else {
            Serial.println(F("[RADIO] Promiscuous mode OFF"));
        }
    #endif
}

void RFM69Manager::onMessage(TMessageCallback fn) {
    _callback = fn;
}

bool RFM69Manager::loop() {

    boolean ret = false;

    if (receiveDone()) {

        uint8_t senderID = SENDERID;
        uint8_t targetID = _promiscuousMode ? TARGETID : _address;
        int16_t rssi = RSSI;
        uint8_t length = DATALEN;
        char buffer[length + 1];
        strncpy(buffer, (const char *) DATA, length);
        buffer[length] = 0;

        // Do not send ACKs in promiscuous mode,
        // we want to listen without being heard
        if (!_promiscuousMode) {
            if (ACKRequested()) sendACK();
        }

        uint8_t parts = 1;
        for (uint8_t i=0; i<length; i++) {
            if (buffer[i] == ':') ++parts;
        }

        if (parts > 1) {

            uint8_t packetID = 0;
            char * name = strtok(buffer, ":");
            char * value = strtok(NULL, ":");
            if (parts > 2) {
                char * packet = strtok(NULL, ":");
                packetID = atoi(packet);
            }

            _message.messageID = ++_receiveCount;
            _message.packetID = packetID;
            _message.senderID = senderID;
            _message.targetID = targetID;
            _message.name = name;
            _message.value = value;
            _message.rssi = rssi;
            ret = true;

            if (_callback != NULL) {
                _callback(&_message);
            }

        }

    }

    return ret;

}

bool RFM69Manager::send(uint8_t destinationID, char * name, char * value, uint8_t retries, bool requestACK) {

    char message[30];
    #if SEND_PACKET_ID
        if (++_sendCount == 0) _sendCount = 1;
        sprintf(message, "%s:%s:%d", name, value, _sendCount);
    #else
        sprintf(message, "%s:%s", name, value);
    #endif

    #if RADIO_DEBUG
        Serial.print(F("[RADIO] Sending: "));
        Serial.print(message);
    #endif

    bool ret = true;
    if (retries > 0) {
        ret = sendWithRetry(destinationID, message, strlen(message), retries);
    } else {
        RFM69_CLASS::send(destinationID, message, strlen(message), requestACK);
    }

    #if RADIO_DEBUG
        if (ret) {
            Serial.println(" OK");
        } else {
            Serial.println(" KO");
        }
    #endif

    return ret;

}
