# Moteino Weather Station

This is a weather station based on **[Moteino WeatherShield R2][6]** with a BME260 Bosch sensor. The whole project has an average power consumption of 26uA and thus can be powered by a small LiPo battery for months or years.
Check my **[Low Power Weather Station with BME280 and Moteino][1]** post for further information about this project, including **power consumption data and hardware modifications**.

## Hardware

![Moteino Weather Station](/images/20161009_021036s.jpg)

## Firmware

The code is very straight forward and there are comments where I thought it was important. It uses the following libraries:

* **[RFM69_ATC][2]** by Felix Rusu and Thomas Studwell
* **[Low-Power][3]** by RocketScream
* **[BME280 Arduino Library][4]** by SparkFun

It also relies on my **RFM69Manager library** to wrap RFM69_ATC. This library manages radio setup and also message sending protocol. Messages from this node have the following format: ```<key>:<value>:<packetID>```. Packet ID is then used in the gateway to detect duplicates or missing packets from a node. If you don't want to send the packetID change the SEND_PACKET_ID value in RFM69Manager.h to 0.

## Configuration

Rename or copy the settings.h.sample file to settings.h and change its values to fit your needs. Check the descriptions for each value.

## Flashing

The project is ready to be build using **[PlatformIO][5]**.
Please refer to their web page for instructions on how to install the builder. Once installed connect the Moteino to your favourite FTDI-like programmer and:

```bash
> platformio run --target upload
```

[1]: http://tinkerman.cat/low-power-weather-station-bme280-moteino/
[2]: https://github.com/LowPowerLab/RFM69
[3]: https://github.com/rocketscream/Low-Power/
[4]: https://github.com/sparkfun/SparkFun_BME280_Arduino_Library
[5]: http://www.platformio.org
[6]: https://lowpowerlab.com/shop/product/123
