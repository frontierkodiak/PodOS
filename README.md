<p align="center">
  <img width="400" alt="PodOS v0 0 t 1044w@2x" src="https://github.com/frontierkodiak/PodOS/assets/18093848/7dc0c136-026a-458d-99f1-98624971e647">
</p>



# PodOS 📸
<p align="center">
  <img width="400" alt="PodOS runs on Polli Pods, which are connected to and managed by a central Polli Hub or Hub+." src="https://github.com/frontierkodiak/PodOS/assets/18093848/6f7ab557-dbd2-4363-bb9e-a10f87a991c9">
</p>


C++ firmware for ESP32-based first-gen Polli Pods. Designed for tight integration with PolliOS for Pods deployed as part of a [Polli Swarm]([url](https://www.polli.ai/hardware)), where Pods are connected to and managed by a Polli Hub or Hub+.

Exposes HTTP endpoints for image/sensor requests and Pod management. Hosts a small management webpage (built atop [IoTWebConf](https://github.com/prampec/IotWebConf)) that can be used to adjust camera settings, configure new sensors, and configure WiFi connections.

Webserver & camera runs on Core 1, while sensor readings run async on Core 2.


## Status
Stable and production-ready, with some important caveats (see below). I don't plan to spend more development time on PodOS until I determine the hardware direction for production Polli Pods. Pod+/Pro/Pro+ will target Linux/SBCs, so we are unlikely to stick with ESP32 arch for the base Pod.

## Features
- Aggressive power management.
  - Naptime: Automatically downclocks to 80MHz b/w frames based on the sampling frequency & time taken to return last frame.
    - Can optionally enter 'light sleep' for longer (>15s) sample frequencies.
  - Optionally allows sleeping GPS sensors b/w reads.
  - Bedtime: enters overnight deep sleep when prompted by PolliOS.
- Rich sensor support.
  - BME280 temperature/pressure/humidity sensor.
  - NEO-6M GPS. Knockoffs also work.
  - Battery charge level (0-100%).
    - NOTE: Cannot use battery monitor alongside other sensors when using AITHINKER ESP32-CAM boards. See 'Caveats'. 
- Simple & stable.
  - ~1k LoC. 
  - Zero stability issues over a 6mo period of constant use across 10 Pods.



## To-Do
- Re-enable OTA updates.
- Re-enable 'light sleep' naptime & automatically trigger for sufficiently long sample frequencies.
- Test lower clocks. We might be able to downclock even outside of the 'naptime' (b/w frames) periods. How much slower is JPEG encoding @160MHz? 80MHz?
  - I get about 20k images across 48hrs per charge (10000mAh battery). Can 2-3x duration with sparser samples, GPS power management. Potential for even more with underclocking.
 

### Hardware/deployment caveats
_Only applicable to AITHINKER ESP32-CAM boards. I have not tested others, which may or may not have the same hardware limitations._
These boards have a number of documented limitations and undocumented hardware bugs that prevent us from using all the I/O pins exposed on the chip-- in some cases conditionally, and in other cases not at all.
If you want more details about any of these issues, ping me and I can send over my notes. 

- You must wait to connect the GPS & BME280 sensors until the ESP32 has connected to the WiFi AP for the first time.
  - Why? Undocumented hardware bugs mean that some GPIO pins are ephemerally used by ESP32 WiFi.
  - After establishing initial WiFi connection and connecting sensors, go to the PodOS management page hosted by the Pod and register the sensors and the pins you have selected. Pod will automatically reboot.
  - After this initial setup & power cycle, Pod has no problem connecting to WiFi with sensors connected.
- ADC2 pins cannot work (at least for analogRead) while WiFi is active. This means:
  - Battery monitor does not work alongside WiFi on these boards.
  - Any power control pins (ex. to turn off GPS when not in use) do not work when WiFi is in use.
    - I inadvertantly purchased counterfeit NEO-6M modules that do not respond to sleep commands. Should be able to solve this in software with legit boards-- IIRC there are enough pins to run TX to the GPS module.
    - If you verify this on a legit NEO-6M module, a PR to actively manage GPS power is welcome.
- ESP32-CAM camera connections can be quite finicky, but most problems can be solved with a reboot. We're currently handling autorecovery within PolliOS, but you could also do this on-device via PodOS.
