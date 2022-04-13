# WHS_v2.1.0.ino
- integrate DDASA v1
- Sampling rate can exceed keepAlive time
- built off of WHS_v2.0
- Publishing feeds: location, ultrasonic stage, pressure stage, temperature, ambient pressure, update gps, fona LiPo
- Subscribing feeds: deployment toggle, sampling rate, initial elevation, update gps

# WHS_v2.1.0.noSPI.ino
- Excludes error messages that stop code when SD card or MPRLS are not working
- this was made purely for a quick field test
- datalogging board experienced some issues at the SPI ports from getting wet
- use when you care about just stage readings