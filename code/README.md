# WLS_MQTT_toAdafruitIO_v6_STABLE
- Shawn's code for his dashboard, unedited

# working_AdafruitIO_MQTT_Demo_v2_stable
- botletics demo code configured for the SIM7000A hardware
- Publishing feeds: temperature, humidity
- Subscribing feeds: led toggle, led selection
- Outdated from current Adafruit IO dashboard

# WHS_v1
- created by Corinne
- Publishing feeds: location, temperature, pressure, stage
- Subscribing feeds: led toggle, sampling rate

# WHS_v1.1
- created by Corinne
- Publishing feeds: temperature, pressure, stage
- Subscribing feeds: deployment toggle, sampling rate, initial sea level elevation
- main feature is adding a while loop in the setup that will take in all subscriptions
to initialize parameters, then only start data collection once the deployment button
is toggled

# WHS_v1.1.2 
- created by Corinne
- same feeds as v1.1
- pressure sensor added

# WHS_v1.1.3
- created by Corinne
- added location feed & GPS functionality
- added indicator LEDs to display when network is connected, MQTT is connected, data is being collected, the GPS is found, and publishing errors
- NOTE: the Adafrut_FONA.h library had to be modified for this code to suppress a Not Authorized to Connect error

# debugging
- folder contains small sketches for troubleshooting sensors, logic flows, etc