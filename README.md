# home-automation
Home automation using Arduino and Raspberry PI

## Water Level Controller
Arduino is used to sense the water level and then switch on and off motor based on the water levels in the overhead tank and storage tank.
It displays the values on an LCD display as well as publishes to a MQTT server running on the Respberry PI.

## MQTT Server
Uses the [Mosquitto MQTT server](https://mosquitto.org/) .

## Home Automation
The MQTT server data is used by [Home Assistant](https://home-assistant.io) running on the PI to store and display the results for now.

