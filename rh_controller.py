# Parts
		# 1 x Adafruit 0.56" 4-Digit 7-Segment Display w/ I2C Backpack - Blue (STEMMA QT / qwiic) [ID:881] = $11.95
      # 1 x Adafruit Sensirion SHT31-D (Temperature & Humidity Sensor) [ID:2857] = $13.95
		# 1 x Adafruit I2C Stemma QT Rotary Encoder Breakout with NeoPixel (STEMMA QT / Qwiic) [ID:4991] = $5.95
		# 2 x STEMMA QT / Qwiic JST SH 4-pin to Premium Male Headers Cable (150mm Long) [ID:4209] = $1.90
		# 1 x Adafruit Perma-Proto Half-sized Breadboard PCB - 3 Pack! [ID:571] =$12.50
		# 3 x STEMMA QT / Qwiic JST SH 4-pin Cable - 100mm Long [ID:4210] = $2.85
		# 1 x Assembled Pi Cobbler Plus - Breakout Cable (for Pi B+/A+/Pi 2/Pi 3/Pi 4) [ID:2029] = $6.95
      # 1 x Adafruit Controllable Four Outlet Power Relay Module version 2 - (Power Switch Tail Alternative) [ID: 2935]
# Python Libraries
   # Adafruit 881 .56" 4-Digit 7-segment display with I2C backpack (Adafruit ID: 881)
   # https://github.com/adafruit/Adafruit_CircuitPython_HT16K33 
   #   python -m pip install adafruit-circuitpython-ht16k33
   #   note from tutorial but doesn't seem to be needed for 7-segment display (sudo apt-get install python3-pil)
   #
   # Adafruit Sensirion SHT31-D (Temperature & Humidity Sensor) (Adafruit ID: 2857)
   #   python -m pip install adafruit-circuitpython-sht31d
   #
   # Adafruit I2C Stemma QT Rotary Encoder Breakout with NeoPixel (Adafruit ID: 4991)
   # https://github.com/adafruit/Adafruit_CircuitPython_seesaw
   # https://github.com/adafruit/Adafruit_CircuitPython_seesaw/blob/main/examples/seesaw_rotary_neopixel.py
   #   python -m pip install adafruit-circuitpython-seesaw
   #
   # For OpenWeatherMap - PyOWM
   #   python -m pip install pyowm
   #   https://github.com/csparpa/pyowm 

# James S. Lucas
# January 29, 2023

'''Measures relative humidity and switches a 110V power strip according to a user defined setpoint. 
Intent is to control a humidifier.

Parts list in the code comments'''

# User defined parameters
time_logging = False
event_logging = True
console_output = False
use_owm = True
# (seconds)
heater_on_interval = 61
heater_on_duration = 1
reading_interval = 3
logging_interval = 300
# % RH
RH_range = 4

import sys
from time import sleep
from datetime import datetime
import csv
import config
import logging

import board
i2c = board.I2C()

# Initialize 7-Segment display (Adafruit ID: 881)
from adafruit_ht16k33.segments import Seg7x4
display = Seg7x4(i2c, address=0x70)
display.brightness = 0.4
display.colon = True

# Initialize SHT31-D Temperature and Humidity Sensor (Adafruit ID: 2857)
import adafruit_sht31d
sensor = adafruit_sht31d.SHT31D(i2c, address=0x44)

# Initialize Adafruit I2C Stemma QT Rotary Encoder with Neopixel Breakout (Adafruit ID: 4991)
from rainbowio import colorwheel
from adafruit_seesaw import seesaw, neopixel, rotaryio, digitalio 
seesaw = seesaw.Seesaw(i2c, 0x36)
encoder = rotaryio.IncrementalEncoder(seesaw)
seesaw.pin_mode(24, seesaw.INPUT_PULLUP)
switch = digitalio.DigitalIO(seesaw, 24)
pixel = neopixel.NeoPixel(seesaw, 6, 1)
pixel.brightness = 0.1
last_position = -1
# start at red
color = 0

#Initialize Motor Hat
from adafruit_motorkit import MotorKit
from adafruit_motor import stepper
kit = MotorKit(address=0x60)

# Initialize GPIO
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
relay_pin = 13
GPIO.setup(relay_pin, GPIO.OUT)
GPIO.output(relay_pin, GPIO.LOW)


# Inititalize OpenWeatherMap (OWM)
from pyowm import OWM
owm = OWM(config.owm_key)
mgr = owm.weather_manager()
#import pyowm

logging.basicConfig(filename='error.log')

def check_enc_position(last_position):
   '''Returns the position of a rotary encoder.
   
   Limts the encoder range to between 0 - 99 inclusively.'''

   # Negate the position to make clockwise rotation positive.
   position = -encoder.position
   if position > 99 and position < 101:
      encoder.position = -99
      position = -encoder.position
   elif position < 0:
      encoder.position = 0
      position = -encoder.position
   # To comepnsate for high vales erroneously returned by the encoder.
   elif position >= 101:
      encoder.position = -last_position
      position = -encoder.position
   return position


def rh_control(position, humidifier_mode, event_logging, use_owm):
   '''Compares the actual measured relative humidity to setpoint.
   
   Turns the power strip on and off with a range of +/- RH_range around setpoint.
   Returns humidifier mode for logging. '''

   if round(sensor.relative_humidity) <= (position - round(RH_range / 2)):
      GPIO.output(relay_pin, GPIO.HIGH)
      if humidifier_mode == 0:
         humidifier_mode = 1
         if event_logging:
            write_log(humidifier_mode, use_owm)
   elif round(sensor.relative_humidity) >= (position + round(RH_range / 2)):
      GPIO.output(relay_pin, GPIO.LOW)
      if humidifier_mode == 1:
         humidifier_mode = 0
         if event_logging:
            write_log(humidifier_mode, use_owm)
   return humidifier_mode


def display_rh():
   '''Displays the measured RH on the last two digits of a four digit 7-segment display.'''

   first_digit = str(round(sensor.relative_humidity) // 10)
   second_digit = str(round(sensor.relative_humidity) % 10)
   display[2] = first_digit
   display[3] = second_digit


def heater_control(console_output, heater_start_time, heater_on_interval, heater_on_duration):
   '''Periodically turns on the RH sensor's internal heater for a short time.'''

   heater_td = datetime.now() - heater_start_time
   if heater_td.total_seconds() >= heater_on_interval:
      if sensor.heater == False:
         sensor.heater = True
         if console_output:
            print("Sensor Heater status =", sensor.heater)
   if heater_td.total_seconds() >= heater_on_interval + heater_on_duration:
      if sensor.heater == True:
         sensor.heater = False
         heater_start_time = datetime.now()
         if console_output:
            print("Sensor Heater status =", sensor.heater)
   return heater_start_time


def rh_log(logging_interval, log_start_time, humidifier_mode, use_owm):
   '''Writes the Datetime, Measured RH Value and Huimidifier Mode to a csv file.'''

   log_td = datetime.now() - log_start_time
   if log_td.total_seconds() >= logging_interval:
      write_log(humidifier_mode, use_owm)
      log_start_time = datetime.now()
   return log_start_time


def write_log(humidifier_mode, use_owm):
   '''Writes the Datetime, Measured RH Value and Huimidifier Mode to a csv file.'''

   if use_owm:
      rh, temp = owm_check()
      row = datetime.now(), sensor.relative_humidity, sensor.temperature, humidifier_mode, rh, temp
   else:
      row = datetime.now(), sensor.relative_humidity, sensor.temperature, humidifier_mode
   with open('rh_log.csv', "a") as csv_file:
      writer = csv.writer(csv_file)
      writer.writerow(row)
      csv_file.flush()


def display_position(position, color):
   '''Displays the encoder position on the first two digits of a 7-segment display.'''

   first_digit = str(position // 10)
   second_digit = str(position % 10)
   display[0] = first_digit
   display[1] = second_digit
   if switch.value:
      # Change the LED color.
      # Advance forward through the colorwheel.
      if position > last_position:
         color += 1
      else:
      # Advance backward through the colorwheel.
         color -= 1
      # wrap around to 0-256
      color = (color + 256) % 256
      pixel.fill(colorwheel(color))
   else:
      # If the encoder is turned while the button is pressed, change the brightness.
      if position > last_position:
         # Increase the brightness.
         pixel.brightness = min(1.0, pixel.brightness + 0.1)
      else:
         # Decrease the brightness.
         pixel.brightness = max(0, pixel.brightness - 0.1)
   return color


def owm_check():
   try:
      observation = mgr.weather_at_place('Jackson,US,WY')
      w = observation.weather
      ambient_rh = w.humidity
      ambient_temp = w.temperature('fahrenheit')['temp']
   except Exception as e:
      print("owm_check() exception:", e)
      logging.exception("RH_Log OWM Error:\n%s" % e)
      ambient_rh = 'error'
      ambient_temp = 'error'
   return ambient_rh, ambient_temp
 

try:
  heater_start_time = datetime.now()
  reading_start_time = datetime.now()
  log_start_time = datetime.now()
  encoder.position = 0
  last_position = 0
  position = check_enc_position(last_position)
  color = display_position(position, color)
  humidifier_mode = 0
  while True:
      sleep(0.01)
      position = check_enc_position(last_position)
      if position != last_position:
         color = display_position(position, color)
      last_position = position
      sleep(.01)
      heater_start_time = heater_control(console_output, heater_start_time, heater_on_interval, heater_on_duration)
      reading_td = datetime.now() - reading_start_time
      if reading_td.total_seconds() >= reading_interval:
         reading_start_time = datetime.now()
         if console_output:
            print("Humidity: %0.1f %%" % sensor.relative_humidity)
         humidifier_mode = rh_control(position, humidifier_mode, event_logging, use_owm)
         display_rh()
         if time_logging == True:
            log_start_time = rh_log(logging_interval, log_start_time, humidifier_mode, use_owm)
except KeyboardInterrupt:
   pixel.brightness = 0.0
   sensor.heater = False
   GPIO.output(relay_pin, GPIO.LOW)
   GPIO.cleanup()
   sleep(1)
   print(" ")
   print("Sensor Heater status =", sensor.heater)
   display.fill(0)
   sys.exit()