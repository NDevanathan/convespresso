from machine import Pin
from utime import sleep

led = Pin("LED", Pin.OUT)
while True:
    led.on()
    sleep(1)
    led.off()
    sleep(1)
