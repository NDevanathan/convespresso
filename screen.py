import random
from time import sleep
from machine import Pin, SPI
from ssd1309 import Display


def test():
    """Test code."""
    x = random.randint(1,126)
    y = random.randint(1,62)
    dx = random.randint(1,3)
    dy = random.randint(1,3)
    
    spi = SPI(1, baudrate=10000000, sck=Pin(14), mosi=Pin(15))
    display = Display(spi, dc=Pin(12), cs=Pin(13), rst=Pin(11))
    
    while True:
        # Clear only the old ball position
        display.fill_circle(x, y, 3, invert=True)  # Assuming color=0 clears the pixels
        
        x += dx
        y += dy
        
        if x >= 127:
            dx = -dx
            x = 127
        if x <= 0:
            dx = -dx
            x = 0
        if y >= 63:
            dy = -dy
            y = 62
        if y <= 0:
            dy = -dy
            y = 1
            
        # Draw the ball at the new position
        display.fill_circle(x, y, 3)
        display.present()
        
        sleep(0.05)



test()
