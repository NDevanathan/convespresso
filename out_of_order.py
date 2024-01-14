from machine import Pin, SPI, ADC, PWM
from ssd1309 import Display
from xglcd_font import XglcdFont

# Define pins for the display
SCR_SCK   = Pin(14)                       # Display clock pin
SCR_MOSI  = Pin(15)                       # Display MOSI pin
SCR_CS    = Pin(13)                       # Display chip select pin
SCR_DC    = Pin(12)                       # Display data/command pin
SCR_RST   = Pin(6)                        # Display reset pin (Not connected, placeholder)

# Define display settings
FONT    = XglcdFont('res/FixedFont5x8.c', 5, 8)  # Display font
WIDTH   = 128                                    # Display width
HEIGHT  = 64                                     # Display height
START_X = WIDTH - 10                             # Display starting x-coordinate
START_Y = 45                                     # Display starting y-coordinate

spi1 = SPI(1, baudrate=10000000, sck=SCR_SCK, mosi=SCR_MOSI)
display = Display(self.spi1, dc=SCR_DC, cs=SCR_CS, rst=SCR_RST)

display.clear_buffers()
m = "ESPRESSO"
display.draw_text(WIDTH//2 + 3 * (len(m)), START_Y - 20, m, FONT, rotate=180)
display.present()
