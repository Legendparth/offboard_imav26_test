import time
import board
import neopixel_spi as neopixel

# Configuration
NUM_PIXELS = 1      # Number of LEDs in your module/strip
PIXEL_ORDER = neopixel.GRB # WS2812B standard color order

# Initialize the SPI bus. board.SPI() automatically maps to SPI1 (Pin 19 MOSI)
spi = board.SPI()

# Create the NeoPixel object
pixels = neopixel.NeoPixel_SPI(
    spi, 
    NUM_PIXELS, 
    pixel_order=PIXEL_ORDER, 
    auto_write=False
)

try:
    # print("Turning LED Blue...")
    # Set the first LED (index 0) to Blue (Red=0, Green=0, Blue=255)
    
    
    for i in range(10):
        pixels[0] = (255, 0, 0)
        pixels.show()  # Update the LED to show the color
        time.sleep(1)  # Wait for 1 second
        pixels[0] = (0, 0, 0)
        pixels.show()
        time.sleep(1)

    
    # Turn it off
    # print("Turning LED Off...")
    

except KeyboardInterrupt:
    # Ensure LED turns off if you exit the script manually
    pixels[0] = (0, 0, 0)
    pixels.show()