import time
import board
import neopixel_spi as neopixel

# Configuration
NUM_PIXELS = 5      # Updated to 5 LEDs
PIXEL_ORDER = neopixel.GRB 

# Initialize the SPI bus
spi = board.SPI()

# Create the NeoPixel object
pixels = neopixel.NeoPixel_SPI(
    spi, 
    NUM_PIXELS, 
    pixel_order=PIXEL_ORDER, 
    auto_write=False
)

try:
    print("Turning all 5 LEDs Red blindking...")
    
    # .fill() applies the color to every LED in the series simultaneously
    for i in range(10):
        pixels.fill((255, 0, 0))  # Red color
        pixels.show()
        time.sleep(0.5)
        
        pixels.fill((0, 0, 0))  # NO color
        pixels.show()
        time.sleep(0.5)
    
    pixels.fill((0, 0, 0))  # Turn off all LEDs
    pixels.show()

except KeyboardInterrupt:
    pixels.fill((0, 0, 0))
    pixels.show()