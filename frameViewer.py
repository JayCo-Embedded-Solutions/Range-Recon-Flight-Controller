import serial
import numpy as np
import matplotlib.pyplot as plt

# QQVGA image resolution
IMAGE_WIDTH = 160
IMAGE_HEIGHT = 120


def parse_rgb565(data):
    # Extract RGB565 values from the 32-bit word
    rgb565_1 = data & 0xFFFF
    rgb565_2 = (data >> 16) & 0xFFFF
    
    # Split RGB565 values into separate R, G, B components
    r1 = (rgb565_1 >> 11) & 0x1F
    g1 = (rgb565_1 >> 5) & 0x3F
    b1 = rgb565_1 & 0x1F
    
    r2 = (rgb565_2 >> 11) & 0x1F
    g2 = (rgb565_2 >> 5) & 0x3F
    b2 = rgb565_2 & 0x1F
    
    return np.array([[r1, g1, b1], [r2, g2, b2]])

def display_image(image):
    plt.imshow(image)
    plt.axis('off')
    plt.show()

def main():
    # Open serial port (replace 'COM3' with the appropriate port name)
    ser = serial.Serial('COM17', 115200, parity=serial.PARITY_EVEN)
    
    # Read and parse serial data
    image_data = np.zeros((IMAGE_HEIGHT, IMAGE_WIDTH, 3), dtype=np.uint8)
    dataList = []
    pixels_received = 0
    try:
        while pixels_received < IMAGE_WIDTH * IMAGE_HEIGHT:
            # Read 32-bit word (4 bytes) from serial port
            data = int.from_bytes(ser.read(4), byteorder='little')

            dataList.append(data)
            
            pixels_received += 2  # Two pixels per 32-bit word

        for i in range(len(dataList)):
            if (i == 0):
                print(dataList[i])

            # Parse RGB565 data
            pixel_values = parse_rgb565(dataList[i])
            
            # Calculate pixel position in the image
            row = (i*2) // IMAGE_WIDTH
            col = (i*2) % IMAGE_WIDTH
            
            # Update image data
            image_data[row, col:col+2, :] = pixel_values
        
        # Display the full image
        display_image(image_data)
                
    except KeyboardInterrupt:
        pass
    
    # Close serial port
    ser.close()

if __name__ == "__main__":
    main()
