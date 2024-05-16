import serial
from PIL import Image
import io

image = Image.Image()

def displayImage():
    # Open serial port
    ser = serial.Serial('COM7', 115200)

    # Initialize an empty byte array
    image_data = bytearray()

    byte = 0

    # Wait for header to be received
    while True:
        prevByte = byte
        byte = ser.read(1)
        if (byte == b'\xD8' and prevByte == b'\xFF'):
            image_data += prevByte
            image_data += byte
            print("Identified frame start")
            break

    # Loop until the end of the image is found
    while True:
        # Read one byte
        prevByte = byte
        byte = ser.read(1)

        # Add the byte to the image data
        image_data += byte

        if (byte == b'\xD9' and prevByte == b'\xFF'):
            print("Identified frame end")
            break

    # Convert bytes to image
    image = Image.open(io.BytesIO(image_data))

    # Display image
    image.show()

while True:
    displayImage()