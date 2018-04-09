# Copyright (c) 2017 Adafruit Industries
# Author: James DeVito
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


import RPi.GPIO as GPIO

import time

import Adafruit_GPIO.SPI as SPI
import Adafruit_SSD1306

from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont



# Input pins:
L_pin = 27 
R_pin = 23 
C_pin = 4 
U_pin = 17 
D_pin = 22 

A_pin = 5 
B_pin = 6 


GPIO.setmode(GPIO.BCM) 

GPIO.setup(A_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP) # Input with pull-up
GPIO.setup(B_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP) # Input with pull-up
GPIO.setup(L_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP) # Input with pull-up
GPIO.setup(R_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP) # Input with pull-up
GPIO.setup(U_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP) # Input with pull-up
GPIO.setup(D_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP) # Input with pull-up
GPIO.setup(C_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP) # Input with pull-up


# Raspberry Pi pin configuration:
RST = 24
# Note the following are only used with SPI:
DC = 23
SPI_PORT = 0
SPI_DEVICE = 0

# Beaglebone Black pin configuration:
# RST = 'P9_12'
# Note the following are only used with SPI:
# DC = 'P9_15'
# SPI_PORT = 1
# SPI_DEVICE = 0

# 128x32 display with hardware I2C:
# disp = Adafruit_SSD1306.SSD1306_128_32(rst=RST)

# 128x64 display with hardware I2C:
disp = Adafruit_SSD1306.SSD1306_128_64(rst=RST)

# Note you can change the I2C address by passing an i2c_address parameter like:
# disp = Adafruit_SSD1306.SSD1306_128_64(rst=RST, i2c_address=0x3C)

# Alternatively you can specify an explicit I2C bus number, for example
# with the 128x32 display you would use:
# disp = Adafruit_SSD1306.SSD1306_128_32(rst=RST, i2c_bus=2)

# 128x32 display with hardware SPI:
# disp = Adafruit_SSD1306.SSD1306_128_32(rst=RST, dc=DC, spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE, max_speed_hz=8000000))

# 128x64 display with hardware SPI:
# disp = Adafruit_SSD1306.SSD1306_128_64(rst=RST, dc=DC, spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE, max_speed_hz=8000000))

# Alternatively you can specify a software SPI implementation by providing
# digital GPIO pin numbers for all the required display pins.  For example
# on a Raspberry Pi with the 128x32 display you might use:
# disp = Adafruit_SSD1306.SSD1306_128_32(rst=RST, dc=DC, sclk=18, din=25, cs=22)

# Initialize library.
disp.begin()

# Clear display.
disp.clear()
disp.display()

# Create blank image for drawing.
# Make sure to create image with mode '1' for 1-bit color.
width = disp.width
height = disp.height
image = Image.new('1', (width, height))

# Get drawing object to draw on image.
draw = ImageDraw.Draw(image)

# Draw a black filled box to clear the image.
draw.rectangle((0,0,width,height), outline=0, fill=0)

# variables for the position of the ball and paddle
paddleX = 0
paddleY = 0
oldPaddleX = 0
oldPaddleY = 0
ballDirectionX = 1
ballDirectionY = 1

ballSpeed = 5

ballX = 0
ballY = 0
oldBallX = 0
oldBallY = 0

try:
    while 1:
        # draw.polygon([(30, 60), (40, 42), (20, 42)], outline=255, fill=0) 
        draw.rectangle((0,0,width,height), outline=0, fill=0)

        paddleX = oldPaddleX
        paddleY = oldPaddleY

        if GPIO.input(U_pin): # button is released
            #Up
            paddleX = paddleX
        else: # button is pressed:
            #Up filled
            if (paddleX >= height):
                paddleX = height
            else:
                paddleX = paddleX + 1

        if GPIO.input(L_pin): # button is released
            #left
            paddleX = paddleX
        else: # button is pressed:
            #left filled
            if (paddleY <= 0):
                paddleY = 0
            else:
                paddleY = paddleY - 1

        if GPIO.input(R_pin): # button is released
            #right
            paddleX = paddleX
        else: # button is pressed:
            #right filled
            if (paddleY >= width):
                paddleY = width
            else:
                paddleY = paddleY + 1

        if GPIO.input(D_pin): # button is released
            #down
            paddleX = paddleX
        else: # button is pressed:
            if (paddleX <= 0):
                paddleX = 0
            else:
                paddleX = paddleX - 1
            #down filled

        if ((oldPaddleX != paddleX) or (oldPaddleY != paddleY)):
           draw.rectangle((oldPaddleX,oldPaddleY, 20, 5), outline=0, fill=0)

        oldPaddleX = paddleX
        oldPaddleY = paddleY

        moveBall()

        disp.display() 


except KeyboardInterrupt: 
    GPIO.cleanup()

def moveBall():
    if (ballX > width) or (ballX < 0):
        ballDirectionX = -ballDirectionX

    if (ballY > height) or (ballY < 0):
        ballDirectionY = -ballDirectionY

    if(inPaddle(ballX, ballY, paddleX, paddleY, 20, 5)):
        ballDirectionX = -ballDirectionX
        ballDirectionY = -ballDirectionY

    ballX += ballDirectionX
    ballY += ballDirectionY

    if (oldBallX != ballX) or (oldBallY != ballY):
        draw.rectangle((oldBallX, oldBallY, 5, 5), outline=0, fill=0)

    draw.rectangle((ballX, ballY, 5, 5), outline=0, fill=0)

    oldBallX = ballX
    oldBallY = ballY


def inPaddle(x, y, rectX, rectY, rectWidth, rectHeight):
    result = False
    if ((x >= rectX and x <= (rectX + rectWidth)) and (y >= rectY and y <= (rectY + rectHeight))):
        result = True
    return result
