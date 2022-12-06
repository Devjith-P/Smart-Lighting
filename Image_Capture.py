import cv2
import sys
import time
import serial
from PIL import Image
import screen_brightness_control as sbc
sys.path.append('/usr/local/lib/python2.7/site-packages')

key = cv2.waitKey(1)
webcam = cv2.VideoCapture(0)





check, frame = webcam.read()
print(check)  # prints true as long as the webcam is running
print(frame)  # prints matrix values of each framecd
cv2.imshow("CAPTURING",frame)
key = cv2.waitKey(1)

cv2.imwrite(filename='saved_img.jpg', img=frame)
webcam.release()
img_new = cv2.imread('saved_img.jpg', cv2.IMREAD_GRAYSCALE)
img_new = cv2.imshow("Captured Image", img_new)
cv2.waitKey(1650)
cv2.destroyAllWindows()
print("Processing image...")
img_ = cv2.imread('saved_img.jpg', cv2.IMREAD_ANYCOLOR)
print("Converting RGB image to grayscale...")
gray = cv2.cvtColor(img_, cv2.COLOR_BGR2GRAY)
print("Converted RGB image to grayscale...")
print("Resizing image to 28x28 scale...")
img_ = cv2.resize(gray, (350, 350))
print("Resized...")
img_resized = cv2.imwrite(filename='saved_img-final.jpg', img=img_)
print("Image saved!")









print("WORKING")
image = Image.open("saved_img.jpg")


#ser = serial.Serial('COM3', 9600, timeout=1)
greyscale_image = image.convert('L')
histogram = greyscale_image.histogram()
pixels = sum(histogram)
br = scale = len(histogram)

for index in range(0, scale):
    ratio = histogram[index] / pixels
    br += ratio * (-scale + index)


output_voltage=2-((br/255)*2)
print("OUTPUT VOLTAGE",output_voltage,"V")
bright_out=br*(100/255)
sbc.set_brightness((120-bright_out), display=0)
while (True):
    if (br > 128):
        time.sleep(0.5)
        ser.write(b'output_voltage')

    else:
        time.sleep(0.5)
        ser.write(b'output_voltage')











