from picamera2 import Picamera2 #Raspberry Pi HQ camera library
import common as cm #For detection
import cv2 #for processing the image and draw boxes
import numpy as np 
from PIL import Image
import time
from threading import Thread
import motorv1 as ut #Motors file
import serial
import RPi.GPIO as GPIO
from gpiozero import Servo
import math
from time import sleep
from gpiozero.pins.pigpio import PiGPIOFactory

#########################################################################

#reading from Arduino serial port
import serial
from threading import Thread

def sensor():
    global distance
    ser = serial.Serial('/dev/ttyUSB0')

    input_str = ser.readline()
    input_str= ser.readline().decode("utf-8").strip()

    while True:
        
        input_str= ser.readline().decode("utf-8").strip()
        if float(input_str) < 1000:
            distance = float(input_str)


        
sensor_thread = Thread(target = sensor)
sensor_thread.start()


#Servo motor pins initialization
OUT_PIN = 12
PULSE_FREQ = 50
env = 1
GPIO.setmode(GPIO.BOARD)
GPIO.setup(OUT_PIN, GPIO.OUT)


#Initializing Pi camera
cv2.startWindowThread()
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
picam2.start()
cap = picam2.capture_array()


#Hyperparameter for Object detection
threshold=0.5
top_k=4 #number of objects to be shown as detected
edgetpu= 1
object_to_track='person'

#Model files directories
model_dir = '/home/pi/Project2'
model_edgetpu = 'edgetpu.tflite'
lbl = 'coco_labels.txt'


#Parameters for person movement in frame
tolerance=0.12
x_deviation=0
distance= 0
x_obj_center = 0

#----------------------initialize motor speed--------------------------
ut.init_gpio()
GPIO.setup(35, GPIO.OUT)# set GPIO 35 as output pin35
GPIO.setup(12, GPIO.OUT)# set GPIO 12 as output pin12
val=100 # maximum speed

pin35 = GPIO.PWM(35, val)    # create object pin35 for PWM on port 35 at 100 Hertz  
pin12 = GPIO.PWM(12, val)    # create object pin12 for PWM on port 12 at 100 Hertz  

pin35.start(100)              # start pin35 on 0 percent duty cycle (off)  
pin12.start(100)              # start pin12 on 0 percent duty cycle (off)  
    
print("speed set to: ", val)


#-----------------Function to track the person---------------------------
def track_object(objs,labels):
    
    global x_deviation, tolerance, x_min, x_max, x_obj_center
    
    
    if(len(objs)==0):
        #print("no objects to track")
        ut.Stop()
        return

    flag=0
    for obj in objs:
        lbl=labels.get(obj.id)
        if (lbl==object_to_track):
            x_min, y_min, x_max, y_max = list(obj.bbox)
            flag=1
            break
        
    if(flag==0):
        #print("selected object no present")
        return
        
    x_diff=x_max-x_min
    y_diff=y_max-y_min

    x_obj_center = x_min*640+(x_max*640-x_min*640)/2
         
    obj_x_center=x_min+(x_diff/2)
    obj_x_center=round(obj_x_center,3)
    
    obj_y_center=y_min+(y_diff/2)
    obj_y_center=round(obj_y_center,3)
    
    x_deviation=round(0.5-obj_x_center,3)
        
    thread = Thread(target = move_robot)
    thread.start()
    
    
    
#----------------------Function to move the robot------------------------
def move_robot():
    global x_deviation, tolerance, distance
    min_D = 100.0
    if(abs(x_deviation)<tolerance):
        if( int(distance) <= min_D):
            ut.Stop()
            print(f"reached person........... {distance}")
        else:
            ut.Forward()
            print(f"moving robot ...Forward....")
    
    else:
       
        if(x_deviation>=tolerance):
            delay1=get_delay(x_deviation)
                
            ut.Left()
            time.sleep(delay1)
            ut.Stop()
            print("moving robot ...Left....<<<<<<<<<<")
    
                
        if(x_deviation<=-1*tolerance):
            delay1=get_delay(x_deviation)
                
            ut.Right()
            time.sleep(delay1)
            ut.Stop()
            print("moving robot ...Right....>>>>>>>>")
        
            
        
def get_delay(deviation):
    
    deviation=abs(deviation)
    
    if(deviation>=0.4):
        d=0.17 #0.15
    elif(deviation>=0.35 and deviation<0.40):
        d=0.15 #0.050
    elif(deviation>=0.20 and deviation<0.35):
        d=0.1 #0.040
        
    else:
        d=0.08
    
    
    return d


def init_servo():
    global servo
        
    

    factory = PiGPIOFactory()

    servo = Servo(12, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, pin_factory=factory)


def move_servo():
    global servo, x_obj_center
        
    while True:
        
        if  x_obj_center < 150:
            servo.value = math.sin(math.radians(30))
        elif  x_obj_center > 450:
            servo.value = math.sin(math.radians(-30))
        
        else:
            servo.value = math.sin(math.radians(0))


def main():
    global x_obj_center
    
    init_servo()
    servo_thread = Thread(target = move_servo)
    servo_thread.start()
    interpreter, labels =cm.load_model(model_dir,model_edgetpu,lbl,edgetpu)
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
    height = input_details[0]['shape'][1]
    width = input_details[0]['shape'][2]
    
    while True:
        #env = input()
        #----------------Capture Camera Frame-----------------
        frame1 = picam2.capture_array()
        cv2_im = frame1
        cv2_im_rgb = cv2.cvtColor(cv2_im, cv2.COLOR_BGR2RGB)
        frame = cv2.resize(cv2_im_rgb, (width, height))
        input_data = np.expand_dims(frame, axis=0)
        pil_im = Image.fromarray(frame)
    
       
        #-------------------Inference---------------------------------
        cm.set_input(interpreter, pil_im)
        interpreter.invoke()
        objs = cm.get_output(interpreter, score_threshold=threshold, top_k=top_k)
        
        #-----------------other------------------------------------
        track_object(objs,labels)#tracking  <<<<<<<

        for obj in objs:
            xmin = int(max(1,(obj.bbox[0] * 640)))
            ymin = int(max(1,(obj.bbox[1] * 480)))
            xmax = int(min(640,(obj.bbox[2] * 640)))
            ymax = int(min(480,(obj.bbox[3] * 480)))
            cv2.rectangle(frame1, (xmin,ymin), (xmax,ymax), (0, 255, 0), 2)
            cv2.imshow('Object detector', frame1)
            

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

