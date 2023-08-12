import RPi.GPIO as GPIO
    
in1=33 
in2=37 
en=35 


in3=36 
in4=38 
en2=12
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
#GPIO.setup(en,GPIO.OUT)
#GPIO.setup(en2,GPIO.OUT)
#pin20 = GPIO.PWM(en,100)
#pin21 = GPIO.PWM(en2,100)
def init_gpio():
    GPIO.setup(in1,GPIO.OUT)# Left
    GPIO.setup(in2,GPIO.OUT)
    

    GPIO.setup(in3,GPIO.OUT)# Right
    GPIO.setup(in4,GPIO.OUT)
    
    #pin20.start(0)
    #pin21.start(0)
    
    




def Forward():
    
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.HIGH) #LEFT
    
    GPIO.output(in3,GPIO.HIGH)
    GPIO.output(in4,GPIO.LOW) #RIGHT
    
  
  
def Right():
    
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.HIGH)
    
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.HIGH)
    
    
def Left():
    
    GPIO.output(in1,GPIO.HIGH)
    GPIO.output(in2,GPIO.LOW)

    GPIO.output(in3,GPIO.HIGH)
    GPIO.output(in4,GPIO.LOW)
    
def Stop():
 
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.LOW)
    #GPIO.output(en,GPIO.LOW)
    
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.LOW)
    #GPIO.output(en2,GPIO.HIGH)
    
    
    
