import gpiod
from gpiod import Chip, Line
import time

# Define motor control pins (modify these based on your wiring)
IN1 = 17  # Enable pin for motor 1 (direction control)
IN2 = 22  # Enable pin for motor 1 (direction control)
ENA = 27  # Enable pin for motor 1 (speed control - PWM)
IN3 = 23  # Enable pin for motor 2 (direction control)
IN4 = 24  # Enable pin for motor 2 (direction control)
ENB = 25  # Enable pin for motor 2 (speed control - PWM)

# Open GPIO chip
chip = gpiod.Chip('gpiochip4')

# Configure motor control pins as outputs
motor1_in1 = chip.get_line(IN1)
motor1_in2 = chip.get_line(IN2)
motor1_ena = chip.get_line(ENA)
motor2_in3 = chip.get_line(IN3)
motor2_in4 = chip.get_line(IN4)
motor2_enb = chip.get_line(ENB)

# Set direction as output
motor1_in1.request(consumer="Motor1_IN1", type=gpiod.LINE_REQ_DIR_OUT)
motor1_in2.request(consumer="Motor1_IN2", type=gpiod.LINE_REQ_DIR_OUT)
motor1_ena.request(consumer="Motor1_ENA", type=gpiod.LINE_REQ_DIR_OUT) 
motor2_in3.request(consumer="Motor2_IN3", type=gpiod.LINE_REQ_DIR_OUT)
motor2_in4.request(consumer="Motor2_IN4", type=gpiod.LINE_REQ_DIR_OUT)
motor2_enb.request(consumer="Motor2_ENA", type=gpiod.LINE_REQ_DIR_OUT)

def test():
  motor1_in1.set_value(0)
  motor1_in2.set_value(1)
  motor2_in3.set_value(0)
  motor2_in4.set_value(1)
  motor1_ena.set_value(1)
  motor2_enb.set_value(1)
  time.sleep(2)
  stop()

def step(direction, dutycycle, duration, freq):
  #print("movement is {}{}{}{}".format(direction[0],direction[1],direction[2],direction[3]))
  motor1_in1.set_value(direction[0])
  motor1_in2.set_value(direction[1])
  motor2_in3.set_value(direction[2])
  motor2_in4.set_value(direction[3])
  sleep1= (1/freq)*(dutycycle/1000)
  sleep2= (1/freq)*(1-(dutycycle/1000))
  start_time = time.time()
  while time.time() - start_time < duration:
    motor1_ena.set_value(1)
    motor2_enb.set_value(1)
    time.sleep(sleep1)
    motor1_ena.set_value(0)
    motor2_enb.set_value(0)
    time.sleep(sleep2)
  stop()

def stop():
  """Stops both motors"""
  motor1_in1.set_value(0)
  motor1_in2.set_value(0)
  motor2_in3.set_value(0)
  motor2_in4.set_value(0)
  motor1_ena.set_value(0)
  motor2_enb.set_value(0)

duty=200
frequency=50
steptime=0.2
while True:
  wasd=input("Enter a direction with W,A,S,D or C for configure:  ")
  if wasd == "w":
    movement=[0,1,0,1]
  if wasd == "s":
    movement=[1,0,1,0]
  if wasd == "a":
    movement=[0,1,1,0]
  if wasd == "d":
    movement=[1,0,0,1]
  if wasd == "c":
    duty=int(input("Dutycycle:    "))
    frequency=int(input("Frequency:   "))
    steptime=int(input("steptime:   "))
    movement=[0,0,0,0]
  if wasd == "t":
    test()
    movement=[0000]
  print("movement is {}{}{}{}".format(movement[0],movement[1],movement[2],movement[3]))  
  step(movement,duty,steptime,frequency)

# Add more functions for other motor controls (e.g., left turn, right turn)

# Clean up GPIO resources (optional but recommended)
motor1_in1.release()
motor1_in2.release()
motor1_ena.release()
motor2_in3.release()
motor2_in4.release()
motor2_enb.release()
chip.close()
