import gpiod
from gpiod import Chip, Line
import time
import logging

logging.basicConfig(format='{%(asctime)s, %(levelname)s, %(filename)s:%(lineno)d}: %(message)s', datefmt='%Y-%m-%d:%H:%M:%S')
logger = logging.getLogger('drive')
logger.setLevel(logging.DEBUG)

# Define motor control pins (modify these based on your wiring)
IN1 = 17  # Enable pin for motor 1 (direction control)
IN2 = 22  # Enable pin for motor 1 (direction control)
ENA = 27  # Enable pin for motor 1 (speed control - PWM)
IN3 = 23  # Enable pin for motor 2 (direction control)
IN4 = 24  # Enable pin for motor 2 (direction control)
ENB = 25  # Enable pin for motor 2 (speed control - PWM)

def init():
    logger.debug('movement.init()')

    chip = {}

    # Open GPIO chip
    chip['chip'] = gpiod.Chip('gpiochip4')
    # Configure motor control pins as outputs
    chip['motor1_in1'] = chip['chip'].get_line(IN1)
    chip['motor1_in2'] = chip['chip'].get_line(IN2)
    chip['motor1_ena'] = chip['chip'].get_line(ENA)
    chip['motor2_in3'] = chip['chip'].get_line(IN3)
    chip['motor2_in4'] = chip['chip'].get_line(IN4)
    chip['motor2_enb'] = chip['chip'].get_line(ENB)

    # Set direction as output
    chip['motor1_in1'].request(consumer="Motor1_IN1", type=gpiod.LINE_REQ_DIR_OUT)
    chip['motor1_in2'].request(consumer="Motor1_IN2", type=gpiod.LINE_REQ_DIR_OUT)
    chip['motor1_ena'].request(consumer="Motor1_ENA", type=gpiod.LINE_REQ_DIR_OUT) 
    chip['motor2_in3'].request(consumer="Motor2_IN3", type=gpiod.LINE_REQ_DIR_OUT)
    chip['motor2_in4'].request(consumer="Motor2_IN4", type=gpiod.LINE_REQ_DIR_OUT)
    chip['motor2_enb'].request(consumer="Motor2_ENA", type=gpiod.LINE_REQ_DIR_OUT)

    return chip

def test(chip):
    logger.debug('movement.test()')

    chip['motor1_in1'].set_value(0)
    chip['motor1_in2'].set_value(1)
    chip['motor2_in3'].set_value(0)
    chip['motor2_in4'].set_value(1)
    chip['motor1_ena'].set_value(1)
    chip['motor2_enb'].set_value(1)
    time.sleep(.01)
    stop(chip)

def step(chip, direction, duty_cycle, duration, freq):
    logger.debug('movement.step(): direction: {}, duty_cycle: {}, duration: {}, freq: {}'.format(direction, duty_cycle, duration, freq))

    chip['motor1_in1'].set_value(direction[0])
    chip['motor1_in2'].set_value(direction[1])
    chip['motor2_in3'].set_value(direction[2])
    chip['motor2_in4'].set_value(direction[3])

    sleep_1 = (1/freq)*(duty_cycle/1000)
    sleep_2 = (1/freq)*(1 - (duty_cycle/1000))

    start_time = time.time()
    while time.time() - start_time < duration:
        chip['motor1_ena'].set_value(1)
        chip['motor2_enb'].set_value(1)
        time.sleep(sleep_1)
        chip['motor1_ena'].set_value(0)
        chip['motor2_enb'].set_value(0)
        time.sleep(sleep_2)

    stop(chip)

def stop(chip):
    logger.debug('movement.stop()')

    """Stops both motors"""
    chip['motor1_in1'].set_value(0)
    chip['motor1_in2'].set_value(0)
    chip['motor2_in3'].set_value(0)
    chip['motor2_in4'].set_value(0)
    chip['motor1_ena'].set_value(0)
    chip['motor2_enb'].set_value(0)

def demo():
    logger.debug('movement.demo()')

    duty = 100
    frequency = 100
    steptime = 0.1

    while True:
        wasd = input("Enter a direction with W,A,S,D or C for configure:  ")
        if wasd == "s":
            movement = [0,1,0,1]
        if wasd == "w":
            movement = [1,0,1,0]
        if wasd == "d":
            movement = [0,1,1,0]
        if wasd == "a":
            movement = [1,0,0,1]
        if wasd == "c":
            # duty = int(input("Dutycycle:    "))
            # frequency = int(input("Frequency:   "))
            steptime = int(input("steptime:   "))
            movement = [0,0,0,0]
        if wasd == "t":
            test()
            movement = [0000]

        step(movement, duty, steptime, frequency)

# Clean up GPIO resources (optional but recommended)
def close(chip):
    logger.debug('movement.close()')

    chip['motor1_in1'].release()
    chip['motor1_in2'].release()
    chip['motor1_ena'].release()
    chip['motor2_in3'].release()
    chip['motor2_in4'].release()
    chip['motor2_enb'].release()
    chip['chip'].close()
