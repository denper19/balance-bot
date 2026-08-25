import RPi.GPIO as GPIO
import argparse

# constants
MOTOR1_A = 40
MOTOR1_B = 38
MOTOR2_A = 37
MOTOR2_B = 35
#MOTOR1_ENA
#MOTOR1_ENB
#MOTOR2_ENA
#MOTOR2_ENB


def main():
    
    parser = argparse.ArgumentParser(prog='motor_ctrl')
    parser.add_argument('-m1', '--motor1', help='set motor1 speed')
    parser.add_argument('-m2', '--motor2', help='set motor2 speed')

    args = parser.parse_args()

    motor1_speed = int(args.motor1)
    motor2_speed = int(args.motor2)

    print(f'speeds are motor1: {motor1_speed} motor2: {motor2_speed}')

    GPIO.setmode(GPIO.BOARD)
    
    GPIO.setup(MOTOR1_A, GPIO.OUT)
    GPIO.setup(MOTOR1_B, GPIO.OUT)
    GPIO.setup(MOTOR2_A, GPIO.OUT)
    GPIO.setup(MOTOR2_B, GPIO.OUT)

    GPIO.output(MOTOR1_A, GPIO.HIGH)
    GPIO.output(MOTOR1_B, GPIO.LOW)
    GPIO.output(MOTOR2_A, GPIO.HIGH)
    GPIO.output(MOTOR2_B, GPIO.LOW)

if __name__ == '__main__':
    main()
