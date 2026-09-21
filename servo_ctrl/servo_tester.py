import Jetson.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
SERVO_PIN = 33 
GPIO.setup(SERVO_PIN, GPIO.OUT)

# Initialize at 50Hz to bypass the Jetson Orin kernel limitation
pwm = GPIO.PWM(SERVO_PIN, 50) 
pwm.start(7.5) # Start at 90 degrees (1.5ms pulse = 30% duty cycle)

def set_servo_angle_50hz(angle):
    angle = max(0, min(90, angle))
    
    # 10% duty cycle = 0 degrees, 50% = 180 degrees
    # 2.5% = 0 degrees, 7.5% = 180 degrees
    duty_cycle = 2.5 + (angle / 90.0) * 5.0
    
    pwm.ChangeDutyCycle(duty_cycle)
    print(f"Moving to {angle} degrees (Duty Cycle: {duty_cycle:.2f}%)")
    time.sleep(0.5)

try:
    print("Starting 50Hz servo sweep. Press Ctrl+C to stop.")
    while True:
        set_servo_angle_50hz(0)
        time.sleep(1)
        set_servo_angle_50hz(90)
        time.sleep(1)

except KeyboardInterrupt:
    print("\nProgram interrupted.")

finally:
    pwm.stop()
    GPIO.cleanup()