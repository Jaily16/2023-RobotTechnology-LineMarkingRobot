"""
    use python to write controller
"""

from controller import *
import enum

# const define
MAX_SPEED = 5.24
MAX_SENSOR_NUMBER = 16
DELAY = 70
MAX_SENSOR_VALUE = 1024
MIN_DISTANCE = 1.0
WHEEL_WEIGHT_THRESHOLD = 100


class SensorData:
    def __init__(self, w1, w2):
        self.device_tag = None
        self.wheel_weight = [w1, w2]


class State(enum.Enum):
    FORWARD = 1
    LEFT = 2
    RIGHT = 3


sensors = [SensorData(150, 0), SensorData(200, 0), SensorData(300, 0), SensorData(600, 0),
           SensorData(0, 600), SensorData(0, 300), SensorData(0, 200), SensorData(0, 150),
           SensorData(0, 0), SensorData(0, 0), SensorData(0, 0), SensorData(0, 0),
           SensorData(0, 0), SensorData(0, 0), SensorData(0, 0), SensorData(0, 0)]

# necessary to initialize Webots
robot = Robot()

# stores simulation time step
timestep = int(robot.getBasicTimeStep())

# stores device IDs for the wheels
left_wheel = robot.getDevice('left wheel')
right_wheel = robot.getDevice('right wheel')

camera = robot.getDevice('camera')
camera.enable(timestep)

# stores device IDs for the LEDs
red_led = [robot.getDevice('red led 1'), robot.getDevice('red led 2'), robot.getDevice('red led 3')]

# sets up sensors and stores some info about them
for i in range(MAX_SENSOR_NUMBER):
    sensors[i].device_tag = robot.getDevice('so' + str(i))
    sensors[i].device_tag.enable(timestep)

# sets up wheels
left_wheel.setPosition(float('inf'))
right_wheel.setPosition(float('inf'))
left_wheel.setVelocity(0.0)
right_wheel.setVelocity(0.0)

led_number = 0
delay = 0
speed = [0.0, 0.0]
wheel_weight_total = [0.0, 0.0]
distance = 0.0
speed_modifier = 0.0
sensor_value = 0.0

# by default, the robot goes forward
state = State.FORWARD

while robot.step(timestep) != -1:

    speed = [0.0, 0.0]
    wheel_weight_total = [0.0, 0.0]

    for i in range(MAX_SENSOR_NUMBER):
        sensor_value = sensors[i].device_tag.getValue()
        # if the sensor doesn't see anything, we don't use it for this round
        if sensor_value == 0.0:
            speed_modifier = 0.0
        else:
            distance = 5.0 * (1.0 - (sensor_value / MAX_SENSOR_VALUE))
            if distance < MIN_DISTANCE:
                speed_modifier = 1 - (distance / MIN_DISTANCE)
            else:
                speed_modifier = 0.0

        for j in range(2):
            wheel_weight_total[j] += sensors[i].wheel_weight[j] * speed_modifier

    if state == State.FORWARD:
        if wheel_weight_total[0] > WHEEL_WEIGHT_THRESHOLD:
            speed[0] = 0.7 * MAX_SPEED
            speed[1] = -0.7 * MAX_SPEED
            state = State.LEFT
        elif wheel_weight_total[1] > WHEEL_WEIGHT_THRESHOLD:
            speed[0] = -0.7 * MAX_SPEED
            speed[1] = 0.7 * MAX_SPEED
            state = State.RIGHT
        else:
            speed[0] = MAX_SPEED
            speed[1] = MAX_SPEED
    elif state == State.LEFT:
        if wheel_weight_total[0] > WHEEL_WEIGHT_THRESHOLD or wheel_weight_total[1] > WHEEL_WEIGHT_THRESHOLD:
            speed[0] = 0.7 * MAX_SPEED
            speed[1] = -0.7 * MAX_SPEED
        else:
            speed[0] = MAX_SPEED
            speed[1] = MAX_SPEED
            state = State.FORWARD
    else:
        if wheel_weight_total[0] > WHEEL_WEIGHT_THRESHOLD or wheel_weight_total[1] > WHEEL_WEIGHT_THRESHOLD:
            speed[0] = -0.7 * MAX_SPEED
            speed[1] = 0.7 * MAX_SPEED
        else:
            speed[0] = MAX_SPEED
            speed[1] = MAX_SPEED
            state = State.FORWARD

    delay += 1
    if delay == DELAY:
        red_led[led_number].set(0)
        led_number += 1
        led_number = led_number % 3
        red_led[led_number].set(1)
        delay = 0

    left_wheel.setVelocity(speed[0])
    right_wheel.setVelocity(speed[1])
