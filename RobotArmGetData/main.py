#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor
from pybricks.parameters import Port, Direction, Stop
from pybricks.tools import wait
from pybricks.messaging import BluetoothMailboxServer, TextMailbox

# Create your objects here.
ev3 = EV3Brick()


# Configure the gripper motor on Port A with default settings.
gripper_motor = Motor(Port.A)

# Configure the elbow motor. It has an 8-teeth and a 40-teeth gear
# connected to it. We would like positive speed values to make the
# arm go upward. This corresponds to counterclockwise rotation
# of the motor.
elbow_motor = Motor(Port.B, Direction.COUNTERCLOCKWISE, [8, 40])

# Configure the motor that rotates the base. It has a 12-teeth and a
# 36-teeth gear connected to it. We would like positive speed values
# to make the arm go away from the Touch Sensor. This corresponds
# to counterclockwise rotation of the motor.
base_motor = Motor(Port.C, Direction.COUNTERCLOCKWISE, [12, 36])

# Limit the elbow and base accelerations. This results in
# very smooth motion. Like an industrial robot.
elbow_motor.control.limits(speed=60, acceleration=120)
base_motor.control.limits(speed=60, acceleration=120)

# Set up the Touch Sensor. It acts as an end-switch in the base
# of the robot arm. It defines the starting point of the base.
base_switch = TouchSensor(Port.S1)

# Set up the Color Sensor. This sensor detects when the elbow
# is in the starting position. This is when the sensor sees the
# white beam up close.
elbow_sensor = ColorSensor(Port.S3)

# Initialize the elbow. First make it go down for one second.
# Then make it go upwards slowly (15 degrees per second) until
# the Color Sensor detects the white beam. Then reset the motor
# angle to make this the zero point. Finally, hold the motor
# in place so it does not move.
elbow_motor.run_time(-30, 1000)
elbow_motor.run(15)
while elbow_sensor.reflection() < 32:
    wait(10)
elbow_motor.reset_angle(0)
elbow_motor.hold()

# Initialize the base. First rotate it until the Touch Sensor
# in the base is pressed. Reset the motor angle to make this
# the zero point. Then hold the motor in place so it does not move.
base_motor.run(-60)
while not base_switch.pressed():
    wait(10)
base_motor.reset_angle(0)
base_motor.hold()

# Initialize the gripper. First rotate the motor until it stalls.
# Stalling means that it cannot move any further. This position
# corresponds to the closed position. Then rotate the motor
# by 90 degrees such that the gripper is open.
gripper_motor.run_until_stalled(200, then=Stop.COAST, duty_limit=50)
gripper_motor.reset_angle(0)
gripper_motor.run_target(200, -90)

# Write your program here.
ev3.speaker.beep()

def convertValue(value):
    # Pastikan nilai berada dalam rentang 0 hingga 100
    if 0 <= value <= 100:
        return -0.9 * value  # Konversi ke rentang 0 hingga -90
    else:
        raise ValueError("Nilai harus berada dalam rentang 0 hingga 100")

# --- Loop Utama ---
dataFromServer = ""  # Inisialisasi string_data
server = BluetoothMailboxServer()
mbox = TextMailbox("greeting", server)

# The server must be started before the client!
print("waiting for connection...")
ev3.speaker.say('waiting for connection')

server.wait_for_connection()
print("connected!")
ev3.speaker.say('connected!')

#openingValue = -90

# Loop utama
while True:
    # Menunggu data dari client
    mbox.wait()
    dataFromServer = mbox.read()

    # Memisahkan data berdasarkan koma
    try:
        # Memecah data yang diterima dari client
        dataParts = dataFromServer.split(',')
        if len(dataParts) == 3:
            gripperOpen = int(dataParts[0].strip())  # Mengambil nilai gripperOpen
            horizontalMovement = int(dataParts[1].strip())  # Mengambil nilai horizontalMovement
            verticalMovement = int(dataParts[2].strip())  # Mengambil nilai verticalMovement
            
            #set nilai data ke motor
            
            gripper_motor.run_target(90,gripperOpen)
            if gripper_motor.stalled():
                gripper_motor.hold()

            base_motor.run_target(80,horizontalMovement)
            if base_motor.stalled():
                base_motor.stop()

            elbow_motor.run_target(80,verticalMovement)
            if elbow_sensor.reflection() < 32:
                wait(10)
                elbow_motor.hold()

            # Print data ke layar EV3
            ev3.screen.clear()
            ev3.screen.print(gripperOpen)
            ev3.screen.print(horizontalMovement)
            ev3.screen.print(verticalMovement)

        else:
            ev3.screen.print("Invalid data format")

    except ValueError:
        ev3.screen.print("Error parsing data")
    
    wait(100)  # Delay for 100ms to avoid rapid processing