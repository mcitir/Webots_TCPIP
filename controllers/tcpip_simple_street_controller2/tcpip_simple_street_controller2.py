
#!/usr/binenv python3

import socket
import select
import errno
import sys

from vehicle import Driver


sensorsNames = [
    "front",
    "front right 0",
    "front right 1",
    "front right 2",
    "front left 0",
    "front left 1",
    "front left 2",
    "rear",
    "rear left",
    "rear right",
    "right",
    "left"]

sensors = {}

#lanePositions = [-10.6, -6.875, -3.2]
lanePositions = [5.8, 1.8, -2.2, -6.0]
currentLane = 1
overtakingSide = None
maxSpeed = 80
safeOvertake = False


def apply_PID(position, targetPosition):
    """Apply the PID controller and return the angle command."""
    P = 0.05
    I = 0.000015
    D = 25
    diff = position - targetPosition
    if apply_PID.previousDiff is None:
        apply_PID.previousDiff = diff
    # anti-windup mechanism
    if diff > 0 and apply_PID.previousDiff < 0:
        apply_PID.integral = 0
    if diff < 0 and apply_PID.previousDiff > 0:
        apply_PID.integral = 0
    apply_PID.integral += diff
    # compute angle
    angle = P * diff + I * apply_PID.integral + \
        D * (diff - apply_PID.previousDiff)
    apply_PID.previousDiff = diff

    # print("PID angle: " + str(angle))  # mc
    return angle


apply_PID.integral = 0
apply_PID.previousDiff = None


def get_filtered_speed(speed):
    """Filter the speed ommand to avoid abrupt speed changes."""
    get_filtered_speed.previousSpeeds.append(speed)
    if len(get_filtered_speed.previousSpeeds) > 100:  # keep only 80 values
        get_filtered_speed.previousSpeeds.pop(0)
    return sum(get_filtered_speed.previousSpeeds) / float(len(get_filtered_speed.previousSpeeds))


def is_vehicle_on_side(side):
    """Check (using the 3 appropriated front distance sensors) if there is a car in front."""
    for i in range(3):
        name = "front " + side + " " + str(i)
        if sensors[name].getValue() > 0.8 * sensors[name].getMaxValue():
            return True
    return False


def reduce_speed_if_vehicle_on_side(speed, side):
    """Reduce the speed if there is some vehicle on the side given in argument."""
    minRatio = 1
    for i in range(3):
        name = "front " + overtakingSide + " " + str(i)
        ratio = sensors[name].getValue() / sensors[name].getMaxValue()
        if ratio < minRatio:
            minRatio = ratio
    return minRatio * speed


get_filtered_speed.previousSpeeds = []
driver = Driver()


# Define sensors
for name in sensorsNames:
    sensors[name] = driver.getDevice("distance sensor " + name)
    sensors[name].enable(10)  # Sampling Period to read data from sensors

gps = driver.getDevice("gps")
gps.enable(10)

#camera = driver.getDevice("camera")
# uncomment those lines to enable the camera
# camera.enable(10)
# camera.recognitionEnable(50)

# Define a socket connection
HEADER_LENGTH = 10
IP = socket.gethostbyname(socket.gethostname())
PORT = 5050

my_username = str(driver.getName())  # input("Username: ")
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((IP, PORT))
client_socket.setblocking(False)

username = my_username.encode("utf-8")
username_header = f"{len(username):<{HEADER_LENGTH}}".encode("utf-8")
client_socket.send(username_header + username)

while driver.step() != -1:

    # adjust speed according to front vehicle
    frontDistance = sensors["front"].getValue()
    leftDistance = sensors["left"].getValue()  # mc
    rightDistance = sensors["right"].getValue()  # mc
    # print("Front Distance: " + str(frontDistance))  # mc
    # print("Left Distance: " + str(leftDistance))  # mc
    # print("Right Distance: " + str(rightDistance))  # mc
    # try:
    #     print("Speed: " + str(speed))  # mc
    # except:
    #     print("No Speed")
    frontRange = sensors["front"].getMaxValue()
    speed = maxSpeed * frontDistance / frontRange
    # print("Speed Calc: " + speed) #mc
    if sensors["front right 0"].getValue() < 8.0 or sensors["front left 0"].getValue() < 8.0:
        # another vehicle is currently changing lane in front of the vehicle => emergency braking
        speed = min(0.5 * maxSpeed, speed)
    if overtakingSide is not None:
        # check if overtaking should be aborted
        if overtakingSide == 'right' and sensors["left"].getValue() < 0.8 * sensors["left"].getMaxValue():
            overtakingSide = None
            currentLane -= 1
        elif overtakingSide == 'left' and sensors["right"].getValue() < 0.8 * sensors["right"].getMaxValue():
            overtakingSide = None
            currentLane += 1
        else:  # reduce the speed if the vehicle from previous lane is still in front
            speed2 = reduce_speed_if_vehicle_on_side(speed, overtakingSide)
            if speed2 < speed:
                speed = speed2
    speed = get_filtered_speed(speed)
    # print("Speed Set: " + str(speed)) #mc
    driver.setCruisingSpeed(speed)
    # brake if needed
    speedDiff = driver.getCurrentSpeed() - speed
    if speedDiff > 0:
        driver.setBrakeIntensity(min(speedDiff / speed, 1))
    else:
        driver.setBrakeIntensity(0)
    # car in front, try to overtake
    if frontDistance < 0.8 * frontRange and overtakingSide is None:
        if (is_vehicle_on_side("left") and
                (not safeOvertake or sensors["rear left"].getValue() > 0.8 * sensors["rear left"].getMaxValue()) and
                sensors["left"].getValue() > 0.8 * sensors["left"].getMaxValue() and
                currentLane < 2):
            currentLane += 1
            overtakingSide = 'right'
        elif (is_vehicle_on_side("right") and
                (not safeOvertake or sensors["rear right"].getValue() > 0.8 * sensors["rear right"].getMaxValue()) and
                sensors["right"].getValue() > 0.8 * sensors["right"].getMaxValue() and
                currentLane > 0):
            currentLane -= 1
            overtakingSide = 'left'
    # adjust steering to stay in the middle of the current lane
    position = gps.getValues()[0]
    # print("Position GPS: " + str(position))  # mc
    angle = max(
        min(apply_PID(position, lanePositions[currentLane]), 0.5), -0.5)
    # print("Angle: " + str(angle))
    # print("Current Lane: " + str(currentLane))
    driver.setSteeringAngle(angle)
    # check if overtaking is over
    # if abs(position - lanePositions[currentLane]) < 1.5:  # the car is just in the lane
    #    overtakingSide = None

# .............................................................................
# TCP / IP Socket Connection Start.............................................
    # input(f"{my_username} > ")
    message = f"Position > {gps.getValues()}"

    if message:
        message = message.encode("utf-8")
        message_header = f"{len(message):<{HEADER_LENGTH}}".encode("utf-8")
        client_socket.send(message_header + message)

    try:
        username_header = client_socket.recv(HEADER_LENGTH)
        if not len(username_header):
            print("connection closed by the server")
            # sys.exit()

        username_length = int(username_header.decode('utf-8').strip())
        username = client_socket.recv(username_length).decode('utf-8')

        message_header = client_socket.recv(HEADER_LENGTH)
        message_length = int(message_header.decode('utf-8').strip())
        message = client_socket.recv(message_length).decode('utf-8')

        print(f"{username} > {message}")

    except IOError as e:
        if e.errno != errno.EAGAIN and e.errno != errno.EWOULDBLOCK:
            print('Reading error', str(e))
            # sys.exit()
        continue

    except Exception as e:
        print('General error', str(e))
        # sys.exit()


# TCP / IP Socket Connection End...............................................
# .............................................................................

client_socket.close()
