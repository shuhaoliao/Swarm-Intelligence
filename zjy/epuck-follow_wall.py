# 避障+绕行
# 绕行对于多个障碍物的复杂情况还不能很好地适应，稳定性也有问题。
"""e-puck_gps controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, DistanceSensor, Motor, GPS, Gyro

def moveForward():
    leftSpeed  = 0.5 * MAX_SPEED
    rightSpeed = 0.5 * MAX_SPEED
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)

def IsopenAhead():
    D0 = psValues[0]
    D7 = psValues[7]
    D1 = psValues[1]
    D6 = psValues[6]
    if D0 >= Dwth and D7 >= Dwth:
        return False
    elif D1 >= Dwth*2 or D6 >= Dwth*2 :
        return False
    else:
        return True

def wallToRight():
    if psValues[2] > 0.2*Dwth:
        return True
    else:
        return False

def followWall(eprev):
    w_d2 = 0.7
    ecurr = (w_d2*psValues[2]+(1-w_d2)*psValues[1])-Dwth
    ep = ecurr
    ed = ecurr - eprev
    ei = ecurr + eprev

    Kp = 0.2
    Kd = 0.6
    Ki = 0.05

    leftSpeed = 0.5*MAX_SPEED
    rightSpeed = 0.5*MAX_SPEED + (Kp*ep+Kd*ed+Ki*ei)/20
    if rightSpeed > MAX_SPEED:
        rightSpeed = MAX_SPEED
    elif rightSpeed < -1*MAX_SPEED:
        rightSpeed =-1*MAX_SPEED
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)

def turn90ccw():
    leftSpeed = -0.5*MAX_SPEED
    rightSpeed = 0.5*MAX_SPEED
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    time90 = 12
    for i in range(time90):
        robot.step(timestep)
    updatePsValues()
    print("turn 90")

def turnRightToWall():
    # turn left
    leftSpeed = -0.5 * MAX_SPEED
    rightSpeed = 0.5 * MAX_SPEED
    if psValues[0] < psValues[3]:
        leftSpeed = 0.5 * MAX_SPEED
        rightSpeed = -0.5 * MAX_SPEED
    isUpdate = 1
    closedDistance = 0
    while isUpdate:
        print("turn right to wall", psValues[2])
        leftMotor.setVelocity(leftSpeed)
        rightMotor.setVelocity(rightSpeed)
        robot.step(timestep)
        updatePsValues()
        if psValues[2] > closedDistance:
            isUpdate = 1
            closedDistance = psValues[2]
        elif psValues[2] < Dwth:
            continue
        else:
            break


# create the Robot instance.
robot = Robot()
gps = GPS(name="e-puck-gps")
# get the time step of the current world.
timestep = 64
MAX_SPEED = 6.28
Dwth = 160
w_d2 = 0.7
time90 = 12
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)
ps = []
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]

for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(timestep)

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
gyro = robot.getDevice('gyro')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

gps.enable(timestep)
gps.LOCAL = gps.getCoordinateSystem()
gyro.enable(timestep)
# Main loop:
# - perform simulation steps until Webots is stopping the controller
psValues = []
for i in range(8):
    psValues.append(ps[i].getValue())
ecurr = 0
eprev = 0
def updatePsValues():
    for i in range(8):
        psValues[i] = ps[i].getValue()

while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    # read sensors outputs
    # Process sensor data here.
    updatePsValues()
    if IsopenAhead():
        moveForward()
    else:
        turnRightToWall()
        # turn90ccw()
        while IsopenAhead():
            eprev = ecurr
            followWall(eprev)
            if not wallToRight():
                print("No obstacle")
                followWall(ecurr)
                if not wallToRight():
                    break
            ecurr = (w_d2*psValues[2]+(1-w_d2)*psValues[1])-Dwth
            robot.step(timestep)
            updatePsValues()
    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    # print(gps.getValues())
    # print(gyro.getValues())
# Enter here exit cleanup code.
