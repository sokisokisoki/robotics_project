import time
import numpy as np

from Arm_Lib import Arm_Device

Arm = Arm_Device()
time.sleep(2)

# Fixed path for arm to follow, determined by hand
q_l = [
    [180.0, 45.0, 60.0, 50.0, 0.0, 0.0],
    [135.0, 50.0, 60.0, 25.0, 0.0, 180.0],
    [135.0, 70.0, 60.0, 25.0, 0.0, 180.0],
    [100.0, 70.0, 60.0, 25.0, 0.0, 180.0],
    [100.0, 60.0, 60.0, 25.0, 0.0, 180.0],
    [100.0, 60.0, 60.0, 25.0, 0.0, 0.0],
    [100.0, 75.0, 60.0, 35.0, 0.0, 0.0],
    [85.0, 75.0, 60.0, 35.0, 0.0, 0.0],
    [85.0, 45.0, 60.0, 35.0, 0.0, 180.0],
    [85.0, 60.0, 60.0, 35.0, 0.0, 180.0],
    [60.0, 60.0, 60.0, 35.0, 0.0, 180.0],
]

def main():
    speedtime = 1000 #time in milliseconds to reach desired joint position
    
    q = q_l
    for step in range(len(q)):
        for jnum in range(len(q[0])):
            ang = q[step][jnum]
            moveJoint(jnum + 1,ang,speedtime)
            time.sleep(0.5)
        time.sleep(0.5)
    q = readAllActualJointAngles()
    print(q)
    
    print("Program Ended")
    
def getJointNumber():
    """
    function used to get the desired joint number using keyboard input
    getJointNumber() requests user input the desired joint number and returns joint number as an integer
    """
    jnum = int(input("Input joint number")) #ask the user to input a joint number. int converts the input to an integer
    print("Joint number: ",jnum) #print out the joint number that was read
    #if the joint number is not valid, keep prompting until a valid number is given
    if jnum<0 or jnum>6:
        while True:
            jnum = int(input("Input valid joint number [1,6]"))
            if jnum>=0 and jnum<=6:
                break
    return jnum #return the read value to the main function

def getJointAngle(jnum):
    """
    function used to get the desired joint angle using keyboard input
    getJointAngle() requests user input the desired joint angle in degrees and returns joint angle as an integer
    function needs to know the target joint (jnum) because joint 5 has a different angle range than the other joints
    """
    ang = float(input("Input angle (degrees)")) #ask the user to input a joint angle in degrees. int converts the input to an integer
    print("Joint angle: ",ang) #print out the joint angle that was read
    #if the joint angle is not valid, keep prompting until a valid number is given   
    if jnum != 5: #range for all joints except 5 is 0 to 180 degrees
        if ang<0 or ang>180:
            while True:
                ang = float(input("Input valid joint angle [0,180]"))
                if ang>=0 and ang<=180:
                    break
    else: #joint 5 range is 0 to 270 degrees
        if ang<0 or ang>270:
            while True:
                ang = float(input("Input valid joint angle [0,270]"))
                if ang>=0 and ang<=270:
                    break
    return ang #return the read value to the main function

def moveJoint(jnum,ang,speedtime):
    """
    function used to move the specified joint to the given position
    moveJoint(jnum, ang, speedtime) moves joint jnum to position ang degrees in speedtime milliseconds
    function returns nothing
    """
    # call the function to move joint number jnum to ang degrees in speedtime milliseconds
    Arm.Arm_serial_servo_write(jnum,ang,speedtime)
    return

def readActualJointAngle(jnum):
    """
    function used to read the position of the specified joint
    readActualJointAngle(jnum) reads the position of joint jnum in degrees
    function returns the joint position in degrees
    """
    # call the function to read the position of joint number jnum
    ang = Arm.Arm_serial_servo_read(jnum)
    return ang

def readAllActualJointAngles():
    """
    # function to read and return all joint angles
    # returns joint angles as a 1x6 numpy array
    """
    q = np.array([Arm.Arm_serial_servo_read(1),
                  Arm.Arm_serial_servo_read(2),
                  Arm.Arm_serial_servo_read(3),
                  Arm.Arm_serial_servo_read(4),
                  Arm.Arm_serial_servo_read(5),
                  Arm.Arm_serial_servo_read(6)])
    return q

def readAllActualJointAngles2():    
    """
    # second version of function to read and return all joint angles
    # returns joint angles as a 6x1 numpy array
    """
    q = np.zeros((6,1)) #set up a 6x1 array placeholder
    for i in range(1,7): #loop through each joint (Note range(1,N) = 1,2,...,N-1)
        #note in Python the array indexing starts at 0 (the reason for i-1 index for q)
        q[i-1] = Arm.Arm_serial_servo_read(i) #store read angle into corresponding index of q
    return q

#execute the main loop unless the stop button is pressed to stop the kernel 
try:
    main()
except KeyboardInterrupt:
    print("Program closed!")
    pass

del Arm # release the arm object
