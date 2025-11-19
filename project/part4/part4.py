import time #import the time module. Used for adding pauses during operation
from Arm_Lib import Arm_Device #import the module associated with the arm

class PID:
    def __init__(self, kp, ki, kd, setpoint=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.integral = 0
        self.prev_error = 0

    def update(self, measured_value, dt):
        error = self.setpoint - measured_value
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return output
    
def moveJointPID(jnum, target_angle, pid_controller, dt=0.05, tol=1.0):
    """
    Move a single joint (1–5) using PID until within tolerance.
    """
    pid_controller.setpoint = target_angle
    while True:
        actual_angle = readActualJointAngle(jnum)
        error = target_angle - actual_angle
        print(f"Joint angle {jnum} error: {error}")
        if abs(error) < tol:
            break
        pid_output = pid_controller.update(actual_angle, dt)
        command_angle = actual_angle + pid_output
        # Clamp to valid range
        if command_angle < 0: command_angle = 0
        if command_angle > 180: command_angle = 180
        Arm.Arm_serial_servo_write(jnum, command_angle, 100)  # 100ms speed
        time.sleep(dt)

q_l_1 = [
  [90.0000, 90.0000, 90.0000, 90.0000, 90.0000],
  [83.3333, 85.0000, 86.6667, 81.6667, 90.0000],
  [76.6667, 80.0000, 83.3333, 73.3333, 90.0000],
  [70.0000, 75.0000, 80.0000, 65.0000, 90.0000],
  [63.3333, 70.0000, 76.6667, 56.6667, 90.0000],
  [56.6667, 65.0000, 73.3333, 48.3333, 90.0000],
  [50.0000, 60.0000, 70.0000, 40.0000, 90.0000],
  [43.3333, 55.0000, 66.6667, 31.6667, 90.0000],
  [36.6667, 50.0000, 63.3333, 23.3333, 90.0000],
  [30.0000, 45.0000, 60.0000, 15.0000, 90.0000],
]

q_l_2 = [
  [30.0000, 45.0000, 60.0000, 15.0000, 90.0000],
  [30.0000, 57.7778, 55.5556, 13.3333, 80.0000],
  [30.0000, 70.5556, 51.1111, 11.6667, 70.0000],
  [30.0000, 83.3333, 46.6667, 10.0000, 60.0000],
  [30.0000, 96.1111, 42.2222, 8.3333, 50.0000],
  [30.0000, 108.8889, 37.7778, 6.6667, 40.0000],
  [30.0000, 121.6667, 33.3333, 5.0000, 30.0000],
  [30.0000, 134.4444, 28.8889, 3.3333, 20.0000],
  [30.0000, 147.2222, 24.4444, 1.6667, 10.0000],
  [30.0000, 160.0000, 20.0000, 0.0000, 0.0000],
]

q_l_3 = [
  [30.0000, 160.0000, 20.0000, 0.0000, 0.0000],
  [26.6667, 156.6667, 18.8889, 0.0000, 0.0000],
  [23.3333, 153.3333, 17.7778, 0.0000, 0.0000],
  [20.0000, 150.0000, 16.6667, 0.0000, 0.0000],
  [16.6667, 146.6667, 15.5556, 0.0000, 0.0000],
  [13.3333, 143.3333, 14.4444, 0.0000, 0.0000],
  [10.0000, 140.0000, 13.3333, 0.0000, 0.0000],
  [6.6667, 136.6667, 12.2222, 0.0000, 0.0000],
  [3.3333, 133.3333, 11.1111, 0.0000, 0.0000],
  [0.0000, 130.0000, 10.0000, 0.0000, 0.0000],
]

q_l_4 = [
  [0.0000, 130.0000, 10.0000, 0.0000, 0.0000],
  [3.3333, 120.5556, 15.5556, 1.6667, 10.0000],
  [6.6667, 111.1111, 21.1111, 3.3333, 20.0000],
  [10.0000, 101.6667, 26.6667, 5.0000, 30.0000],
  [13.3333, 92.2222, 32.2222, 6.6667, 40.0000],
  [16.6667, 82.7778, 37.7778, 8.3333, 50.0000],
  [20.0000, 73.3333, 43.3333, 10.0000, 60.0000],
  [23.3333, 63.8889, 48.8889, 11.6667, 70.0000],
  [26.6667, 54.4444, 54.4444, 13.3333, 80.0000],
  [30.0000, 45.0000, 60.0000, 15.0000, 90.0000],
]

q_l_1_20n = [
  [90.0000, 90.0000, 90.0000, 90.0000, 90.0000],
  [86.8421, 87.6316, 88.4211, 86.0526, 90.0000],
  [83.6842, 85.2632, 86.8421, 82.1053, 90.0000],
  [80.5263, 82.8947, 85.2632, 78.1579, 90.0000],
  [77.3684, 80.5263, 83.6842, 74.2105, 90.0000],
  [74.2105, 78.1579, 82.1053, 70.2632, 90.0000],
  [71.0526, 75.7895, 80.5263, 66.3158, 90.0000],
  [67.8947, 73.4211, 78.9474, 62.3684, 90.0000],
  [64.7368, 71.0526, 77.3684, 58.4211, 90.0000],
  [61.5789, 68.6842, 75.7895, 54.4737, 90.0000],
  [58.4211, 66.3158, 74.2105, 50.5263, 90.0000],
  [55.2632, 63.9474, 72.6316, 46.5789, 90.0000],
  [52.1053, 61.5789, 71.0526, 42.6316, 90.0000],
  [48.9474, 59.2105, 69.4737, 38.6842, 90.0000],
  [45.7895, 56.8421, 67.8947, 34.7368, 90.0000],
  [42.6316, 54.4737, 66.3158, 30.7895, 90.0000],
  [39.4737, 52.1053, 64.7368, 26.8421, 90.0000],
  [36.3158, 49.7368, 63.1579, 22.8947, 90.0000],
  [33.1579, 47.3684, 61.5789, 18.9474, 90.0000],
  [30.0000, 45.0000, 60.0000, 15.0000, 90.0000],
]

Arm = Arm_Device() # Get DOFBOT object
time.sleep(2) #this pauses execution for the given number of seconds
# def main(): #define the main program function
#     speedtime = 100 #time in milliseconds to reach desired joint position
    
#     q = q_l_1_20n # change q_l_x accordingly
#     for step in range(len(q)):
#         for jnum in range(len(q[0])):
#             ang = q[step][jnum]
#             moveJoint(jnum + 1,ang,speedtime)
#             time.sleep(0.1)
#         time.sleep(1)
#     q = readAllActualJointAngles() # read the current position of all joints
#     print(q)
    
#     print("Program Ended")

def main():
    q_path = q_l_1_20n
    pid_controllers = [PID(0.6, 0.2, 0.1) for _ in range(5)]  # one per joint
    
    start_time = time.time()

    for step_idx, step in enumerate(q_path):
        print(f"Moving to step {step_idx + 1}: {step}")
        for jnum in range(5):
            moveJointPID(jnum+1, step[jnum], pid_controllers[jnum])
        time.sleep(0.2)
    
    end_time = time.time()

    print("Final joint angles:", readAllActualJointAngles()[:5])
    print(f"Total time {end_time - start_time}")
    
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

#this cell provides two versions of a function to read all joint angles
import numpy as np #import module numpy, assign new name for module (np) for readability

# function to read and return all joint angles
# returns joint angles as a 1x6 numpy array
def readAllActualJointAngles():
    q = np.array([Arm.Arm_serial_servo_read(1),Arm.Arm_serial_servo_read(2),Arm.Arm_serial_servo_read(3),Arm.Arm_serial_servo_read(4),Arm.Arm_serial_servo_read(5),Arm.Arm_serial_servo_read(6)])
    return q

# second version of function to read and return all joint angles
# returns joint angles as a 6x1 numpy array
def readAllActualJointAngles2():    
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