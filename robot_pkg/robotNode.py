import sys          # This is going to be used if there's an error
import numpy as np  # Numpy operations
import math         # Math operations
import rclpy        # Initilize the communication
from rclpy.node import Node                   # To make the nodes
from std_msgs.msg import Float32MultiArray    # The type of array we are going to use
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS  # To manipulate the robot

# You shouldn't get confused with the variables X, Y and Z. That will depend on the context from robot view or camera view.
# That's the reason in strawberryDetection is seen from Camera View, not Robot View.

# ROBOT AXIS
# We create the robot class
class Robot(Node):
    # The constructor
    def __init__(self):
        # We initialize the node called "robot".
        super().__init__('robot')
        # This is the initial position of the robot.
        self.angleJoint = 51          # [degree]
        # The time that needs a movement to be executed, it is more explained in the original class.
        self.moving_time = 0.1
        # This will give the main direction
        self.deltaJoint = -1
        # This delta is used when it is detected a strawberry
        self.deltaStrawW = -1
        # The deltaX is for the position in X of the Robot
        self.deltaX = 0
        self.deltaY = 0
        self.fixedZ = 0.4 #Minimum height of the robot with the coordinates (x = 0,y = 0)
        self.deltaZ = 0
        # IMPORTANT DON'T ALTER THIS PART.
        # IT IS VITAL TO MOVE THE ROBOT ACCORDING TO THE RELATION BETWEEN THE CHANGE IN ROBOT HEIGHT WITH
        # THE SAME HEIGHT BUT IN PIXELS DETECTING A STRAWBERRY TO GET THIS DELTA.
        self.fixedZPixels = 95
        self.fixedZMeters = 0.085
        # This is the correct pixel position in Z (robot Axis), to pick up the strawberry
        self.fixedZLim = 335
        # Camera Axis
        # These are the limits that the strawberry should be between to be ready to be picked
        self.limInfX= 340
        self.limSupX= 360
        
        # We create the robot object according to the information we have
        self.bot = InterbotixManipulatorXS(
            robot_model='vx300',
            group_name='arm',
            gripper_name='gripper',
            accel_time = 0.02,
            moving_time = 3.0
        )
        self.bot
        
        # The robot should have at least 5 joints to be run.
        if self.bot.arm.group_info.num_joints < 5:
            self.get_logger().fatal('The robot has to have at least 5 joints!')
            self.bot.shutdown()
            sys.exit()
        
        # We go the initial position
        self.initialPosition()
        
        
        # We subcribed the topic 'strawberry_info' type Float32MultiArray with a queue of one.
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'strawberry_info',  # Topic name to subscribe to
            self.callback,
            1
        )
        
        self.subscription  # Prevent unused variable warning
       
        
    def callback(self, msg):
        # Here we convert the information into an array
        info = np.array(msg.data)
        # We reshape the array to make it as the original matrix.
        info = info.reshape(msg.layout.dim[1].size,msg.layout.dim[0].size)
        # redInfo will have just the red ones information
        redInfo = []
        # We are going to go inside if there's information
        if len(info[0]) != 0:
            # Here we get just the red ones.
            redInfo = info[info[:,0].astype(int) == 1,:]
            # We order it again with reference to the second column from left to right, ascending order (X Axis -Camera View-)
            redInfo = redInfo[np.lexsort((redInfo[:,1],))]
            # We print the information
            print("RED INFO ",str(redInfo))
        
        # Process the received information here
        print("Received strawberry information:", str(info))
        
        # If there are no red ones we move the robot in the next way.
        if len(redInfo) == 0:
            self.moving_time = 0.1
            self.angleJoint = self.angleJoint + self.deltaJoint
            print("Angle11 = "+str(self.angleJoint))
            # We move just the waist
            self.bot.arm.set_single_joint_position(joint_name='waist', position=math.radians(self.angleJoint),moving_time=self.moving_time,accel_time = 0.02)
        else:
            # If there are red strawberries
            # And we are moving CW
            if self.deltaJoint == -1:
                # If there are more than one strawberry, we are going to choose the first of the left side.
                chosenOne = redInfo[0,:]
            else:
                # And we are moving CCW
                # If there are more than one strawberry, we are going to choose the first from the right side.
                chosenOne = redInfo[-1,:]
        
        # If the strawberry is thin the limits [limInfX-10,limSupX+10], we change the the movement speed to slower
        # Increasing the moving_time = 1
        # Else it should be faster, moving_time = 0.1
            if chosenOne[1].astype(int)<=self.limSupX+10 and chosenOne[1].astype(int)>=self.limInfX-10:
                self.moving_time = 1
                print("moving_time = 1")
            else:
                self.moving_time = 0.1
            
            
            # If when it is detected there are red strawberries we ignore the deltaJoint, we use deltaStrawW
            # Because the deltaStrawW can change negative and positive rapidly because the objective is turn the robot
            # until the strawberry is within the limits [limInfX,limSupX]
            if chosenOne[1].astype(int)>self.limSupX:
                self.deltaStrawW = -1
                print("deltaStrawW = -1")
                print("Angle12 = "+str(self.angleJoint))
            elif chosenOne[1].astype(int)<self.limInfX:
                self.deltaStrawW = 1
                print("deltaStrawW = 1")
                print("Angle12 = "+str(self.angleJoint))
            else:
                # When it is whitin the limits we don't move anymore
                self.deltaStrawW = 0
                # We open the gripper
                self.bot.gripper.release()
                # We got deltaY, deltaX, and deltaZ (Axis Robot). 
                # The explanation can be seen in the images.
                # IMPORTANT: They should be rounded to 3 decimals (until milimeters), because the given distances have a lot of decimals
                # and if we don't round them, the robot won't be able to do the movement.
                self.deltaY = round(float((chosenOne[3]-0.07)*math.sin(math.radians(self.angleJoint))),3)
                print("DistanceY = "+str(round(chosenOne[3]-0.07,3))+"sinTheta = " +str(math.sin(math.radians(self.angleJoint))))
                self.deltaX = round(float((chosenOne[3]-0.07)*math.cos(math.radians(self.angleJoint))),3)
                print("DistanceX = "+str(round(chosenOne[3]-0.07,3))+"cosTheta = " +str(math.cos(math.radians(self.angleJoint))))
                self.deltaZ = round(float(self.fixedZ+(((self.fixedZLim-chosenOne[2])/self.fixedZPixels)*self.fixedZMeters)),3)
                
                # We print the results.
                print("deltaY "+str(round(self.deltaY*0.5,3)))
                print("deltaX "+str(round(self.deltaX*0.5,3)))
                print("deltaZ "+str(round(self.deltaZ*1.4,3)))
                print("rounded deltaZ "+str())
                # The 1.4 can be changed, it means that it will go up a little, and it will take the half way in X and Y (0.5),
                self.bot.arm.set_ee_pose_components(z=round(self.deltaZ*1.4,3), x = round(self.deltaX*0.5,3), y=round(self.deltaY*0.5,3), moving_time=3)
                # We complete the the path
                self.bot.arm.set_ee_pose_components(z=self.deltaZ, x = self.deltaX, y=self.deltaY, moving_time=3)
                # We grasp the strawberry
                self.bot.gripper.grasp()
                # We go back, again the half way in X and Y; however, now the height is exactly the same, it doesn't need to go up a little.
                self.bot.arm.set_ee_pose_components(z=self.deltaZ, x = round(self.deltaX*0.5,3), y=round(self.deltaY*0.5,3), moving_time=3)
                # We go the specified basket location
                self.bot.arm.set_ee_pose_components(z=0.2,x= 0.28,moving_time=2)
                # We open the gripper.
                self.bot.gripper.release()
                # We go the middle in the fixedZ position
                self.bot.arm.set_ee_pose_components(z=self.fixedZ,moving_time=2)
                # We got the current position of the "waist"
                self.angleJoint = math.degrees(self.bot.arm.get_single_joint_command("waist"))
                # It is printed
                print("AngleJoint = "+str(self.angleJoint))
            
            #We move the "waist" using now "deltaStrawW" instead of "deltaJoint"
            print("Outside deltaStrawW = "+str(self.deltaStrawW))
            self.angleJoint =  self.angleJoint + self.deltaStrawW
            print("Angle Result = "+str(self.angleJoint) )
            
            self.bot.arm.set_single_joint_position(joint_name='waist', position=math.radians(self.angleJoint),moving_time=self.moving_time,accel_time = 0.02)                
            
        # If the angle is less than or equal -50, we change the deltaJoint to positive
        if self.angleJoint <= -50:
            self.deltaJoint = 1
        # If the angle is bigger than or equal 50, we change the deltaJoint to negative
        elif self.angleJoint >= 50:
            self.deltaJoint = -1
            
        
    def initialPosition(self):
        # Firstly we go to sleep position
        self.bot.arm.go_to_sleep_pose(moving_time = 3.0,accel_time = 0.02)
        # We go the fixedZ position
        self.bot.arm.set_ee_pose_components(z=self.fixedZ)
        # We move the initial waist position (51º)
        self.bot.arm.set_single_joint_position(joint_name='waist', position=math.radians(self.angleJoint),moving_time=3,accel_time = 0.02)
    
        # Rest of the code
def main(args=None):
    rclpy.init(args=args)
    robot = Robot()
    
    try:
        rclpy.spin(robot)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup code here
        robot.bot.arm.go_to_home_pose(moving_time = 3.0)
        robot.bot.arm.go_to_sleep_pose(moving_time = 3.0)
        robot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()