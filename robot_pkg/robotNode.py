from time import sleep
import sys
import numpy as np
import math
import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

class Robot(Node):
    def __init__(self):
        super().__init__('robot')
        self.angleJoint = 51          # [degree]
        self.moving_time = 0.1
        self.deltaJoint = -1
        self.deltaStrawW = -1
        self.deltaStrawZ = 0
        self.deltaX = 0
        self.deltaY = 0
        self.fixedZ = 0.4 #Minimum height of the robot with the coordinates (x = 0,y = 0)
        self.deltaZ = 0
        # IMPORTANT DON'T ALTER THIS PART.
        # IT IS VITAL TO MOVE THE ROBOT ACCORDING TO THE HEIGHT OF THE 
        self.fixedZPixels = 95
        self.fixedZMeters = 0.085
        self.fixedZLim = 335
        self.time = 0
        self.limInfX= 340
        self.limSupX= 360
        self.limInfY= 235
        self.limSupY= 240
        self.bot = InterbotixManipulatorXS(
            robot_model='vx300',
            group_name='arm',
            gripper_name='gripper',
            accel_time = 0.02,
            moving_time = 3.0
        )
        self.bot
        if self.bot.arm.group_info.num_joints < 5:
            self.get_logger().fatal('The robot has to have at least 5 joints!')
            self.bot.shutdown()
            sys.exit()
        
        self.initialPosition()
        
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'strawberry_info',  # Topic name to subscribe to
            self.callback,
            1
        )
        
        self.subscription  # Prevent unused variable warning
       
        

    def callback(self, msg):
        info = np.array(msg.data)
        info = info.reshape(msg.layout.dim[1].size,msg.layout.dim[0].size)
        redInfo = []
        if len(info[0]) != 0:
            redInfo = info[info[:,0].astype(int) == 1,:]
            
            
            redInfo = redInfo[np.lexsort((redInfo[:,1],))]
            
            print("RED INFO ",str(redInfo))
        
        # Process the received information here
        print("Received strawberry information:", str(info))
        
        if len(info[0]) == 0 or len(redInfo) == 0:
            self.moving_time = 0.1
            self.angleJoint = self.angleJoint + self.deltaJoint
            print("Angle11 = "+str(self.angleJoint))
            self.bot.arm.set_single_joint_position(joint_name='waist', position=math.radians(self.angleJoint),moving_time=self.moving_time,accel_time = 0.02)
        else:
            if redInfo[0,1].astype(int)<=self.limSupX+10 and redInfo[0,1].astype(int)>=self.limInfX-10:
                self.moving_time = 1
                print("moving_time = 1")
            else:
                self.moving_time = 0.1
                
            if redInfo[0,1].astype(int)>self.limSupX:
                self.deltaStrawW = -1
                print("deltaStrawW = -1")
                print("Angle12 = "+str(self.angleJoint))
            elif redInfo[0,1].astype(int)<self.limInfX:
                self.deltaStrawW = 1
                print("deltaStrawW = 1")
                print("Angle12 = "+str(self.angleJoint))
            else:
                self.deltaStrawW = 0
                self.bot.gripper.release()
                self.deltaY = round(float((redInfo[0,3]-0.07)*math.sin(math.radians(self.angleJoint))),3)
                print("DistanceY = "+str(round(redInfo[0,3]-0.06,3))+"sinTheta = " +str(math.sin(math.radians(self.angleJoint))))
                self.deltaX = round(float((redInfo[0,3]-0.07)*math.cos(math.radians(self.angleJoint))),3)
                print("DistanceX = "+str(round(redInfo[0,3]-0.06,3))+"cosTheta = " +str(math.cos(math.radians(self.angleJoint))))
                self.deltaZ = round(float(self.fixedZ+(((self.fixedZLim-redInfo[0,2])/self.fixedZPixels)*self.fixedZMeters)),3)
                
                print("deltaY "+str(round(self.deltaY*0.5,3)))
                print("deltaX "+str(round(self.deltaX*0.5,3)))
                print("deltaZ "+str(round(self.deltaZ*1.5,3)))
                print("rounded deltaZ "+str())
                self.bot.arm.set_ee_pose_components(z=round(self.deltaZ*1.4,3), x = round(self.deltaX*0.5,3), y=round(self.deltaY*0.5,3), moving_time=3)
                self.bot.arm.set_ee_pose_components(z=self.deltaZ, x = self.deltaX, y=self.deltaY, moving_time=3)
                self.bot.gripper.grasp()
                self.bot.arm.set_ee_pose_components(z=self.deltaZ, x = round(self.deltaX*0.5,3), y=round(self.deltaY*0.5,3), moving_time=3)
                self.bot.arm.set_ee_pose_components(z=0.2,x= 0.28,moving_time=2)
                self.bot.gripper.release()
                self.bot.arm.set_ee_pose_components(z=self.fixedZ,moving_time=2)
                self.angleJoint = math.degrees(self.bot.arm.get_single_joint_command("waist"))
                print("AngleJoint = "+str(self.angleJoint))
               
            
            print("Outside deltaStrawW = "+str(self.deltaStrawW))
            self.angleJoint =  self.angleJoint + self.deltaStrawW
            print("Angle Result = "+str(self.angleJoint) )
            self.bot.arm.set_single_joint_position(joint_name='waist', position=math.radians(self.angleJoint),moving_time=self.moving_time,accel_time = 0.02)                
            
            
        if self.angleJoint <= -50:
            self.deltaJoint = 1
        elif self.angleJoint >= 50:
            self.deltaJoint = -1
            
        
    def initialPosition(self):
        self.bot.arm.go_to_sleep_pose(moving_time = 3.0,accel_time = 0.02)
        self.bot.arm.set_ee_pose_components(z=self.fixedZ)
        self.bot.arm.set_single_joint_position(joint_name='waist', position=math.radians(self.angleJoint),moving_time=3,accel_time = 0.02)
    
        
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



