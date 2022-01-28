# Make sure to have the server side running in CoppeliaSim:
# in a child script of a CoppeliaSim scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!
import rclpy
try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')

import os
from rclpy.node import Node
from trajectory_follower import TrajectoryFollower
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from rclpy.time import Time
from std_msgs.msg import Float32

TIME_STEP = 0.05

class VrepSim(Node):

    def __init__(self, client_id):
        super().__init__('VrepSim')
        self.publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.clock_sub = self.create_subscription(Float32, "/simulationTime",self.clock_callback, 10)
        self.timer = self.create_timer(TIME_STEP, self.step)
        self.client_id = client_id
        self.joints = {}
        self.hand_joints = {}

        for i in range(1,8):
            name = 'panda_joint' + str(i)
            self.joints[name.lower()]=sim.simxGetObjectHandle(self.client_id, name, sim.simx_opmode_blocking)[1]
        for i in range(1,3):
            name = 'Panda_gripper_joint' + str(i)
            saved_name = 'panda_finger_joint' + str(i)

            self.hand_joints[saved_name]=sim.simxGetObjectHandle(self.client_id, saved_name, sim.simx_opmode_blocking)[1]
            # self.get_logger().info(f"{name}:{self.hand_joints[saved_name]}")

        self.follower = TrajectoryFollower(self.client_id, self, self.joints, "panda_arm_controller")
        self.hand_follower = TrajectoryFollower(self.client_id, self, self.hand_joints, "panda_hand_controller")
        self.joints.update(self.hand_joints)


    def clock_callback(self, inmsg):
        msg = Clock()
        msg.clock = Time(nanoseconds=inmsg.data*1e9).to_msg()
        self.clock_pub.publish(msg)



    def step(self):
        # p.stepSimulation()
        self.publisher.publish(self.publish())


    def publish(self) -> JointState:

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for name, idx in self.joints.items():
            state = sim.simxGetJointPosition(self.client_id, idx, sim.simx_opmode_streaming)[1]
            msg.name.append(name)
            msg.position.append(state)
        return msg

def main(args=None):
    rclpy.init()
    print ('Program started')
    sim.simxFinish(-1) # just in case, close all opened connections
    clientID=sim.simxStart('127.0.0.1',19997,True,False,20000,5) # Connect to CoppeliaSim
    print(f"Client id is {clientID}")

    if clientID!=-1:
        print ('Connected to remote API server')
        scene_path = os.path.join(
                get_package_share_directory("panda_vrep"),
                "models",
                "empty_2_server.ttt",
            )
        print(sim.simxLoadScene(clientID, scene_path,0, sim.simx_opmode_blocking ))
        # sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)
        sim.simxSetIntegerParameter(clientID, sim.sim_intparam_dynamic_engine, 3, sim.simx_opmode_oneshot) # Newton

        ## https://www.coppeliarobotics.com/helpFiles/en/remoteApiConstants.htm#functionErrorCodes
        ## 0 means server side, 1 means client side
        model_path = os.path.join(
                get_package_share_directory("panda_vrep"),
                "models",
                "panda_urdf.ttm",
            )
        print(sim.simxLoadModel(clientID, model_path,0, sim.simx_opmode_blocking ))
        minimal_publisher = VrepSim(clientID)
        
        executor = rclpy.executors.MultiThreadedExecutor()


        rclpy.spin(minimal_publisher, executor=executor)        
        # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
        sim.simxGetPingTime(clientID)


        # Now close the connection to CoppeliaSim:
        sim.simxFinish(clientID)
        minimal_publisher.destroy_node()
    else:
        print ('Failed connecting to remote API server')

    rclpy.shutdown()
if __name__ == '__main__':
    main()