import rclpy
import os
import sys
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

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.srv import GetEntityState, GetModelList
from gazebo_msgs.msg import EntityState

from rclpy.node import Node

class SpawnerNode(Node):
    def __init__(self, args=None):
        super().__init__("SpawnerNode")
        self.client_id=sim.simxStart('127.0.0.1',20000,True,False,2000,5) # Connect to CoppeliaSim
        self.get_logger().info(f"Client id is {self.client_id}")
        if self.client_id < 0:
            self.get_logger().fatal('Could not connect to remote API server')
            return
        cube_path = os.path.join(
            get_package_share_directory("panda_vrep"),
            "models",
            "cube.ttm",
            )
        sphere_path = os.path.join(
            get_package_share_directory("panda_vrep"),
            "models",
            "sphere.ttm",
            )

        self.entity = self.create_service(
            GetEntityState, 'get_entity_state', self.get_entity_state)
        self.model = self.create_service(
            GetModelList, 'get_model_list', self.get_model_list)
        self.objs = {}
        
        self.spawn_obj(cube_path, position=[0.5,0.0,0.025], param_name="target") # target must be second to spawn

        self.spawn_obj(cube_path, position=[1.1,0.065,0.025])
        self.spawn_obj(cube_path, position=[1.1,0.0,0.025])
        self.spawn_obj(cube_path, position=[1.1,-0.065,0.025])
        self.spawn_obj(cube_path, position=[1.1,-0.0335,0.075])
        self.spawn_obj(cube_path, position=[1.1,0.0335,0.075])
        self.spawn_obj(cube_path, position=[1.1,0.0,0.135])
        sim.simxSetBooleanParameter(self.client_id, sim.sim_boolparam_realtime_simulation, True, sim.simx_opmode_oneshot)

        sim.simxStartSimulation(self.client_id, sim.simx_opmode_oneshot)


    def spawn_obj(self, path, position=[0, 0, 0], rotation = [0,0,0,1], param_name=None):
        if not param_name:    
            name = path.split("/")[-1].split(".")[0]
            if name != "table":
                name += "_" + str(len(self.objs.keys()))
        else:
            name = param_name
        code, handle = sim.simxLoadModel(self.client_id, path,0, sim.simx_opmode_blocking )
        if code == 0: # no problem spawning obj
            sim.simxSetObjectPosition(self.client_id, handle,-1, position, sim.simx_opmode_oneshot )
            sim.simxSetObjectQuaternion(self.client_id, handle,handle, rotation, sim.simx_opmode_oneshot )# rotate around own axis
        if param_name:
            self.get_logger().info(f"{handle}")
            sim.simxSetStringSignal(self.client_id, "renameDummySignal", str(handle) + " " + param_name, sim.simx_opmode_oneshot_wait);
        
        # discard first value
        self.get_position(handle)
        self.get_rotation(handle)

        self.objs[name] = handle

    def get_model_list(self, request: GetModelList.Request, response: GetModelList.Response):
        response.model_names = list(self.objs.keys())
        response.success = True
        return response

    def get_entity_state(self, request: GetEntityState.Request, response: GetEntityState.Response):
        obj = self.objs.get(request.name)
        success = True
        if obj is None:
            response.success = False
            return response
        state = EntityState()
        state.name = request.name
        pose = Pose()
        try:    
            pose.position = self.get_position(obj)
            pose.orientation = self.get_rotation(obj)
        except: # object got deleted
            success = False
        finally:    
            state.pose = pose
            response.state = state
            response.success = success
        return response

    def get_position(self, obj):
        position = Point()
        code, obj_pose = sim.simxGetObjectPosition(self.client_id, obj,-1, sim.simx_opmode_streaming )
        position.x = obj_pose[0]
        position.y = obj_pose[1]
        position.z = obj_pose[2]
        return position

    def get_rotation(self, obj):
        rotation = Quaternion()
        code, obj_rot = sim.simxGetObjectQuaternion(self.client_id, obj,-1, sim.simx_opmode_streaming )
        rotation.x = float(obj_rot[0])
        rotation.y = float(obj_rot[1])
        rotation.z = float(obj_rot[2])
        rotation.w = float(obj_rot[3])
        return rotation

def main(args=None):
    rclpy.init(args=args)

    spawner = SpawnerNode(args=args)

    rclpy.spin(spawner)
    sim.simxFinish(spawner.client_id)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
