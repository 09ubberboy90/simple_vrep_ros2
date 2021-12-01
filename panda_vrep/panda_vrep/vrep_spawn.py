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

import time

def main(args=None):
    rclpy.init()
    time.sleep(5)
    print ('Program started')
    sim.simxFinish(-1) # just in case, close all opened connections
    clientID=sim.simxStart('127.0.0.1',19997,True,True,20000,5) # Connect to CoppeliaSim
    if clientID!=-1:
        print ('Connected to remote API server')

        ## https://www.coppeliarobotics.com/helpFiles/en/remoteApiConstants.htm#functionErrorCodes
        ## 0 means server side, 1 means client side
        print(sim.simxLoadModel(clientID, "models/robots/non-mobile/FrankaEmikaPandaGripperROS2.ttm",0, sim.simx_opmode_blocking ))

        sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)
        # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
        sim.simxGetPingTime(clientID)


        # Now close the connection to CoppeliaSim:
        sim.simxFinish(clientID)
    else:
        print ('Failed connecting to remote API server')

    rclpy.shutdown()
if __name__ == '__main__':
    main()