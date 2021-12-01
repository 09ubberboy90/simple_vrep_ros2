jointHandles={-1,-1,-1,-1,-1,-1,-1}
vel=90  
accel=40
jerk=80
maxVel={}
maxAccel={}
maxJerk={}
names = {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7", "panda_finger_joint1", "panda_finger_joint2"}
function sysCall_init()
    objectHandle=sim.getObjectAssociatedWithScript(sim.handle_self)
    objectName=sim.getObjectName(objectHandle)
    ros2InterfacePresent=simROS2

    -- Prepare the float32 publisher and subscriber (we subscribe to the topic we publish):
    if ros2InterfacePresent then
        subscriber=simROS2.createSubscription('/joint_states_broadcast','sensor_msgs/msg/JointState','subscriber_callback')
        publisher=simROS2.createPublisher('/joint_states','sensor_msgs/msg/JointState')

    end
    for i=1,7,1 do
        jointHandles[i]=sim.getObjectHandle('Panda_joint'..i)
    end
    for i=1,2,1 do
        jointHandles[7+i]=sim.getObjectHandle('Panda_gripper_joint'..i)
    end
    for i=1,9,1 do
        maxVel[i]=vel*math.pi/180
        maxAccel[i]=accel*math.pi/180
        maxJerk[i]=jerk*math.pi/180
    end

end

function sysCall_actuation()
    -- Send an updated simulation time message, and send the transform of the object attached to this script:
    if ros2InterfacePresent then
        local currentConf={}
        
        for i=1,#jointHandles,1 do
            currentConf[i]=sim.getJointPosition(jointHandles[i])
        end
        d={}
        d.header={stamp=simROS2.getTime(), frame_id='panda'}
        d.name=names
        d.position=currentConf

        simROS2.publish(publisher,d)
        -- To send several transforms at once, use simROS2.sendTransforms instead
    end
end

function sysCall_cleanup()
    -- Following not really needed in a simulation script (i.e. automatically shut down at simulation end):
    if rosInterfacePresent then
        simROS.shutdownPublisher(publisher)
        simROS.shutdownSubscriber(subscriber)
    end
end
function subscriber_callback(msg)
    local objective = {-1,-1,-1,-1,-1,-1,-1,-1,-1}
    local j = 1
    -- reorganize incoming joint to match table order
    for i=1,#msg.position,1 do
            if string.find(msg.name[i], "finger_joint1")  then
        objective[8] = msg.position[i]
        j=j-1
        elseif string.find(msg.name[i], "finger_joint2")  then
        objective[9] = msg.position[i]
        j=j-1
        else
        objective[j]=msg.position[i]
        end
        j =j+1
    end
    moveToConfig(jointHandles,maxVel,maxAccel,maxJerk, objective)
end

function movCallback(config,vel,accel,handles)
    for i=1,#handles,1 do
        if sim.getJointMode(handles[i])==sim.jointmode_force and sim.isDynamicallyEnabled(handles[i]) then
            sim.setJointTargetPosition(handles[i],config[i])
        else    
            sim.setJointPosition(handles[i],config[i])
        end
    end
end

function moveToConfig(handles,maxVel,maxAccel,maxJerk,targetConf)
    local currentConf={}
    for i=1,#handles,1 do
        currentConf[i]=sim.getJointPosition(handles[i])
    end
    sim.moveToConfig(-1,currentConf,nil,nil,maxVel,maxAccel,maxJerk,targetConf,nil,movCallback,handles)
end


