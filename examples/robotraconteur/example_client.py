from RobotRaconteur.Client import *

import numpy as np
import general_robotics_toolbox as rox
import math

c = RRN.ConnectService("rr+tcp://localhost:59845?service=mp_robot")

robot_pose_type = RRN.GetStructureType("experimental.robotics.motion_program.RobotPose",c)
moveabsj_type = RRN.GetStructureType("experimental.robotics.motion_program.MoveAbsJCommand",c)
movej_type = RRN.GetStructureType("experimental.robotics.motion_program.MoveJCommand",c)
movel_type = RRN.GetStructureType("experimental.robotics.motion_program.MoveLCommand",c)
movec_type = RRN.GetStructureType("experimental.robotics.motion_program.MoveCCommand",c)
motionprogram_type = RRN.GetStructureType("experimental.robotics.motion_program.MotionProgram",c)
toolinfo_type = RRN.GetStructureType("com.robotraconteur.robotics.tool.ToolInfo",c)
transform_dt = RRN.GetNamedArrayDType("com.robotraconteur.geometry.Transform",c)
spatialinertia_dt = RRN.GetNamedArrayDType("com.robotraconteur.geometry.SpatialInertia",c)

def wpr2R(wpr):    
    return rox.rot([0,0,1],math.radians(wpr[2]))@rox.rot([0,1,0],math.radians(wpr[1]))@rox.rot([1,0,0],math.radians(wpr[0]))

def robot_pose(p,wpr,conf):
    R = wpr2R(wpr)
    q = rox.R2q(R)

    ret = robot_pose_type()
    ret.tcp_pose[0]["orientation"]["w"] = q[0]
    ret.tcp_pose[0]["orientation"]["x"] = q[1]
    ret.tcp_pose[0]["orientation"]["y"] = q[2]
    ret.tcp_pose[0]["orientation"]["z"] = q[3]
    ret.tcp_pose[0]["position"]["x"] = p[0]*1e-3
    ret.tcp_pose[0]["position"]["y"] = p[1]*1e-3
    ret.tcp_pose[0]["position"]["z"] = p[2]*1e-3

    rr_confdata = [None]*6
    for i in range(3):
        rr_confdata[i] = RR.VarValue(conf[i],"string")
    for i in range(3,6):
        rr_confdata[i] = RR.VarValue(int(conf[i]),"int32")

    confdata = RR.VarValue(rr_confdata, "varvalue{list}")
    
    return ret, confdata

jt1 = np.deg2rad(np.array([0,20,-10,0,-20,10],dtype=np.float64))
pt1 = robot_pose([1850,200,290],[-180,0,0],('N','U','T',0,0,0))
pt2 = robot_pose([1850,200,589],[-180,0,0],('N','U','T',0,0,0))
pt3 = robot_pose([1850,250,400],[-180,0,0],('N','U','T',0,0,0))

setup_cmds = []
mp_cmds = []

def moveabsj(j,velocity,blend_radius,fine_point, velocity_units = None):
    cmd = moveabsj_type()
    cmd.joint_position = j
    cmd.tcp_velocity = velocity
    cmd.blend_radius = blend_radius
    cmd.fine_point = fine_point
    if velocity_units is not None:
        cmd.extended = {
            "tcp_velocity_units": RR.VarValue(velocity_units, "string")
        }
    return RR.VarValue(cmd,"experimental.robotics.motion_program.MoveAbsJCommand")

def movel(robot_pose,velocity,blend_radius,fine_point, j = None, velocity_units = None):    
    cmd = movel_type()
    cmd.extended = {}
    if robot_pose is not None:
        cmd.tcp_pose = robot_pose[0]
        cmd.extended["confdata"] = robot_pose[1]
    cmd.tcp_velocity = velocity
    cmd.blend_radius = blend_radius
    cmd.fine_point = fine_point

    if j is not None:
        # Fanuc allows for passing joint positions to moveJ. Use extended field
        cmd.extended["joint_position"] = RR.VarValue(j, "double[]")
    if velocity_units is not None:
        # Set the velocity units
        cmd.extended["tcp_velocity_units"] = RR.VarValue(velocity_units, "string")
    
    
    return RR.VarValue(cmd,"experimental.robotics.motion_program.MoveLCommand")

def movej(robot_pose,velocity,blend_radius,fine_point, j = None, velocity_units = None):
    cmd = movej_type()
    cmd.extended = {}
    if robot_pose is not None:
        cmd.tcp_pose = robot_pose[0]
        cmd.extended["confdata"] = robot_pose[1]
    cmd.tcp_velocity = velocity
    cmd.blend_radius = blend_radius
    cmd.fine_point = fine_point
    
    if j is not None:
        # Fanuc allows for passing joint positions to moveL. Use extended field
        cmd.extended["joint_position"] = RR.VarValue(j, "double[]")
    if velocity_units is not None:
        # Set the velocity units
        cmd.extended["tcp_velocity_units"] = RR.VarValue(velocity_units, "string")
    return RR.VarValue(cmd,"experimental.robotics.motion_program.MoveJCommand")

def movec(robot_pose,robot_via_pose,velocity,blend_radius,fine_point, j = None, j_via = None, velocity_units = None):
    cmd = movec_type()
    cmd.extended = {}
    if robot_pose is not None:
        cmd.tcp_pose = robot_pose[0]
        cmd.extended["confdata"] = robot_pose[1]
        cmd.tcp_via_pose = robot_via_pose[0]
        cmd.extended["confdata_via"] = robot_via_pose[1]
    cmd.tcp_velocity = velocity
    cmd.blend_radius = blend_radius
    cmd.fine_point = fine_point
       
    if j is not None:
        # Fanuc allows for passing joint positions to moveC. Use extended field
        cmd.extended["joint_position"] = RR.VarValue(j, "double[]")
        cmd.extended["joint_via_position"] = RR.VarValue(j_via, "double[]")
    if velocity_units is not None:
        # Set the velocity units
        cmd.extended["tcp_velocity_units"] = RR.VarValue(velocity_units, "string")
    return RR.VarValue(cmd,"experimental.robotics.motion_program.MoveCCommand")

mp_cmds.append(movel(pt1,0.050,0.100,False))
mp_cmds.append(movel(pt2,0.050,0.100,True))
mp_cmds.append(moveabsj(jt1,100,0.100,False,velocity_units = "%"))
mp_cmds.append(movel(pt2,0.050,0.100,False))
mp_cmds.append(movec(pt3,pt1,0.050,0.100,True))
mp_cmds.append(movel(pt2,0.050,0.100,True))

mp = motionprogram_type()

mp.motion_program_commands = mp_cmds
mp.motion_setup_commands = setup_cmds

# If multimove is available, optionally use a different robot other than first robot.
# See multimove example for demonstration of synchronizing multiple robots
# Groups are zero indexed
# mp.extended = {
#     "groups": RR.VarValue(1, "int32")
# }

mp_gen = c.execute_motion_program_record(mp,False)

res = None
status = None

while True:
    res, status1 = mp_gen.TryNext()
    if not res:
        break
    status = status1
    print(status)

print(f"recording_handle: {status.recording_handle}")

robot_recording = c.read_recording(status.recording_handle).NextAll()[0]

print(robot_recording.time)
print(robot_recording.command_number)
print(robot_recording.joints)

c.clear_recordings()

print("Done!")
