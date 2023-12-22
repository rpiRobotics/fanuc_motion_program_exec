from contextlib import suppress
from RobotRaconteur.RobotRaconteurPythonUtil import NamedArrayToArray
import numpy as np
from .. import fanuc_motion_program_exec_client as fanuc_exec
import RobotRaconteur as RR
import general_robotics_toolbox as rox
import re

def rr_pose_to_fanuc(rr_pose):
    a = NamedArrayToArray(rr_pose)
    wpr = fanuc_exec.R2wpr(rox.q2R(a[0][0:4]))
    return a[0][4:7]*1000.0, wpr

def rr_joints_to_fanuc(rr_joints, rr_joint_units, motion_group = 1, uframe = 1, utool = 1):
    #TODO: joint units
    #TODO: use "extended" for external axes
    return fanuc_exec.jointtarget(motion_group, uframe, utool, np.rad2deg(rr_joints), [6e5]*6)

def rr_conf_to_fanuc(rr_joint_seed, rr_confdata_extra):
    # TODO: use joint_position_seed to compute confdata
    assert rr_confdata_extra is not None, "confdata_extra must be specified in extended field"
    assert rr_confdata_extra.datatype == "varvalue{list}"
    assert len(rr_confdata_extra.data) == 6
    confdata_a = [None]*6
    for i in range(3):
        assert rr_confdata_extra.data[i].datatype == "string"
        confdata_a[i] = rr_confdata_extra.data[i].data
    for i in range(3,6):
        if rr_confdata_extra.data[i].datatype == "string":
            confdata_a[i] = int(rr_confdata_extra.data[i].data)
        elif rr_confdata_extra.data[i].datatype in ("int32[]", "double[]"):
            confdata_a[i] = int(rr_confdata_extra.data[i].data[0])
        else:
            assert False, "Invalid confdata_extra type"
    
    return fanuc_exec.confdata(*confdata_a)


def rr_robot_pose_to_fanuc(rr_robot_pose, cfx_robot, confdata_extra = None, motion_group = 1, uframe = 1, utool = 1):
    #TODO: joint units
    #TODO: use "extended" for external axes
    xyz, wpr = rr_pose_to_fanuc(rr_robot_pose.tcp_pose)

    cd  = rr_conf_to_fanuc(rr_robot_pose.joint_position_seed, confdata_extra)
    return fanuc_exec.robtarget(motion_group, uframe, utool, xyz,wpr, cd, [6e5]*6)

def rr_target_to_fanuc(rr_robot_pose, cfx_robot, joint_position, joint_position_units, confdata_extra = None, motion_group = 1, uframe = 1, utool = 1):
    if rr_robot_pose is None:
        # Assume joint target
        return rr_joints_to_fanuc(joint_position, joint_position_units, motion_group, uframe, utool)
    else:
        return rr_robot_pose_to_fanuc(rr_robot_pose, cfx_robot, confdata_extra, motion_group, uframe, utool)

def rr_zone_to_fanuc(fine_point, blend_radius):
    if fine_point:
        return -1
    else:
        return blend_radius*1000.0
    
def rr_speed_to_fanuc(rr_velocity, rr_velocity_units):
    if rr_velocity_units is None:
        return [rr_velocity*1000.0, "mmsec"]
    rr_velocity_units = rr_velocity_units.data.lower()
    if rr_velocity_units == "mmsec":
        return [rr_velocity, "mmsec"]
    elif rr_velocity_units == "msec":
        return [rr_velocity*1000.0, "mmsec"]
    elif rr_velocity_units == "%":
        return [rr_velocity, "%"]
    assert False, "Invalid velocity units"

cmd_get_arg_sentinel = object()
cmd_arg_no_default = object()

def cmd_get_arg(cmd, arg_name, default_value = cmd_arg_no_default):
    if isinstance(cmd, RR.VarValue):
        cmd = cmd.data

    val = getattr(cmd, arg_name, cmd_get_arg_sentinel)
    if val is cmd_get_arg_sentinel:
        freeform_args = getattr(cmd, "command_args", cmd_get_arg_sentinel)
        assert freeform_args is not cmd_get_arg_sentinel, f"Invalid command type, missing argument {arg_name}"

        val = freeform_args.get(arg_name, cmd_get_arg_sentinel)
        if val is cmd_get_arg_sentinel and default_value is not cmd_arg_no_default:
            return default_value
        assert val is not cmd_get_arg_sentinel, f"Invalid command type, missing argument {arg_name}"

    if isinstance(val, RR.VarValue):
        val = val.data

    return val

def cmd_get_extended(cmd, extended_name, default_value = None):
    if isinstance(cmd, RR.VarValue):
        cmd = cmd.data
    if not cmd.extended:
        return default_value
    val = cmd.extended.get(extended_name, default_value)
    return val

def get_common_args(kwargs):
    motion_group = kwargs.get("motion_group",1)
    uframe = kwargs.get("uframe",1)
    utool = kwargs.get("utool",1)
    return [motion_group, uframe, utool]

class MoveAbsJCommandConv:
    rr_types = ["experimental.robotics.motion_program.MoveAbsJCommand"]
    freeform_names = ["MoveAbsJ", "MoveAbsJCommand", "experimental.robotics.motion_program.MoveAbsJCommand"]

    def apply_rr_command(self, cmd, mp, **kwargs):        
        zd = rr_zone_to_fanuc(cmd_get_arg(cmd,"fine_point"),cmd_get_arg(cmd,"blend_radius"))
        sd = rr_speed_to_fanuc(cmd_get_arg(cmd, "tcp_velocity"), cmd_get_extended(cmd, "tcp_velocity_units", None))
        jt = rr_joints_to_fanuc(cmd_get_arg(cmd,"joint_position", None), cmd_get_arg(cmd,"joint_units", []), *get_common_args(kwargs))
        mp.moveJ(*([jt] + sd + [zd]))

class MoveJCommandConv:
    rr_types = ["experimental.robotics.motion_program.MoveJCommand"]
    freeform_names = ["MoveJ","MoveJCommand","experimental.robotics.motion_program.MoveJCommand"]

    def apply_rr_command(self, cmd, mp, cfx_robot, **kwargs):
        zd = rr_zone_to_fanuc(cmd_get_arg(cmd,"fine_point"),cmd_get_arg(cmd,"blend_radius"))
        sd = rr_speed_to_fanuc(cmd_get_arg(cmd,"tcp_velocity"), cmd_get_extended(cmd,"tcp_velocity_units", None))
        rt = rr_target_to_fanuc(cmd_get_arg(cmd,"tcp_pose"), cfx_robot, 
            cmd_get_extended(cmd,"joint_position", None), cmd_get_extended(cmd,"joint_units", []), cmd_get_extended(cmd,"confdata", None),
            *get_common_args(kwargs))
        mp.moveJ(*([rt] + sd + [zd]))

class MoveLCommandConv:
    rr_types = ["experimental.robotics.motion_program.MoveLCommand"]
    freeform_names = ["MoveL","MoveLCommand","experimental.robotics.motion_program.MoveLCommand"]

    def apply_rr_command(self, cmd, mp, cfx_robot, **kwargs):
        zd = rr_zone_to_fanuc(cmd_get_arg(cmd,"fine_point"),cmd_get_arg(cmd,"blend_radius"))
        sd = rr_speed_to_fanuc(cmd_get_arg(cmd,"tcp_velocity"), cmd_get_extended(cmd,"tcp_velocity_units", None))
        rt = rr_target_to_fanuc(cmd_get_arg(cmd,"tcp_pose"), cfx_robot,
            cmd_get_extended(cmd,"joint_position", None), cmd_get_extended(cmd,"joint_units", []), cmd_get_extended(cmd,"confdata", None),
            *get_common_args(kwargs))
        mp.moveL(*([rt] + sd + [zd]))

class MoveCCommandConv:
    rr_types = ["experimental.robotics.motion_program.MoveCCommand"]
    freeform_names = ["MoveC","MoveCCommand","experimental.robotics.motion_program.MoveCCommand"]

    def apply_rr_command(self, cmd, mp, cfx_robot, **kwargs):
        zd = rr_zone_to_fanuc(cmd_get_arg(cmd,"fine_point"),cmd_get_arg(cmd,"blend_radius"))
        sd = rr_speed_to_fanuc(cmd_get_arg(cmd,"tcp_velocity"), cmd_get_extended(cmd,"tcp_velocity_units", None))
        rt = rr_target_to_fanuc(cmd_get_arg(cmd,"tcp_pose"), cfx_robot, 
            cmd_get_extended(cmd,"joint_position"), cmd_get_extended(cmd,"joint_units", []), cmd_get_extended(cmd,"confdata", None),
            *get_common_args(kwargs))
        rt2 = rr_target_to_fanuc(cmd_get_arg(cmd,"tcp_via_pose", None), cfx_robot, 
            cmd_get_extended(cmd,"joint_via_position", None), cmd_get_extended(cmd,"joint_via_units", []), cmd_get_extended(cmd,"confdata_via", None),
            *get_common_args(kwargs))
        mp.moveC(*([rt2, rt] + sd + [zd]))

class WaitTimeCommandConv:
    rr_types = ["experimental.robotics.motion_program.WaitTimeCommand"]
    freeform_names = ["WaitTime", "WaitTimeCommand", "experimental.robotics.motion_program.WaitTimeCommand"]

    def apply_rr_command(self, cmd, mp, **kwargs):
        mp.waittime(cmd_get_arg(cmd, "time"))

#FANUC Setup Commands

_command_convs = dict()
_freeform_command_convs = dict()

conv_types = [
    MoveAbsJCommandConv,
    MoveJCommandConv,
    MoveLCommandConv,
    MoveCCommandConv,
    WaitTimeCommandConv
]

def _init_convs():
    for c in conv_types:
        c_inst = c()
        for x in c_inst.rr_types:
            _command_convs[x] = c_inst
        for y in c_inst.freeform_names:
            _freeform_command_convs[y] = c_inst

_init_convs()

class OptionalCommandException(Exception):
    def __init__(self, message):
        super().__init__(message=message)

def get_command_conv(cmd):
    if cmd.datatype == "experimental.robotics.motion_program.FreeformCommand":
        conv = _freeform_command_convs.get(cmd.data.command_name, None)
        if conv is None:
            if cmd.data.optional:
                raise OptionalCommandException(f"Optional command {cmd.data.command_name}")
            else:
                assert False, f"Unknown command {cmd.data.command_name}"
        return conv
    else:
        conv = _command_convs.get(cmd.datatype, None)
        assert conv is not None, f"Unknown command {cmd.datatype}"
        return conv

def apply_rr_motion_command_to_mp(cmd, mp, **kwargs):
    conv = get_command_conv(cmd)
    conv.apply_rr_command(cmd, mp, **kwargs)

def get_extended_int(rr_mp, name, default_value = None):
    if rr_mp.extended is None:
        return default_value
    val = rr_mp.extended.get(name, default_value)
    if val is None:
        return default_value
    if hasattr(val, "data"):
        val = val.data[0]
    return int(val)

def rr_motion_program_to_fanuc(rr_mp, robot, motion_group):

    # Robot structure for computing confdata.cfx from joint seed
    P_cfx_w = np.copy(robot.P)
    P_cfx_w[:,0] = 0.0
    P_cfx_w[:,5:7] = 0.0    
    cfx_robot = rox.Robot(robot.H, P_cfx_w, robot.joint_type)

    setup_args = dict()
    if rr_mp.motion_setup_commands is not None:
        for setup_cmd in rr_mp.motion_setup_commands:
            #with suppress(OptionalCommandException):
                conv = get_command_conv(setup_cmd)
                conv.add_setup_args(setup_cmd, setup_args)
      
    if rr_mp.extended is not None:
        first_cmd_num_rr = rr_mp.extended.get("first_command_number", None)
        if first_cmd_num_rr is not None:
            setup_args["first_cmd_num"] = int(first_cmd_num_rr.data)
    motion_group = get_extended_int(rr_mp, "motion_group", motion_group)
    uframe = get_extended_int(rr_mp, "uframe", 1)
    utool = get_extended_int(rr_mp, "utool", 1)
    mp = fanuc_exec.TPMotionProgram(**setup_args)
    for cmd in rr_mp.motion_program_commands:
        #with suppress(OptionalCommandException):
            apply_rr_motion_command_to_mp(cmd, mp, robot=robot, cfx_robot=cfx_robot, motion_group=motion_group, uframe=uframe, utool=utool)
        
    return mp

def is_rr_motion_program_multimove(rr_mp):
    if rr_mp.extended is None:
        return False
    groups =rr_mp.extended.get("groups", None)
    if groups is None:
        groups = rr_mp.extended.get("tasks", None)
    if groups is None:
        return False
    
    if groups.datatype == "string":
        return False
    assert groups.datatype in ("int32[]", "uint32[]", "double[]", "varvalue{list}"), "Invalid groups type"
    return len(groups.data) > 1

def get_rr_motion_program_task(rr_mp, default_task = 1):
    if rr_mp.extended is None:
        return default_task
    groups =rr_mp.extended.get("groups", None)
    if groups is None:
        groups = rr_mp.extended.get("tasks", None)
    if groups is None:
        return default_task
    
    if groups.datatype in ("string", "int32[]", "uint32[]", "double[]"):
        return groups.data
    else:
        assert groups.datatype == "varvalue{list}"
        assert len(groups) == 1, "Multiple tasks not expected"
        if groups.data[0].datatype == "string":
            return int(groups.data[0].data)
        elif groups.data[0].datatype in ("int32[]", "uint32[]", "double[]"):
            return groups.data[0].data[0]
        else:
            assert False, "Invalid task type"
    
def rr_motion_program_to_fanuc2(program, robots):
    if (is_rr_motion_program_multimove(program)):
        return rr_multimove_motion_program_to_fanuc(program, robots)
    task = get_rr_motion_program_task(program)

    robot_ind = int(task)-1

    rox_robot = robots[robot_ind]
    mp = rr_motion_program_to_fanuc(program, rox_robot, task)
    return mp, False, task

def rr_multimove_motion_program_to_fanuc(program, robots):
    motion_programs = [program]
    multi_programs = program.extended.get("multi_motion_programs", None)
    assert multi_programs is not None, "Invalid multimove motion program"
    assert multi_programs.datatype == "varvalue{list}", "Invalid multimove motion program"
    for mp in multi_programs.data:
        assert mp.datatype == "experimental.robotics.motion_program.MotionProgram", "Invalid multimove motion program"
        motion_programs.append(mp.data)

    groups = program.extended.get("groups", None)
    assert groups is not None, "Invalid multimove motion program"
    if groups.datatype in ("int32[]", "uint32[]", "double[]"):
        tasks = [int(x+1) for x in groups.data]
    elif groups.datatype == "varvalue{list}":
        tasks = [int(x.data[0])+1 for x in groups.data]
    else:
        assert False, "Invalid multimove motion program"
        
    programs = []
    for i in range(len(motion_programs)):
        robot_ind = int(tasks[i])-1
        rox_robot = robots[robot_ind]
        programs.append(rr_motion_program_to_fanuc(motion_programs[i], rox_robot, tasks[i]))

    return programs, True, tasks
