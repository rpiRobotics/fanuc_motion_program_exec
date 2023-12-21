from fanuc_motion_program_exec import *

tp1 = TPMotionProgram()
tp2 = TPMotionProgram()

# robtarget(motion group, uframe, utool, [x,y,z], [w,p,r], confdata, external axis)
# jointtarget(motion group, uframe, utool, [j1,j2,j3,j4,j5,j6], confdata, external axis)

# use uframe=1 and utool=2
# these are for motion group 1 (robot 1)
jt11 = jointtarget(1,1,2,[38.3,23.3,-10.7,45.7,-101.9,-48.3],[0]*6)
pt12 = robtarget(1,1,2,[994.0,924.9,1739.5],[163.1,1.5,-1.0],confdata('N','U','T',0,0,0),[0]*6)
pt13 = robtarget(1,1,2,[1620.0,0,1930.0],[180.0,0,0],confdata('N','U','T',0,0,0),[0]*6)

# these are for motion group 2 (robot 2)
jt21 = jointtarget(2,1,2,[-49.7,4.3,-30.9,-20.9,-35.8,52.1],[0]*6)
pt22 = robtarget(2,1,2,[1383.1,-484.0,940.6],[171.5,-26.8,-9.8],confdata('N','U','T',0,0,0),[0]*6)
pt23 = robtarget(2,1,2,[1166.0,0,1430.0],[180.0,0,0],confdata('N','U','T',0,0,0),[0]*6)

tp1.moveJ(jt11,100,'%',-1)
tp1.moveL(pt12,500,'mmsec',100)
tp1.moveL(pt13,500,'mmsec',-1)

tp2.moveJ(jt21,100,'%',-1)
tp2.moveL(pt22,500,'mmsec',100)
tp2.moveL(pt23,500,'mmsec',-1)

client = FANUCClient()
res = client.execute_motion_program_multi(tp1,tp2)

with open("fanuc_log.csv","wb") as f:
    f.write(res)

print(res.decode('utf-8'))