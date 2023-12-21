from fanuc_motion_program_exec import *

tp = TPMotionProgram()

# robtarget(motion group, uframe, utool, [x,y,z], [w,p,r], confdata, external axis)
pt1 = robtarget(1,1,2,[1850,200,290],[-180,0,0],confdata('N','U','T',0,0,0),[0]*6)
pt2 = robtarget(1,1,2,[1850,200,589],[-180,0,0],confdata('N','U','T',0,0,0),[0]*6)
# jointtarget(motion group, uframe, utool, [j1,j2,j3,j4,j5,j6], confdata, external axis)
jt1 = jointtarget(1,1,2,[0,20,-10,0,-20,10],[0]*6)
pt3 = robtarget(1,1,2,[1850,250,400],[-180,0,0],confdata('N','U','T',0,0,0),[0]*6)

tp.moveL(pt1,50,'mmsec',100)
tp.moveL(pt2,50,'mmsec',-1)
tp.moveJ(jt1,100,'%',-1)
tp.moveL(pt2,50,'mmsec',100)
tp.moveC(pt3,pt1,50,'mmsec',-1)
tp.moveL(pt2,50,'mmsec',-1)

print(tp.get_tp())

client = FANUCClient()
res = client.execute_motion_program(tp)

with open("fanuc_log.csv","wb") as f:
    f.write(res)

print(res.decode('utf-8'))

tp = TPMotionProgram()
tp.moveL(pt1,50,'mmsec',100)
tp.moveL(pt2,50,'mmsec',-1)
client = FANUCClient()
res = client.execute_motion_program(tp)