########
# The module automatically connect to fanuc robot 
# and execute LS program
########

from fileinput import filename
import time
from tkinter import N
import numpy as np
from typing import NamedTuple
import itertools
from ftplib import FTP
from urllib.request import urlopen   
import urllib
import os
import general_robotics_toolbox as rox
import math

class pose(NamedTuple):
    trans: np.ndarray # [x,y,z]
    rot: np.ndarray # [w,p,r] deg

class confdata(NamedTuple):
    J5: str # F or N
    J3: str # U or D
    J1: str # T or B
    turn4: int # -1 0 1
    turn5: int # -1 0 1
    turn6: int # -1 0 1

class robtarget(NamedTuple):
    group: int
    uframe: int
    utool: int
    trans: np.ndarray # [x,y,z]
    rot: np.ndarray # [w,p,r] deg
    robconf: confdata # 
    extax: np.ndarray # shape=(6,)

class jointtarget(NamedTuple):
    group: int
    uframe: int
    utool: int
    robax: np.ndarray # shape=(6,)
    extax: np.ndarray # shape=(6,)

def unwrapped_angle_check(q_init,q_all):

    temp_q=q_all-q_init
    temp_q = np.unwrap(temp_q)
    order=np.argsort(np.linalg.norm(temp_q,axis=1))
    # return q_all[order[0]]
    return temp_q[order[0]]+q_init

def R2wpr(R):
    q=rox.R2q(R)
    r=math.degrees(math.atan2(2*(q[0]*q[3]+q[1]*q[2]),1.-2*(q[2]**2+q[3]**2)))
    p=math.degrees(math.asin(2*(q[0]*q[2]-q[3]*q[1])))
    w=math.degrees(math.atan2(2*(q[0]*q[1]+q[2]*q[3]),1.-2*(q[1]**2+q[2]**2)))
    return [w,p,r]

def getrobtarget(pose,ref_q,robot,group,uframe,utool):
    
    qall=robot.inv(pose.p,pose.R)
    q = unwrapped_angle_check(ref_q,qall)
    wpr=R2wpr(pose.R)
    robt = joint2robtarget(q,robot,group,uframe,utool)
    for i in range(3):
        robt.trans[i]=pose.p[i]
        robt.rot[i]=wpr[i]
    return robt

def joint2robtarget(q,robot,group,uframe,utool):
    
    pose_tar = robot.fwd(q)

    # joint 5
    if q[4] <= 0:
        FN='N'
    else:
        FN='F'
    # joint 3
    if q[2] >= np.deg2rad(90) or q[2] <= np.deg2rad(-90):
        UD='D'
    else:
        UD='U'
    # joint 1
    pose_j1_j456 = robot.fwd_j456(q)
    if pose_j1_j456.p[0] <= 0:
        BF='B'
    else:
        BF='T' 
    
    if q[0]>=180:
        tn1 = 1
    elif q[0]<=-180:
        tn1 = -1
    else:
        tn1 = 0
    if q[3]>=180:
        tn2 = 1
    elif q[3]<=-180:
        tn2 = -1
    else:
        tn2 = 0
    if q[5]>=180:
        tn3 = 1
    elif q[5]<=-180:
        tn3 = -1
    else:
        tn3 = 0

    return robtarget(group,uframe,utool,pose_tar.p,R2wpr(pose_tar.R),\
                    confdata(FN,UD,BF,tn1,tn2,tn3),[0]*6)

class TPMotionProgram(object):
    def __init__(self) -> None:
        
        self.progs = []
        self.target = []
        self.t_num = 0

    def moveJ(self,target,vel,vel_unit,zone):
        '''
        
        '''
        
        mo = 'J '

        self.target.append(target)
        self.t_num += 1
        mo += 'P['+str(self.t_num)+'] '
        
        if vel_unit == 'msec':
            vel = np.min([np.max([vel,1]),32000])
            mo += str(vel) + 'msec '
        else:
            vel = np.min([np.max([vel,1]),100])
            mo += str(vel) + '% '
        
        if zone < 0:
            mo += 'FINE '
        else:
            zone = np.min([zone,100])
            mo += 'CNT'+str(zone)+' '
        
        mo += ';'

        self.progs.append(mo)

    def moveL(self,target,vel,vel_unit,zone):
        '''
        
        '''
        
        mo = 'L '

        self.target.append(target)
        self.t_num += 1
        mo += 'P['+str(self.t_num)+'] '
        
        # only support mm/sec for now
        vel = np.min([np.max([vel,1]),2000])
        mo += str(vel) + 'mm/sec '
        
        if zone < 0:
            mo += 'FINE '
        else:
            zone = np.min([zone,100])
            mo += 'CNT'+str(zone)+' '
        
        mo += ';'

        self.progs.append(mo)

    def moveC(self,mid_target,end_target,vel,vel_unit,zone):
        '''
        
        '''
        
        mo = 'C '

        # mid point
        self.target.append(mid_target)
        self.t_num += 1
        mo += 'P['+str(self.t_num)+'] \n    :  '
        # end point
        self.target.append(end_target)
        self.t_num += 1
        mo += 'P['+str(self.t_num)+'] '
        
        # only support mm/sec for now
        vel = np.min([np.max([vel,1]),2000])
        mo += str(vel) + 'mm/sec '
        
        if zone < 0:
            mo += 'FINE '
        else:
            zone = np.min([zone,100])
            mo += 'CNT'+str(zone)+' '
        
        mo += ';'

        self.progs.append(mo)

    def get_tp(self):
        filename = 'TMP'
        # program name, attribute, motion
        mo = '/PROG  '+filename+'\n/ATTR\n/MN\n'
        mo += '   1:  UFRAME_NUM=0 ;\n   2:  UTOOL_NUM=2 ;\n   3:  DO[101]=ON ;\n   4:  RUN DATARECORDER ;\n'
        line_num=5
        for prog in self.progs:
            mo += '   '+str(line_num)+':'
            mo += prog
            mo += '\n'
            line_num += 1
        mo += '   '+str(line_num)+':  DO[101]=OFF ;\n'

        # pose data
        mo += '/POS\n'
        for (t_num, target) in itertools.zip_longest(range(self.t_num), self.target):
            
            if type(target) == jointtarget:
                mo+='P['+str(t_num+1)+']{\n'
                mo+='   GP'+str(target.group)+':\n'
                mo+='   UF : '+str(target.uframe)+', UT : '+str(target.utool)+',\n'
                mo+='   J1 = '+format(round(target.robax[0],3),'.3f')+' deg,  J2 = '+format(round(target.robax[1],3),'.3f')+' deg,  J3 = '+format(round(target.robax[2],3)-round(target.robax[1],3),'.3f')+' deg,\n'
                mo+='   J4 = '+format(round(target.robax[3],3),'.3f')+' deg,  J5 = '+format(round(target.robax[4],3),'.3f')+' deg,  J6 = '+format(round(target.robax[5],3),'.3f')+' deg\n'
                mo+='};\n'
            if type(target) == robtarget:
                mo+='P['+str(t_num+1)+']{\n'
                mo+='   GP'+str(target.group)+':\n'
                mo+='   UF : '+str(target.uframe)+', UT : '+str(target.utool)+',     CONFIG : \''+\
                    target.robconf.J5+' '+target.robconf.J3+' '+target.robconf.J1+', '+\
                    str(target.robconf.turn4)+', '+str(target.robconf.turn5)+', '+str(target.robconf.turn6)+'\',\n'
                mo+='   X = '+format(round(target.trans[0],3),'.3f')+' mm,  Y = '+format(round(target.trans[1],3),'.3f')+' mm,  Z = '+format(round(target.trans[2],3),'.3f')+' mm,\n'
                mo+='   W = '+format(round(target.rot[0],3),'.3f')+' deg,  P = '+format(round(target.rot[1],3),'.3f')+' deg,  R = '+format(round(target.rot[2],3),'.3f')+' deg\n'
                mo+='};\n'
        
        # end
        mo += '/END\n'
        return mo

    def dump_program(self,filename):
        
        # program name, attribute, motion
        mo = '/PROG  '+filename+'\n/ATTR\n/MN\n'
        mo += '   1:  UFRAME_NUM=1 ;\n   2:  UTOOL_NUM=2 ;\n   3:  DO[101]=ON ;\n   4:  RUN DATARECORDER ;\n'
        line_num=5
        for prog in self.progs:
            mo += '   '+str(line_num)+':'
            mo += prog
            mo += '\n'
            line_num += 1
        mo += '   '+str(line_num)+':  DO[101]=OFF ;\n'

        # pose data
        mo += '/POS\n'
        for (t_num, target) in itertools.zip_longest(range(self.t_num), self.target):
            
            if type(target) == jointtarget:
                mo+='P['+str(t_num+1)+']{\n'
                mo+='   GP'+str(target.group)+':\n'
                mo+='   UF : '+str(target.uframe)+', UT : '+str(target.utool)+',\n'
                mo+='   J1 = '+format(round(target.robax[0],3),'.3f')+' deg,  J2 = '+format(round(target.robax[1],3),'.3f')+' deg,  J3 = '+format(round(target.robax[2],3)-round(target.robax[1],3),'.3f')+' deg,\n'
                mo+='   J4 = '+format(round(target.robax[3],3),'.3f')+' deg,  J5 = '+format(round(target.robax[4],3),'.3f')+' deg,  J6 = '+format(round(target.robax[5],3),'.3f')+' deg\n'
                mo+='};\n'
            if type(target) == robtarget:
                mo+='P['+str(t_num+1)+']{\n'
                mo+='   GP'+str(target.group)+':\n'
                mo+='   UF : '+str(target.uframe)+', UT : '+str(target.utool)+',     CONFIG : \''+\
                    target.robconf.J5+' '+target.robconf.J3+' '+target.robconf.J1+', '+\
                    str(target.robconf.turn4)+', '+str(target.robconf.turn5)+', '+str(target.robconf.turn6)+'\',\n'
                mo+='   X = '+format(round(target.trans[0],3),'.3f')+' mm,  Y = '+format(round(target.trans[1],3),'.3f')+' mm,  Z = '+format(round(target.trans[2],3),'.3f')+' mm,\n'
                mo+='   W = '+format(round(target.rot[0],3),'.3f')+' deg,  P = '+format(round(target.rot[1],3),'.3f')+' deg,  R = '+format(round(target.rot[2],3),'.3f')+' deg\n'
                mo+='};\n'
        
        # end
        mo += '/END\n'

        with open(filename+'.LS', "w") as f:
            f.write(mo)

    def dump_program_multi(self,filename,motion_group):
        
        # motion group
        dg = '*,*,*,*,*'
        dg=dg[:2*(motion_group-1)]+'1'+dg[2*(motion_group-1)+1:]

        # program name, attribute, motion
        mo = '/PROG  '+filename+'\n/ATTR\nDEFAULT_GROUP	= '+dg+';\n/MN\n'
        mo += '   1:  UFRAME_NUM=1 ;\n   2:  UTOOL_NUM=2 ;\n'
        line_num=3
        for prog in self.progs:
            mo += '   '+str(line_num)+':'
            mo += prog
            mo += '\n'
            line_num += 1
        mo += '   '+str(line_num)+':  DO[10'+str(motion_group+2)+']=OFF ;\n'

        # pose data
        mo += '/POS\n'
        for (t_num, target) in itertools.zip_longest(range(self.t_num), self.target):
            
            if type(target) == jointtarget:
                mo+='P['+str(t_num+1)+']{\n'
                mo+='   GP'+str(target.group)+':\n'
                mo+='   UF : '+str(target.uframe)+', UT : '+str(target.utool)+',\n'
                mo+='   J1 = '+format(round(target.robax[0],3),'.3f')+' deg,  J2 = '+format(round(target.robax[1],3),'.3f')+' deg,  J3 = '+format(round(target.robax[2],3)-round(target.robax[1],3),'.3f')+' deg,\n'
                mo+='   J4 = '+format(round(target.robax[3],3),'.3f')+' deg,  J5 = '+format(round(target.robax[4],3),'.3f')+' deg,  J6 = '+format(round(target.robax[5],3),'.3f')+' deg\n'
                mo+='};\n'
            if type(target) == robtarget:
                mo+='P['+str(t_num+1)+']{\n'
                mo+='   GP'+str(target.group)+':\n'
                mo+='   UF : '+str(target.uframe)+', UT : '+str(target.utool)+',     CONFIG : \''+\
                    target.robconf.J5+' '+target.robconf.J3+' '+target.robconf.J1+', '+\
                    str(target.robconf.turn4)+', '+str(target.robconf.turn5)+', '+str(target.robconf.turn6)+'\',\n'
                mo+='   X = '+format(round(target.trans[0],3),'.3f')+' mm,  Y = '+format(round(target.trans[1],3),'.3f')+' mm,  Z = '+format(round(target.trans[2],3),'.3f')+' mm,\n'
                mo+='   W = '+format(round(target.rot[0],3),'.3f')+' deg,  P = '+format(round(target.rot[1],3),'.3f')+' deg,  R = '+format(round(target.rot[2],3),'.3f')+' deg\n'
                mo+='};\n'
        
        # end
        mo += '/END\n'

        with open(filename+'.LS', "w") as f:
            f.write(mo)
        
        

class FANUCClient(object):
    def __init__(self,robot_ip='127.0.0.2') -> None:

        self.robot_ip = robot_ip
        self.robot_ftp = FTP(self.robot_ip,user='robot')
        self.robot_ftp.login()
        self.robot_ftp.cwd('UD1:')
    
    def execute_motion_program(self, tpmp: TPMotionProgram):

        # # save a temp
        tpmp.dump_program('TMP')

        # # copy to robot via ftp
        with open('TMP.LS','rb') as the_prog:
            self.robot_ftp.storlines('STOR TMP.LS',the_prog)

        try:
            motion_url='http://'+self.robot_ip+'/karel/remote'
            res = urlopen(motion_url)
        except urllib.error.HTTPError:
            pass

        while True:
            try:
                file_url='http://'+self.robot_ip+'/ud1/log.txt'
                res = urlopen(file_url)
                break
            except urllib.error.HTTPError:
                time.sleep(1)

        if os.path.exists("TMP.LS"):
            os.remove("TMP.LS")
        else:
            print("TMP.LS is deleted.")

        return res.read()
    
    def execute_motion_program_multi(self, tpmp1: TPMotionProgram, tpmp2: TPMotionProgram):

        # # save a temp
        tpmp1.dump_program_multi('TMPA',1)
        tpmp2.dump_program_multi('TMPB',2)

        # # copy to robot via ftp
        with open('TMPA.LS','rb') as the_prog:
            self.robot_ftp.storlines('STOR TMPA.LS',the_prog)
        with open('TMPB.LS','rb') as the_prog:
            self.robot_ftp.storlines('STOR TMPB.LS',the_prog)

        motion_url='http://'+self.robot_ip+'/karel/remote'
        res = urlopen(motion_url)

        while True:
            try:
                file_url='http://'+self.robot_ip+'/ud1/log.txt'
                res = urlopen(file_url)
                break
            except urllib.error.HTTPError:
                time.sleep(1)

        if os.path.exists("TMPA.LS"):
            os.remove("TMPA.LS")
        else:
            print("TMPA.LS is deleted.")
        if os.path.exists("TMPB.LS"):
            os.remove("TMPB.LS")
        else:
            print("TMPB.LS is deleted.")

        return res.read()


def multi_robot():

    tp1 = TPMotionProgram()
    tp2 = TPMotionProgram()

    jt11 = jointtarget(1,1,1,[38.3,23.3,-10.7,45.7,-101.9,-48.3],[0]*6)
    pt12 = robtarget(1,1,1,[994.0,924.9,1739.5],[163.1,1.5,-1.0],confdata('N','U','T',0,0,0),[0]*6)
    pt13 = robtarget(1,1,1,[1620.0,0,1930.0],[180.0,0,0],confdata('N','U','T',0,0,0),[0]*6)

    jt21 = jointtarget(2,1,1,[-49.7,4.3,-30.9,-20.9,-35.8,52.1],[0]*6)
    pt22 = robtarget(2,1,1,[1383.1,-484.0,940.6],[171.5,-26.8,-9.8],confdata('N','U','T',0,0,0),[0]*6)
    pt23 = robtarget(2,1,1,[1166.0,0,1430.0],[180.0,0,0],confdata('N','U','T',0,0,0),[0]*6)

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

def single_robot():

    tp = TPMotionProgram()

    pt1 = robtarget(1,0,1,[1850,200,290],[-180,0,0],confdata('N','U','T',0,0,0),[0]*6)
    pt2 = robtarget(1,0,1,[1850,200,589],[-180,0,0],confdata('N','U','T',0,0,0),[0]*6)
    jt1 = jointtarget(1,0,1,[0,20,-10,0,-20,10],[0]*6)
    pt3 = robtarget(1,0,1,[1850,250,400],[-180,0,0],confdata('N','U','T',0,0,0),[0]*6)
    
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

def main():

    # single robot
    # single_robot()
    # multi robot
    multi_robot()

if __name__ == "__main__":
    main()
