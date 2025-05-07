from fanuc_motion_program_exec_client import *

client = FANUCClient('192.168.0.1')
    
read_N=3
st=time.perf_counter()
res = client.get_joint_angle(read_N=read_N)
print(res)
et=time.perf_counter()
print("Total time:",et-st)
print("Time per Read:",(et-st)/read_N)