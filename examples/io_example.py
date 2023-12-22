from fanuc_motion_program_exec_client import *

client = FANUCClient('192.168.0.1')
client.set_ioport('DOUT',10,True)
res = client.read_ioport('DOUT',10)
print("The port is:",res)
client.set_ioport('DOUT',10,False)
res = client.read_ioport('DOUT',10)
print("The port is:",res)