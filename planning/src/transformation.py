#!/usr/bin/env python
import numpy
import math as m
from numpy.linalg import inv
import zmq
import sys
import pickle
import os


port = "5556"

if len(sys.argv) > 1:
    port =  sys.argv[1]
    int(port)
    
context = zmq.Context()
print "Connecting to server..."
socket = context.socket(zmq.REQ)
socket.connect ("tcp://localhost:%s" % port)

print "Sending request..."
socket.send ("Target Peg Position?")

data = socket.recv()
p2r_matrix = pickle.loads(data)

print "Received reply..."

def peg2creel(p2r,row,col, quad):

    pegmatrix = [[[.41413,-.25831,1.91632,m.radians(14.11603)], [.41531,-.55342,1.91305,m.radians(-14.526)], [.40872,-1.03864,1.91707,m.radians(1.0472)], [.40644,-1.34428,1.91574,m.radians(-14.526)], [.40644,-1.82950,1.91632,m.radians(13.117)], [.41143,-2.13687,1.91632,m.radians(-13.50)]],   #0
                [[.41413,-.25831,1.58612,m.radians(14.11603)], [.41531,-.55372,1.58585,m.radians(-14.526)], [.40872,-1.03894,1.58687,m.radians(1.0472)], [.41304,-1.34428,1.58554,m.radians(-14.526)], [.40644,-1.82950,1.58612,m.radians(13.117)], [.41143,-2.13687,1.58698,m.radians(-13.50)]],    #1
                [[.41413,-.25831,1.25592,m.radians(14.11603)], [.41531,-.55402,1.25565,m.radians(-14.526)], [.40872,-1.03824,1.25667,m.radians(1.0472)], [.41304,-1.34428,1.25534,m.radians(-14.526)], [.40644,-1.82950,1.25592,m.radians(13.117)], [.41143,-2.13687,1.25687,m.radians(-13.50)]],    #2
                [[.45858,-.25831,.925720,m.radians(14.11603)], [.4153,-.55432,.92545,m.radians(-14.526)],   [.40872,-1.03854,.92647,m.radians(1.0472)],  [.41304,-1.34428,.92514,m.radians(-14.526)],  [.40644,-1.82950,.92572,m.radians(13.117)],  [.41143,-2.13687,.59638,m.radians(-13.50)]],     #3
                [[.41413,-.25831,.595520,m.radians(14.11603)], [.4153,-.55462,.59525,m.radians(-14.526)],   [.40872,-1.03984,.59627,m.radians(1.0472)],  [.41304,-1.34428,.59494,m.radians(-14.526)],  [.40644,-1.82950,.59552,m.radians(13.117)],  [.40644,-2.13687,.59638,m.radians(-13.50)]],     #4
                [[.41413,-.25831,.26532,m.radians(14.11603)],  [.4153,-.55492,.26505,m.radians(-14.526)],   [.40871,-1.04014,.26607,m.radians(1.0472)],  [.41304,-1.34428,.26474,m.radians(-14.526)],  [.40644,-1.82950,.26532,m.radians(13.117)],  [.40644,-2.13687,.26618,m.radians(-13.50)]]]     #5

    q1_prcH = numpy.array([[-m.cos(5*3.14159/6),0,m.sin(5*3.14159/6),pegmatrix[row][col][0]],
                      [m.sin(5*3.14159/6),0,m.cos(5*3.14159/6),pegmatrix[row][col][1]],
                      [0,-1,0,pegmatrix[row][col][2]],
                      [0,0,0,1]])
                      
    q2_prcH = numpy.array([[m.cos(3.14159/6),0,m.sin(3.14159/6),pegmatrix[row][col][0]],
                          [-m.sin(3.14159/6),0,m.cos(3.14159/6),pegmatrix[row][col][1]],
                          [0,-1,0,pegmatrix[row][col][2]],
                          [0,0,0,1]]) 
                                           
    q1_pinv = inv(q1_prcH)
    q2_pinv = inv(q2_prcH)
    
    if not quad == 1:
        current_pos = p2r.dot(q2_pinv)
        current_rot = current_pos[:3, :3]
        
        w = numpy.math.sqrt(float(1)+current_rot[0,0]+current_rot[1,1]+current_rot[2,2])*0.5
        x = (current_rot[2,1]-current_rot[1,2])/(4*w)
        y = (current_rot[0,2]-current_rot[2,0])/(4*w)
        z = (current_rot[1,0]-current_rot[0,1])/(4*w)
        print("quaternion: x= %s y= %s z= %s w= %s" % (x,y,z,w))
        print("position: x= %s y=%s z= 0.0" % (current_pos[0][3], current_pos[1][3]))
        
        command = "rosrun gazebo_ros spawn_model -file /home/art/.gazebo/models/Creel/model.sdf -sdf -model Creel -x "
        command += str(current_pos[0][3])
        command += " -y "
        command += str(current_pos[1][3]+1)
        command += " -wr "
        command += str(w)
        command += " -zr "
        command += str(z)
        os.system(command)
        
        return p2r.dot(q2_pinv)
        
    else:
        current_pos = p2r.dot(q1_pinv)
        current_rot = current_pos[:3, :3]
        
        command = "rosrun gazebo_ros spawn_model -file /home/art/.gazebo/models/Creel/model.sdf -sdf -model Creel -x "
        command += str(current_pos[0][3])
        command += " -y "
        command += str(current_pos[1][3])
        os.system(command)
        
        return p2r.dot(q1_pinv)

peg2robot = numpy.array([[0.9373, 0, 0.3460, 0.9864],    
                         [-0.3460, 0, 0.9373, 0.4868],
                         [0, -1, 0, -0.6741],
                         [0, 0, 0, 1]])
                         
                                
peg2creel(p2r_matrix, 5, 1, 2)


