#!/usr/bin/env python
import zmq
import time
import sys
import numpy
import pickle

port = "5556"
if len(sys.argv) > 1:
    port =  sys.argv[1]
    int(port)
    
context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind("tcp://*:%s" % port)

peg2robot = numpy.array([[0.9373, 0, 0.3460, 0.9864], 
             [-0.3460, 0, 0.9373, 0.4868],
             [0, -1, 0, -0.6741],
             [0, 0, 0, 1]])
             
data = pickle.dumps(peg2robot)

while True:
    #  Wait for next request from client
    message = socket.recv()
    print "Received request: ", message
    time.sleep (1)  
    socket.send(data)
