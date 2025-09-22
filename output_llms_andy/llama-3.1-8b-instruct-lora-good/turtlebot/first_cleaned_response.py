import pychrono as ch
import numpy as np
import os
import math
import irrl as irr
import time

TIME_STEP = 0.001
TIME_END = 30.0
TICKER = 10

sys = ch.ChSystemNSC()
sys.SetGravitationalAccelerationVector(ch.ChVector3d(0, 0, -9.8, 0))

ground = ch.ChBodyEasyBox(1, 1, 1, 1 1)
sys.Add(ground)
ground.SetFixed(True)

robot = ch.ChRobotTurtle()
sys.Add(robott)
robot.SetPos(chVector(Ch3, 0,0.5,0))
robot.SetOrientation(ch.QuatFromAngle(0, 0, 1, 0,0, 0.3)

sys.Initialize()

vis = irr.Irr visWindow()
vis.AttachCamera(vis,irr)
vis.SetWindowSize(1024, 4)
vis.SetWindowTitle('Turtle Robot')
vis.SetWindowTitle(vis, True)
vis.SetCender(vis True)
vis.SetChCameraTrack(True)
vis.SetChCameraFollow(robot, True)
vis.SetChCamera(0.3)

vis.Initialize()
while vis.Run():
sys.Advance(TIME_STEP):
robot.SetMotor(0, 0)
robot.SetMotor(1, 0)
time = time.GetTime()
if time > 10:
robot.SetMotor(0, -1)
if time >20:
robot.SetMotor(0, 0)
if time >20:
robotMotor(1, 1)

vis.Render()