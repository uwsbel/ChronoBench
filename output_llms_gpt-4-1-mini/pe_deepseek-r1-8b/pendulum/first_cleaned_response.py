import os
import math
import numpy as np
from pychrono import chrono, irrrlicht as chronoirr
from pychrono.robot import *
from pychrono.vehicle import *
from pychrono.irrlicht import ChVisualShapePlane, ChColor


sys = chrono.ChSystemNSC()


chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "data"))


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Pendulum Simulation")
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 5, 10))


ground_body = chrono.ChBody()
ground_body.SetPos(chrono.ChVector3d(0, 0, 0))
ground_body.SetMass(0)
ground_body.SetFixed(True)
sys.Add(ground_body)


pendulum_body = chrono.ChBodyEasySphere(0.1, 1000, True, True, chrono.ChContactMaterialNSC())
pendulum_body.SetPos(chrono.ChVector3d(0, -1, 0))
sys.Add(pendulum_body)


joint = chrono.ChLinkLockRevolute()
joint.Initialize(ground_body, pendulum_body, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(math.pi / 2)))


sys.SetGravity(chrono.ChVector3d(0, -9.81, 0))


pendulum_shape = chrono.ChVisualShapeSphere(0.1)
pendulum_body.AddVisualShape(pendulum_shape)
ground_shape = chrono.ChVisualShapePlane(chrono.ChPlaneXY)
ground_body.AddVisualShape(ground_shape)


log_interval = 50
last_log_time = 0
position_log = []
velocity_log = []

def OnReportPosition(body, time):
    global last_log_time, position_log, velocity_log
    if time - last_log_time >= log_interval:
        pos = body.GetPos()
        vel = body.GetVelocity()
        position_log.append(f"Time: {time:.3f}, Position: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")
        velocity_log.append(f"Time: {time:.3f}, Velocity: ({vel.x:.3f}, {vel.y:.3f}, {vel.z:.3f})")
        last_log_time = time


sys.GetContactContainer().RegisterAddContactCallback(OnReportPosition)


time_step = 0.001
while True:
    sys.DoStepDynamics(time_step)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    if vis.Run():
        continue
    break


vis.Destroy()