import os
import math
import numpy as np
import pychrono as chrono
from pychrono.ros import ChROSHandler
from pychrono.robot import *
from pychrono.vehicle import *
from pychrono.irrlicht import *
from pychrono.contact import *
from pychrono.system import *


sys = chrono.ChSystemSMC()
sys.SetG(9.81)  


material = chrono.ChContactMaterialSMC()
material.SetFriction(0.3)
material.SetDampingF(0.1)
material.SetCompliance(0.01)


floor_body = chrono.ChBodyEasyPlane(chrono.ChVector3d(0, 0, 0), 0.0, True, True, material)
floor_body.SetPos(chrono.ChVector3d(0, 0, 0))
sys.Add(floor_body)


box_body = chrono.ChBodyEasyBox(1, 1, 1, 1, True, True, material)
box_body.SetPos(chrono.ChVector3d(0, 0, 1))
sys.Add(box_body)


box_shape = chrono.ChVisualShapeCylinder(0.5, 1.0)
box_body.AddVisualShape(box_shape)


class ROSHandler(ChROSHandler):
    def OnConnect(self):
        self.pub = self.pub_.ros publisher for '/status'
    
    def Update(self):
        if self.simulation_running:
            self.pub.publish(math.randint(0, 10))
            

ros_manager = ChROSManager(sys, "my_node")
ros_manager.SetClockSource(chrono.ChClockSourceSim())
ros_manager.SetBodyHandler(box_body)
ros_manager.SetTransformHandler(box_body)
ros_manager.SetClockHandler(chrono.ChClockSourceSim())
ros_manager.RegisterHandler(ROSHandler)
ros_manager.SetRate(1)  


ros_manager.RegisterHandler(ROSHandler)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('PyChrono ROS Simulation')
vis.Initialize()
cam_pos = chrono.ChVector3d(5, 5, 10)
vis.AddCamera(cam_pos)
vis.AddSkyBox()
vis.AddTypicalLights()


simulation_running = True
while True:
    if vis.Run():
        sys.DoStepDynamics(0.01)
        if simulation_running:
            ros_manager.Update()
    else:
        break