import os
import math
import numpy as np
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


sys = chrono.ChSystemSMC()


chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "data"))


ground = chrono.ChBodyEasyBox(sys, 10, 1, 10, 1000, True, True, chrono.ChColor(0.5, 0.5, 0.5))
ground.SetPos(chrono.ChVector3d(0, -3, 0))
ground.SetBodyFixed(True)


pendulum = chrono.ChBodyEasySphere(sys, 1, 1000, True, True, chrono.ChColor(1, 0, 0))
pendulum.SetPos(chrono.ChVector3d(0, 0, 0))
pendulum.SetMass(1)
pendulum.SetInertiaXX(chrono.ChVector3d(1, 1, 1))


revolute_joint = chrono.ChLinkLockRevolute()
revolute_joint.Initialize(ground, pendulum, chrono.ChCoordsysD(chrono.ChVector3d(0, -3, 0), chrono.QUNIT))
sys.Add(revolute_joint)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Pendulum Simulation")
vis.AttachSystem(sys)
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0, 6))
vis.AddTypicalLights()


while vis.Run():
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()
    
    
    print(f"Pendulum position: {pendulum.GetPos().x}, {pendulum.GetPos().y}, {pendulum.GetPos().z}")
    print(f"Pendulum velocity: {pendulum.GetPos_dt().x}, {pendulum.GetPos_dt().y}, {pendulum.GetPos_dt().z}")
    
    
    sys.DoStepDynamics(0.01)