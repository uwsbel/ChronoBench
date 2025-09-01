import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


sys = chrono.ChSystemNSC()


contact_material = chrono.ChMaterialSurfaceNSC()
contact_material.SetFriction(0.5)
contact_material.SetDampingF(0.1)
contact_material.SetCompliance(0.0001)
contact_material.SetComplianceT(0.0001)


sys.SetTol(1e-6)
sys.SetMaxIters(100)


ground = chrono.ChBodyEasyBox(sys, 10, 1, 10, 1000, True, contact_material)
ground.SetPos(chrono.ChVectorD(0, -5, 0))
ground.SetBodyFixed(True)


pendulum = chrono.ChBodyEasyCylinder(sys, 1, 0.1, 1000, True, contact_material)
pendulum.SetPos(chrono.ChVectorD(0, 5, 0))
pendulum.SetRot(chrono.Q_from_AngX(chrono.CH_C_PI_4))


revolute_joint = chrono.ChLinkRevolute()
revolute_joint.Initialize(ground, pendulum, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))
sys.Add(revolute_joint)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(0, 10, -10))
vis.AddCamera(chrono.ChVectorD(0, 10, 10))
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Simple Pendulum Simulation")
vis.AddTypicalLogo()
vis.SetLogoPosition(chrono.ChVectorD(10, 10, 0))
vis.SetLogoScale(0.5)


dt = 0.01
t_end = 10


t = 0


while t < t_end:
    
    sys.DoStepDynamics(dt)
    
    
    if t % 0.1 < dt:
        print(f"Time: {t:.2f}, Position: {pendulum.GetPos().x():.2f}, {pendulum.GetPos().y():.2f}, {pendulum.GetPos().z():.2f}, Velocity: {pendulum.GetPos_dt().x():.2f}, {pendulum.GetPos_dt().y():.2f}, {pendulum.GetPos_dt().z():.2f}")
    
    
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()
    
    
    t += dt