import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math


sys = chrono.ChSystemNSC()


sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))  


ground = chrono.ChBody()
sys.Add(ground)
ground.SetFixed(True)  
ground.EnableCollision(False)  


ground.AddVisualShape(chrono.ChVisualShapeBox(1, 1, 1), chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))


pend Body = chrono.ChBody()
sys.AddBody(pend Body)
pend Body.SetFixed(False)  
pend Body.EnableCollision(False)  
pend Body.SetMass(1)  
pend Body.SetInertiaXX(chrono.ChVector3d(0.2, 1, 1))  


pend Body.AddVisualShape(chrono.ChVisualShapeBox(0.2, 1, 1), chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))



pend Body.SetPos(chrono.ChVector3d(1, 0, 1))



rev_joint = chrono.ChLinkLockRevolute()
rev_joint.Initialize(ground, pend Body, chrono.ChFramed(chrono.ChVector3d(0, 0, 1), chrono.ChQuaterniond(1, 0, 0, 0)))
sys.AddLink(rev_joint)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  
vis.SetWindowSize(1024, 768)  
vis.SetWindowTitle('ChBodyAuxRef demo')  
vis.Initialize()  
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  
vis.AddSkyBox()  
vis.AddCamera(chrono.ChVector3d(0, 3, 6))  
vis.AddTypicalLights()  


log_info = True  
while vis.Run():
    vis.BeginScene()  
    vis.Render()  
    vis.EndScene()  
    sys.DoStepDynamics(1e-3)  

    
    if log_info and sys.GetChTime() > 1:
        pos = pend Body.GetPos()  
        print("t = ", sys.GetChTime())
        print("     ", pos.x, "  ", pos.y)
        lin_vel = pend Body.GetPosDt()  
        print("     ", lin_vel.x, "  ", lin_vel.y)
        log_info = False