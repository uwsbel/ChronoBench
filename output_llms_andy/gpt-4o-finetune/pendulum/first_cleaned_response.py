import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math


sys = chrono.ChSystemNSC()


sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


mat = chrono.ChContactMaterialNSC()
mat.SetFriction(0.5)  


ground = chrono.ChBody()
sys.AddBody(ground)  
ground.SetFixed(True)  
ground.SetPos(chrono.ChVector3d(0, 0, 0))  
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))  


box = chrono.ChVisualShapeBox(20, 20, 1)  
box.SetColor(chrono.ChColor(0.6, 0, 0))  
ground.AddVisualShape(box, chrono.ChFramed(chrono.ChVector3d(0, 0, -1)))  


pend = chrono.ChBody()
sys.AddBody(pend)  
pend.SetPos(chrono.ChVector3d(0, -1, 0))  
pend.SetFixed(False)  
pend.SetMass(1)  
pend.SetInertiaXX(chrono.ChVector3d(0.2, 1, 1))  
pend.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))  


box = chrono.ChVisualShapeBox(1, 2, 1)  
box.SetColor(chrono.ChColor(0, 0, 0.6))  
pend.AddVisualShape(box)  


rev_joint = chrono.ChJointRevolute()
rev_joint.Initialize(ground, pend, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))  
sys.AddLink(rev_joint)  




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  
vis.SetWindowSize(1024, 768)  
vis.SetWindowTitle('ChBodyAuxRef demo')  
vis.Initialize()  
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  
vis.AddSkyBox()  
vis.AddCamera(chrono.ChVector3d(0, 5, -6))  
vis.AddTypicalLights()  


log_info = True  
while vis.Run():
    vis.BeginScene()  
    vis.Render()  
    vis.EndScene()  
    
    if log_info:
        
        pos = pend.GetPos()  
        lin_vel = pend.GetPosDt()  
        print("     t = 0.0s x = " + str(round(pos.x, 3)) + "  y = " + str(round(pos.y, 3)) + "  z = " + str(
            round(pos.z, 3)) + "  Vx = " + str(round(lin_vel.x, 3)) + "  Vy = " + str(round(lin_vel.y, 3)) + "  Vz = " + str(
            round(lin_vel.z, 3)))
        log_info = False  
    
    sys.DoStepDynamics(1e-3)