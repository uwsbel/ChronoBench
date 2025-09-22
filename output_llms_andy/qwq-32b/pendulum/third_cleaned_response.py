import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math


sys = chrono.ChSystemNSC()


sys.SetGravitationalAcceleration(chrono.ChVectorD(0, -9.81, 0))  


ground = chrono.ChBody()
sys.Add(ground)
ground.SetFixed(True)  
ground.EnableCollision(False)  


cyl_ground = chrono.ChVisualShapeCylinder(0.2, 0.4)  
ground.AddVisualShape(cyl_ground, chrono.ChFrameD(chrono.ChVectorD(0, 0, 1), chrono.QIDENTITY))


pend_1 = chrono.ChBody()
sys.AddBody(pend_1)
pend_1.SetFixed(False)  
pend_1.EnableCollision(False)  
pend_1.SetMass(1)  

pend_1.SetInertiaXX(chrono.ChVectorD(
    (1/12)*1*(2)**2,  
    0.5*1*(0.2)**2,   
    0.5*1*(0.2)**2    
))


cyl_1 = chrono.ChVisualShapeCylinder(0.2, 2)  
cyl_1.SetColor(chrono.ChColor(0.6, 0, 0))  
pend_1.AddVisualShape(cyl_1, chrono.ChFrameD(
    chrono.ChVectorD(0, 0, 0),
    chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.ChVectorD(0, 1, 0))  
))


pend_1.SetPos(chrono.ChVectorD(1, 0, 1))


rev_1 = chrono.ChLinkLockRevolute()
rev_1.Initialize(
    ground,
    pend_1,
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 1), chrono.QIDENTITY),  
    chrono.ChFrameD(chrono.ChVectorD(-1, 0, 0), chrono.QIDENTITY)  
)
sys.AddLink(rev_1)


pend_2 = chrono.ChBody()
sys.AddBody(pend_2)
pend_2.SetFixed(False)
pend_2.EnableCollision(False)
pend_2.SetMass(1)

pend_2.SetInertiaXX(chrono.ChVectorD(
    (1/12)*1*(2)**2,
    0.5*1*(0.2)**2,
    0.5*1*(0.2)**2
))


pend_2.SetPos(chrono.ChVectorD(3, 0, 1))  


cyl_2 = chrono.ChVisualShapeCylinder(0.2, 2)
cyl_2.SetColor(chrono.ChColor(0, 0.6, 0))
pend_2.AddVisualShape(cyl_2, chrono.ChFrameD(
    chrono.ChVectorD(0, 0, 0),
    chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.ChVectorD(0, 1, 0))
))


rev_2 = chrono.ChLinkLockRevolute()
rev_2.Initialize(
    pend_1,
    pend_2,
    chrono.ChFrameD(chrono.ChVectorD(1, 0, 0), chrono.QIDENTITY),  
    chrono.ChFrameD(chrono.ChVectorD(-1, 0, 0), chrono.QIDENTITY)  
)
sys.AddLink(rev_2)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Double Pendulum Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 3, 6))
vis.AddTypicalLights()


log_info = True
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)  

    if log_info and sys.GetChTime() > 1:
        print(f"t = {sys.GetChTime():.2f}")
        print(f"Pendulum 1 Position: ({pend_1.GetPos().x:.2f}, {pend_1.GetPos().y:.2f})")
        print(f"Pendulum 1 Velocity: ({pend_1.GetPos_dt().x:.2f}, {pend_1.GetPos_dt().y:.2f})")
        print(f"Pendulum 2 Position: ({pend_2.GetPos().x:.2f}, {pend_2.GetPos().y:.2f})")
        print(f"Pendulum 2 Velocity: ({pend_2.GetPos_dt().x:.2f}, {pend_2.GetPos_dt().y:.2f})")
        log_info = False