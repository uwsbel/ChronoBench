import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math


sys = chrono.ChSystemNSC()


sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))  



L1 = 2.0  
R1 = 0.1  
M1 = 1.0  


L2 = 1.5  
R2 = 0.08 
M2 = 0.8  


JOINT_Z = 1.0


ground = chrono.ChBody()
sys.Add(ground)
ground.SetFixed(True)  
ground.EnableCollision(False)  


ground_pivot_vis = chrono.ChVisualShapeCylinder(0.15, 0.4)  
ground_pivot_vis.SetColor(chrono.ChColor(0.3, 0.3, 0.3)) 
ground.AddVisualShape(ground_pivot_vis, chrono.ChFramed(chrono.ChVector3d(0, 0, JOINT_Z))) 


pend_1 = chrono.ChBody()
sys.AddBody(pend_1)
pend_1.SetFixed(False)
pend_1.EnableCollision(False)
pend_1.SetMass(M1)


inertia_p1_x = 0.5 * M1 * R1**2
inertia_p1_yz = (1/12) * M1 * (3 * R1**2 + L1**2)
pend_1.SetInertiaXX(chrono.ChVector3d(inertia_p1_x, inertia_p1_yz, inertia_p1_yz))



vis_shape_p1 = chrono.ChVisualShapeCylinder(R1, L1)
vis_shape_p1.SetColor(chrono.ChColor(0.8, 0.2, 0.2))  

pend_1.AddVisualShape(vis_shape_p1, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))



pos_p1_com = chrono.ChVector3d(L1/2, 0, JOINT_Z)
pend_1.SetPos(pos_p1_com)
pend_1.SetRot(chrono.QUNIT) 


rev_1 = chrono.ChLinkLockRevolute()

joint_pos_1 = chrono.ChVector3d(0, 0, JOINT_Z)
rev_1.Initialize(ground, pend_1, chrono.ChFramed(joint_pos_1, chrono.QUNIT))
sys.AddLink(rev_1)



pend_2 = chrono.ChBody()
sys.AddBody(pend_2)
pend_2.SetFixed(False)
pend_2.EnableCollision(False)
pend_2.SetMass(M2)


inertia_p2_x = 0.5 * M2 * R2**2
inertia_p2_yz = (1/12) * M2 * (3 * R2**2 + L2**2)
pend_2.SetInertiaXX(chrono.ChVector3d(inertia_p2_x, inertia_p2_yz, inertia_p2_yz))


vis_shape_p2 = chrono.ChVisualShapeCylinder(R2, L2)
vis_shape_p2.SetColor(chrono.ChColor(0.2, 0.2, 0.8))  

pend_2.AddVisualShape(vis_shape_p2, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))



pos_p2_com = chrono.ChVector3d(L1 + L2/2, 0, JOINT_Z)
pend_2.SetPos(pos_p2_com)
pend_2.SetRot(chrono.QUNIT) 


rev_2 = chrono.ChLinkLockRevolute()

joint_pos_2 = chrono.ChVector3d(L1, 0, JOINT_Z)
rev_2.Initialize(pend_1, pend_2, chrono.ChFramed(joint_pos_2, chrono.QUNIT))
sys.AddLink(rev_2)



vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Double Pendulum Demo') 
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(L1, L1, JOINT_Z + L1*1.5)) 
vis.SetCameraTarget(chrono.ChVector3d(L1/2, 0, JOINT_Z)) 
vis.AddTypicalLights()


log_info = True
time_step = 1e-3

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(time_step)

    if log_info and sys.GetChTime() > 1:
        pos_p2 = pend_2.GetPos()
        print(f"t = {sys.GetChTime():.3f} s")
        print(f"  Pendulum 2 COM Pos: x={pos_p2.x:.3f}, y={pos_p2.y:.3f}, z={pos_p2.z:.3f}")
        vel_p2 = pend_2.GetPosDt()
        print(f"  Pendulum 2 COM Vel: vx={vel_p2.x:.3f}, vy={vel_p2.y:.3f}, vz={vel_p2.z:.3f}")
        
        
        
        
        
        log_info = False 

del sys