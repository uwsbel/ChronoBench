import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math




sys = chrono.ChSystemNSC()


sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -1.62, 0))  




ground = chrono.ChBody()
ground.SetFixed(True)
ground.EnableCollision(False)
sys.Add(ground)


cyl_ground = chrono.ChVisualShapeCylinder(0.2, 0.4)
ground.AddVisualShape(cyl_ground, chrono.ChFramed(chrono.ChVector3d(0, 0, +1)))




pend_1 = chrono.ChBody()
pend_1.SetFixed(False)
pend_1.EnableCollision(False)
pend_1.SetMass(2)  
pend_1.SetInertiaXX(chrono.ChVector3d(0.4, 1.5, 1.5))  
sys.AddBody(pend_1)


cyl_pend = chrono.ChVisualShapeCylinder(0.1, 1.5)  
cyl_pend.SetColor(chrono.ChColor(0.6, 0, 0))

pend_1.AddVisualShape(
    cyl_pend,
    chrono.ChFramed(
        chrono.ChVector3d(0, 0.75, 0),  
        chrono.ChQuaterniond(1, 0, 0, 0)  
    )
)



pivot_pos = chrono.ChVector3d(0, 0, 1)

pend_length = 1.5
com_offset = chrono.ChVector3d(0.75, 0, 0)  

rot = chrono.Q_from_AngAxis(-math.pi/2, chrono.ChVector3d(0, 0, 1))
com_offset_rot = rot.Rotate(com_offset)
pend_1.SetPos(pivot_pos + com_offset_rot)
pend_1.SetRot(rot)


pend_1.SetWvel_loc(chrono.ChVector3d(0, 0, 1))  





sph_joint = chrono.ChLinkLockSpherical()

sph_joint.Initialize(ground, pend_1, chrono.ChFramed(pivot_pos, chrono.ChQuaterniond(1, 0, 0, 0)))
sys.AddLink(sph_joint)




sphere_joint = chrono.ChVisualShapeSphere(2.0)  
sphere_joint.SetColor(chrono.ChColor(0, 0.5, 1))
ground.AddVisualShape(sphere_joint, chrono.ChFramed(pivot_pos))




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('ChBodyAuxRef demo (Lunar Pendulum)')
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
        pos_1 = pend_1.GetPos()
        print("t = ", sys.GetChTime())
        print("     ", pos_1.x, "  ", pos_1.y)
        lin_vel_1 = pend_1.GetPosDt()
        print("     ", lin_vel_1.x, "  ", lin_vel_1.y)
        log_info = False