import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math




sys = chrono.ChSystemNSC()


sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -1.62, 0))


step_size = 1e-3
pend_length = 1.5
pend_radius = 0.1
pivot = chrono.ChVector3d(0, 0, 1)
initial_angular_speed = 1.0  




ground = chrono.ChBody()
sys.Add(ground)
ground.SetFixed(True)
ground.EnableCollision(False)


joint_sphere = chrono.ChVisualShapeSphere(2.0)
joint_sphere.SetColor(chrono.ChColor(0.2, 0.6, 0.9))
ground.AddVisualShape(joint_sphere, chrono.ChFramed(pivot))




pend_1 = chrono.ChBody()
sys.Add(pend_1)
pend_1.SetFixed(False)
pend_1.EnableCollision(False)


pend_1.SetMass(2.0)
pend_1.SetInertiaXX(chrono.ChVector3d(0.4, 1.5, 1.5))



pend_1.SetPos(chrono.ChVector3d(pend_length / 2.0, 0, 1))


pend_cyl = chrono.ChVisualShapeCylinder(pend_radius, pend_length)
pend_cyl.SetColor(chrono.ChColor(0.6, 0.0, 0.0))



pend_1.AddVisualShape(
    pend_cyl,
    chrono.ChFramed(
        chrono.ChVector3d(0, 0, 0),
        chrono.QuatFromAngleY(math.pi / 2.0)
    )
)


omega0 = chrono.ChVector3d(0, 0, initial_angular_speed)




v_com0 = chrono.ChVector3d(
    0,
    initial_angular_speed * pend_length / 2.0,
    0
)



if hasattr(pend_1, "SetAngVelParent"):
    pend_1.SetAngVelParent(omega0)
else:
    pend_1.SetWvel_par(omega0)

if hasattr(pend_1, "SetPosDt"):
    pend_1.SetPosDt(v_com0)
else:
    pend_1.SetPos_dt(v_com0)







sph_1 = chrono.ChLinkLockSpherical()
sph_1.Initialize(ground, pend_1, chrono.ChFramed(pivot))
sys.AddLink(sph_1)




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Moon pendulum with spherical joint")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))
vis.AddTypicalLights()




log_info = True

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    sys.DoStepDynamics(step_size)

    
    if log_info and sys.GetChTime() > 1.0:
        pos_1 = pend_1.GetPos()

        if hasattr(pend_1, "GetPosDt"):
            lin_vel_1 = pend_1.GetPosDt()
        else:
            lin_vel_1 = pend_1.GetPos_dt()

        print("t =", sys.GetChTime())
        print("Position:        ", pos_1.x, pos_1.y, pos_1.z)
        print("Linear velocity: ", lin_vel_1.x, lin_vel_1.y, lin_vel_1.z)

        log_info = False