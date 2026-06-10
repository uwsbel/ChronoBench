import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math






sys = chrono.ChSystemNSC()


sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))






ground = chrono.ChBody()
ground.SetFixed(True)                       
ground.SetName("Ground")
sys.Add(ground)


ground_sphere = chrono.ChVisualShapeSphere(0.05)
ground_sphere.SetColor(chrono.ChColor(0.0, 0.0, 1.0))
ground.AddVisualShape(ground_sphere, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))


pendulum = chrono.ChBody()


pendulum_mass = 1.0
pendulum.SetMass(pendulum_mass)

pendulum.SetInertiaXX(chrono.ChVector3d(0.2, 0.2, 0.2))



pendulum_length = 2.0
pendulum.SetPos(chrono.ChVector3d(pendulum_length / 2.0, 0, 0))
pendulum.SetName("Pendulum")
sys.Add(pendulum)


pend_cyl = chrono.ChVisualShapeCylinder(0.05, pendulum_length)
pend_cyl.SetColor(chrono.ChColor(1.0, 0.4, 0.0))

rod_rotation = chrono.QuatFromAngleZ(chrono.CH_PI_2)
pendulum.AddVisualShape(pend_cyl, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), rod_rotation))


bob = chrono.ChVisualShapeSphere(0.15)
bob.SetColor(chrono.ChColor(1.0, 0.0, 0.0))
pendulum.AddVisualShape(bob, chrono.ChFramed(chrono.ChVector3d(pendulum_length / 2.0, 0, 0)))





revolute_joint = chrono.ChLinkLockRevolute()



revolute_joint.Initialize(
    ground,
    pendulum,
    chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT)
)
sys.AddLink(revolute_joint)





vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('PyChrono Simple Pendulum Simulation')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0, 6))   
vis.AddTypicalLights()





time_step = 0.01           
log_interval = 0.5         
last_log_time = 0.0

print("Starting Simple Pendulum Simulation...\n")

while vis.Run():
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    sys.DoStepDynamics(time_step)

    
    current_time = sys.GetChTime()
    if current_time - last_log_time >= log_interval:
        pos = pendulum.GetPos()
        vel = pendulum.GetPosDt()
        print(f"Time: {current_time:6.2f} s | "
              f"Position: ({pos.x:7.4f}, {pos.y:7.4f}, {pos.z:7.4f}) | "
              f"Velocity: ({vel.x:7.4f}, {vel.y:7.4f}, {vel.z:7.4f})")
        last_log_time = current_time