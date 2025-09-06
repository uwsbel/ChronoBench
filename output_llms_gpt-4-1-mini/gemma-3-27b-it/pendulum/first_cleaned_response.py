import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math


sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


body_ground = chrono.ChBody()
body_ground.SetBodyFixed(True)
body_ground.SetPos(chrono.ChVector3d(0, -1, 0))
body_ground.SetMass(1000)  
body_ground.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
body_ground.AddVisualShape(chrono.ChVisualShapeBox(10, 1, 10))
sys.Add(body_ground)


pendulum_length = 1.0
pendulum_mass = 1.0
pendulum_radius = 0.1

body_pendulum = chrono.ChBody()
body_pendulum.SetMass(pendulum_mass)
body_pendulum.SetPos(chrono.ChVector3d(0, 0, 0))
body_pendulum.SetInertiaXX(chrono.ChVector3d(pendulum_radius**2, pendulum_radius**2, pendulum_radius**2))
body_pendulum.AddVisualShape(chrono.ChVisualShapeSphere(pendulum_radius))
sys.Add(body_pendulum)


joint_pendulum = chrono.ChLinkLockRevolute()
joint_pendulum.Initialize(body_ground, body_pendulum, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.Q_from_AngAxis(chrono.CH_PI/2, chrono.ChVector3d(1,0,0))))
sys.Add(joint_pendulum)


initial_angle = math.pi / 4  
body_pendulum.SetPos(chrono.ChVector3d(pendulum_length * math.sin(initial_angle), 0, pendulum_length * math.cos(initial_angle)))


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Simple Pendulum')
vis.Initialize()
vis.AddCamera(chrono.ChVector3d(5, 5, -5))
vis.AddTypicalLights()


time_step = 0.01
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    sys.DoStepDynamics(time_step)

    
    pos = body_pendulum.GetPos()
    vel = body_pendulum.GetPos_WVEL()
    print(f"Time: {sys.GetChTime()}, Pendulum Position: {pos}, Velocity: {vel}")