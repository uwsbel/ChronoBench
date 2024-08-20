import pychrono as chrono
import pychrono.irrlicht as irr
import math


sys = chrono.ChSystemNSC()


ground = chrono.ChBody()
sys.AddBody(ground)


pendulum = chrono.ChBody()
sys.AddBody(pendulum)


pendulum.SetMass(1.0)
pendulum.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))


pendulum.SetPos(chrono.ChVector3d(1.0, 0, 0))
pendulum.SetRot(chrono.QuatFromAngleAxis(1.0, chrono.ChVector3d(0, 1, 0)))


joint = chrono.ChLinkLockRevolute()
joint.Initialize(ground, pendulum, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
sys.AddLink(joint)


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(chrono.ChVector2i(1024, 768))
vis.SetWindowTitle('Simple Pendulum Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2, -5), chrono.ChVector3d(0, 1, 0))
vis.AddTypicalLights()
vis.AddLightDirectional()


time_step = 0.01


while vis.Run():
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    pos = pendulum.GetPos()
    vel = pendulum.GetPosDt()
    print(f"Time: {sys.GetChTime():.2f}, Pendulum Position: {pos}, Velocity: {vel}")

    
    sys.DoStepDynamics(time_step)