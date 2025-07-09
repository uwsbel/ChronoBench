import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math


chrono.SetChronoDataPath('/path/to/chrono/data')


sys = chrono.ChSystemNSC()


ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetPos(chrono.ChVector3d(0, -10, 0))
sys.Add(ground)


pendulum = chrono.ChBody()
pendulum.SetMass(1)
pendulum.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))
pendulum.SetPos(chrono.ChVector3d(0, 0, 0))
sys.Add(pendulum)


joint = chrono.ChLinkLockRevolute()
joint.Initialize(ground, pendulum, chrono.ChCoordsysd(chrono.ChVector3d(0, -10, 0), chrono.QuatFromAngleX(chrono.CH_PI / 2)))
sys.Add(joint)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Simple Pendulum Simulation')
vis.Initialize()


vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()


vis.AddCamera(chrono.ChVector3d(0, 0, 10))
vis.AddTypicalLights()


time_step = 0.01
t = 0
while vis.Run():
    sys.DoStepDynamics(time_step)
    t += time_step
    
    
    if t % 1.0 < time_step:
        print(f"Time: {t:.2f}, Pendulum Position: {pendulum.GetPos()}, Pendulum Velocity: {pendulum.GetPos_dt()}")

    vis.BeginScene()
    vis.Render()
    vis.EndScene()