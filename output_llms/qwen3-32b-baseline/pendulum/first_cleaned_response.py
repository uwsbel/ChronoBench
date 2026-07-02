import pychrono as chrono
from pychrono import irrlicht as chronoirr


system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBodyEasyBox(2, 0.1, 2, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground.SetBodyFixed(True)
system.Add(ground)


mass = 1  
radius = 0.1  
length = 1  
density = mass / (chrono.CH_PI * radius**2 * length)


pendulum = chrono.ChBodyEasyCylinder(radius, length, density, True, True, chrono.ChAxisRotation.eX)
pendulum.SetPos(chrono.ChVectorD(-length / 2, 0, 0))
system.Add(pendulum)


revolute_joint = chrono.ChLinkLockRevolute()
revolute_joint.Initialize(ground, pendulum, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)))
system.AddLink(revolute_joint)


application = chronoirr.ChIrrApp(system, 'Pendulum Simulation', chronoirr.dimension2du(800, 600))
application.SetSkyBox()
application.SetCamera(chronoirr.vector3df(0, 1, 3))
application.SetShowInfos(True)
application.SetVSync(True)
application.SetAmbientLight(chronoirr.SColorf(0.4, 0.4, 0.4, 1))
application.AddTypicalLights()


time_step = 0.001
log_interval = 0.05
next_log_time = 0.0


application.BeginScene()
application.DrawAll()
application.EndScene()

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    
    
    system.DoStepDynamics(time_step)
    
    
    current_time = system.GetChTime()
    if current_time >= next_log_time:
        pos = pendulum.GetPos()
        ang_vel = pendulum.GetWvel_abs()
        print(f"Time: {current_time:.2f}, Position: {pos}, Angular Velocity: {ang_vel}")
        next_log_time += log_interval
    
    application.EndScene()