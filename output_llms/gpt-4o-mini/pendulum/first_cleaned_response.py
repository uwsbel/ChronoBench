import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


chrono.ChSystemNSC.SetNumThreads(1)
system = chrono.ChSystemNSC()


application = chronoirr.ChIrrApp(system, "Simple Pendulum Simulation", chronoirr.dimension2du(800, 600))
application.AddLogo()
application.AddSkyBox()
application.AddCamera(chrono.ChVectorD(0, 2, -5))
application.AddLight(chrono.ChVectorD(0, 10, -10), chrono.ChColor(1, 1, 1))
application.SetShowInfos(True)


ground = chrono.ChBodyEasyBox(10, 0.1, 10, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -0.05, 0))
ground.SetBodyFixed(True)
system.Add(ground)


pendulum_length = 2.0
pendulum_mass = 1.0
pendulum_inertia = (1/3) * pendulum_mass * pendulum_length**2

pendulum = chrono.ChBodyEasyBox(0.1, pendulum_length, 0.1, pendulum_mass, True, True)
pendulum.SetPos(chrono.ChVectorD(0, -pendulum_length / 2, 0))
pendulum.SetInertiaXX(chrono.ChVectorD(pendulum_inertia, pendulum_inertia, pendulum_inertia))
system.Add(pendulum)


joint = chrono.ChLinkLockRevolute()
joint.Initialize(ground, pendulum, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
system.Add(joint)


pendulum.SetPos(chrono.ChVectorD(0, -pendulum_length / 2, 0))
pendulum.SetPos_dt(chrono.ChVectorD(0, 0, 0))  


time_step = 0.01
end_time = 10.0


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.EndScene()

    
    system.DoStepDynamics(time_step)

    
    pendulum_position = pendulum.GetPos()
    pendulum_velocity = pendulum.GetPos_dt()
    print(f"Time: {system.GetChTime():.2f}, Position: {pendulum_position}, Velocity: {pendulum_velocity}")

    
    if system.GetChTime() >= end_time:
        break


application.Close()