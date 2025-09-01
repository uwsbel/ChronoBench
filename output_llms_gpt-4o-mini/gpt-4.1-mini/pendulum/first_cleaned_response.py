import pychrono as chrono
import pychrono.irrlicht as chronoirr




system = chrono.ChSystemNSC()




ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetName("Ground")
system.Add(ground)


pendulum = chrono.ChBodyEasyBox(0.2, 1.0, 0.05,  
                                1000,           
                                True,           
                                True)           
pendulum.SetName("Pendulum")



pendulum.SetPos(chrono.ChVectorD(0, -0.5, 0))





system.Add(pendulum)


joint = chrono.ChLinkLockRevolute()
joint.Initialize(pendulum, ground, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 0, 1))))
system.AddLink(joint)


vis = chronoirr.ChIrrApp(system, "Simple Pendulum", chronoirr.dimension2du(800, 600))


vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(1.0, 0.0, 1.5))
vis.AddLightWithShadow(chrono.ChVectorD(2, 4, 2), chrono.ChVectorD(0, 0, 0), 9, 1, 9, 30)
vis.AddShadowAll()

vis.AssetBindAll()
vis.AssetUpdateAll()


time_step = 1e-3
next_log_time = 0
log_interval = 0.1

vis.SetTimestep(time_step)

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    system.DoStepDynamics(time_step)

    
    current_time = system.GetChTime()
    if current_time >= next_log_time:
        pos = pendulum.GetPos()
        vel = pendulum.GetPos_dt()
        print(f"Time: {current_time:.2f} s, Position: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}), "
              f"Velocity: ({vel.x:.3f}, {vel.y:.3f}, {vel.z:.3f})")
        next_log_time += log_interval