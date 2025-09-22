import pychrono.core as chrono
import pychrono.irrlicht as irr



system = chrono.ChSystemNSC()


application = irr.ChIrrApp(system, "Simple Pendulum Simulation", irr.dimension2du(800, 600))
application.GetSceneManager().AddSkyBox()
application.GetCamera().SetChPos(chrono.ChVectorD(0, 0, 1))
application.AssetBindAll()
application.AssetUpdateAll()


ground = chrono.ChBodyEasyBox(2, 0.1, 2, 1000, True, True)
ground.SetPosition(chrono.ChVectorD(0, -0.05, 0))
ground.SetBodyFixed(True)
system.Add(ground)


mass = 1.0
radius = 0.05
length = 1.0


pendulum = chrono.ChBodyEasyCylinder(radius, length, mass, True, True)
pendulum.SetPos(chrono.ChVectorD(0, -length / 2, 0))
system.Add(pendulum)


pivot_point = chrono.ChVectorD(0, 0, 0)
revolute_joint = chrono.ChLinkRevolute()
revolute_joint.Initialize(ground, pendulum, chrono.ChCoordsysD(pivot_point))
system.Add(revolute_joint)



initial_angle_deg = 30
initial_angle_rad = chrono CH_C_PI/6  
pendulum.SetPos(chrono.ChVectorD(length * chrono.ChMathsProjects.Cos(initial_angle_rad),
                                 -length * chrono.ChMathsProjects.Sin(initial_angle_rad),
                                 0))

pendulum.SetW(chrono.ChVectorD(0, 0, 0))


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


time_step = 0.005
total_time = 10  


application.SetTimestep(time_step)
application.StartScene()
application.DrawAll()

current_time = 0.0
while current_time < total_time:
    system.DoStepDynamics(time_step)
    application.BeginScene()
    application.DrawAll()

    
    pos = pendulum.GetPos()
    vel = pendulum.GetW()
    print(f"Time: {current_time:.3f} s - Pendulum position: {pos}, angular velocity: {vel}")

    application.EndScene()

    current_time += time_step
    application.GetVideoDriver().DrawLine(chrono.ChVectorD(0,0,0), pos, irr.SColor(255,255,0,0))
    
application.GetDevice().closeDevice()