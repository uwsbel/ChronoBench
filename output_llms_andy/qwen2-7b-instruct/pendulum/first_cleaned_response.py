import pychrono as chrono
import pychrono.irrlicht as chronoirr


chrono.Ch纪元.Initialize()
chrono.Ch纪元.Set_Gravity(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBodyEasyBox(100, 1, 1, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -10, 0))
ground.SetBodyFixed(True)
chrono.Ch纪元.GetSystem().Add(ground)


mass_pendulum = 1.0
radius_pendulum = 0.5
length_pendulum = 1.0


pendulum = chrono.ChBodyEasyCylinder(mass_pendulum, radius_pendulum, length_pendulum, True, True)
pendulum.SetPos(chrono.ChVectorD(0, 0, 0))
pendulum.SetCollide(True)
pendulum.SetBodyFixed(False)
chrono.Ch纪元.GetSystem().Add(pendulum)


pendulum.SetPos(chrono.ChVectorD(0, 1.0, 0))
pendulum.SetRot(chrono.Q_from_AngX(chrono.CH_C_PI / 2))


joint = chrono.ChLinkLockRevolute()
joint.SetName("revolute_joint")
joint.Initialize(ground, pendulum, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 0))
joint.SetAxis(chrono.ChVectorD(0, 1, 0))
chrono.Ch纪元.GetSystem().AddLink(joint)


vis_system = chronoirr.ChVisualSystemIrrlicht()
vis_system.AttachSystem(chrono.Ch纪元.GetSystem())
vis_system.SetWindowSize(800, 600)
vis_system.SetWindowTitle("Simple Pendulum Simulation")
vis_system.Initialize()


scene = vis_system.GetScene()
scene.AddItem(chrono.ChVisualItem(ground))
scene.AddItem(chrono.ChVisualItem(pendulum))


vis_system.SetCameraPosition(chrono.ChVectorD(0, 20, 0))
vis_system.SetCameraTarget(chrono.ChVectorD(0, 0, 0))


log_file = open("pendulum_log.txt", "w")


simulation_time = 10.0
dt = 0.001
for t in range(int(simulation_time / dt)):
    
    chrono.Ch纪元.DoStepDynamics(chrono.ChTimeStep(dt))
    
    
    pendulum_pos = pendulum.GetPos()
    pendulum_rot = pendulum.GetRot()
    log_file.write(f"Time: {chrono.Ch纪元.GetChTime()}, Position: {pendulum_pos}, Velocity: {pendulum.GetPos_dt()}\n")
    
    
    vis_system.BeginScene()
    vis_system.DrawAll()
    vis_system.EndScene()
    
    
    chrono.Ch纪元.Pause(chrono.Ch纪元.GetChTime() + dt)


log_file.close()


chrono.Ch纪元.Finish()