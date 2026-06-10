import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math




print("Initializing PyChrono Pendulum Simulation...")


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  




ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetName("Ground")


ground_shape = chrono.ChBoxShape()
ground_shape.GetBoxGeometry().SetLengths(chrono.ChVectorD(2, 0.1, 2))
ground.AddVisualShape(ground_shape, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))


pivot_shape = chrono.ChSphereShape()
pivot_shape.GetSphereGeometry().rad = 0.05
ground.AddVisualShape(pivot_shape, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))

system.Add(ground)




pendulum_length = 1.5       
pendulum_mass   = 1.0       
pendulum_radius = 0.05      

pendulum = chrono.ChBody()
pendulum.SetName("Pendulum")
pendulum.SetMass(pendulum_mass)




initial_angle = math.radians(45)
com_x = (pendulum_length / 2) * math.sin(initial_angle)
com_y = -(pendulum_length / 2) * math.cos(initial_angle)
pendulum.SetPos(chrono.ChVectorD(com_x, com_y, 0))


I_rod = (1.0 / 12.0) * pendulum_mass * pendulum_length**2
pendulum.SetInertiaXX(chrono.ChVectorD(I_rod, I_rod * 0.01, I_rod))


initial_rot = chrono.ChQuaternionD()
initial_rot.Q_from_AngZ(initial_angle)
pendulum.SetRot(initial_rot)


cyl_shape = chrono.ChCylinderShape()
cyl_shape.GetCylinderGeometry().rad = pendulum_radius
cyl_shape.GetCylinderGeometry().p1 = chrono.ChVectorD(0,  pendulum_length / 2, 0)
cyl_shape.GetCylinderGeometry().p2 = chrono.ChVectorD(0, -pendulum_length / 2, 0)
pendulum.AddVisualShape(cyl_shape)


bob_shape = chrono.ChSphereShape()
bob_shape.GetSphereGeometry().rad = 0.1

pendulum.AddVisualShape(
    bob_shape,
    chrono.ChFrameD(chrono.ChVectorD(0, -pendulum_length / 2, 0))
)

system.Add(pendulum)





revolute_joint = chrono.ChLinkLockRevolute()


joint_frame = chrono.ChCoordsysD(
    chrono.ChVectorD(0, 0, 0),   
    chrono.Q_from_AngX(0)         
)

revolute_joint.Initialize(pendulum, ground, joint_frame)
revolute_joint.SetName("RevoluteJoint")
system.Add(revolute_joint)




application = chronoirr.ChIrrApp(
    system,
    "Simple Pendulum Simulation",
    chronoirr.dimension2du(1024, 768)
)

application.AddTypicalSky()
application.AddTypicalLogo()
application.AddTypicalCamera(chronoirr.vector3df(0, 0, 4))
application.AddTypicalLights()


application.AssetBindAll()
application.AssetUpdateAll()




time_step    = 0.001   
log_interval = 0.1     
next_log_time = 0.0

print("\n{'='*60}")
print("Starting Pendulum Simulation")
print(f"  Pendulum Length : {pendulum_length} m")
print(f"  Pendulum Mass   : {pendulum_mass} kg")
print(f"  Initial Angle   : {math.degrees(initial_angle):.1f} degrees")
print(f"  Time Step       : {time_step} s")
print("="*60)
print(f"{'Time (s)':>10} | {'Pos X (m)':>10} | {'Pos Y (m)':>10} | "
      f"{'Vel X (m/s)':>12} | {'Vel Y (m/s)':>12} | {'Speed (m/s)':>12}")
print("-" * 75)

application.SetTimestep(time_step)
application.SetTryRealtime(True)




while application.GetDevice().run():
    application.BeginScene(True, True, chronoirr.SColor(255, 140, 161, 192))
    application.DrawAll()
    application.DoStep()
    application.EndScene()

    current_time = system.GetChTime()

    
    if current_time >= next_log_time:
        pos = pendulum.GetPos()
        vel = pendulum.GetPos_dt()
        speed = vel.Length()

        
        bob_local = chrono.ChVectorD(0, -pendulum_length / 2, 0)
        bob_world = pendulum.TransformPointLocalToParent(bob_local)

        print(f"{current_time:>10.3f} | "
              f"{bob_world.x:>10.4f} | "
              f"{bob_world.y:>10.4f} | "
              f"{vel.x:>12.4f} | "
              f"{vel.y:>12.4f} | "
              f"{speed:>12.4f}")

        next_log_time += log_interval

    
    if current_time >= 30.0:
        print("\nSimulation completed (30 seconds elapsed).")
        break

print("\nSimulation finished.")