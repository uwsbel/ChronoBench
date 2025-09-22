import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math


chrono.SetChronoDataPath('/path/to/chrono/data/')  


system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(ground)


ground_shape = chrono.ChBoxShape()
ground_shape.GetBoxGeometry().Size = chrono.ChVectorD(2, 0.1, 0.5)
ground.AddVisualShape(ground_shape, chrono.ChFrameD())


pendulum_length = 2.0  
pendulum_mass = 1.0    
pendulum_radius = 0.1  


pendulum = chrono.ChBody()
pendulum.SetMass(pendulum_mass)


initial_angle = math.pi / 6  
pendulum_pos = chrono.ChVectorD(
    pendulum_length * math.sin(initial_angle),
    -pendulum_length * math.cos(initial_angle),
    0
)
pendulum.SetPos(pendulum_pos)


inertia = pendulum_mass * pendulum_length * pendulum_length
pendulum.SetInertiaXX(chrono.ChVectorD(inertia, inertia, inertia))


pendulum_shape = chrono.ChSphereShape()
pendulum_shape.GetSphereGeometry().rad = pendulum_radius
pendulum.AddVisualShape(pendulum_shape, chrono.ChFrameD())


pendulum.GetCollisionModel().ClearModel()
pendulum.GetCollisionModel().AddSphere(chrono.ChMaterialSurface.DefaultMaterial(chrono.ChContactMethod_NSC), pendulum_radius)
pendulum.GetCollisionModel().BuildModel()
pendulum.SetCollide(True)

system.Add(pendulum)



revolute_joint = chrono.ChLinkRevolute()
revolute_joint.Initialize(
    ground,
    pendulum,
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0))
)
system.Add(revolute_joint)


arm_shape = chrono.ChCylinderShape()
arm_shape.GetCylinderGeometry().rad = 0.02
arm_shape.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0, 0)
arm_shape.GetCylinderGeometry().p2 = chrono.ChVectorD(0, -pendulum_length, 0)


arm_body = chrono.ChBody()
arm_body.SetBodyFixed(True)
arm_body.AddVisualShape(arm_shape, chrono.ChFrameD())
system.Add(arm_body)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Simple Pendulum Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(4, 2, 6))
vis.AddTypicalLights()


time_step = 0.01
simulation_time = 0.0
log_interval = 1.0  
next_log_time = 0.0

print("Starting pendulum simulation...")
print("Time\tPosition (x, y, z)\t\tVelocity (x, y, z)")
print("-" * 70)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    
    
    pendulum_pos = pendulum.GetPos()
    arm_rotation = chrono.ChQuaternionD()
    arm_rotation.Q_from_AngAxis(-math.atan2(pendulum_pos.x, -pendulum_pos.y), chrono.ChVectorD(0, 0, 1))
    arm_body.SetPos(chrono.ChVectorD(pendulum_pos.x/2, pendulum_pos.y/2, 0))
    arm_body.SetRot(arm_rotation)
    
    
    if simulation_time >= next_log_time:
        pos = pendulum.GetPos()
        vel = pendulum.GetPos_dt()
        print(f"{simulation_time:.2f}\t({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})\t\t({vel.x:.3f}, {vel.y:.3f}, {vel.z:.3f})")
        next_log_time += log_interval
    
    
    system.DoStepDynamics(time_step)
    simulation_time += time_step
    
    vis.EndScene()

print("Simulation completed.")