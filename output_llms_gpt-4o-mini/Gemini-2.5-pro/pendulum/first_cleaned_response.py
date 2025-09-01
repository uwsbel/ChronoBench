import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math






my_system = chrono.ChSystemNSC() 


my_system.SetGravitationalAcceleration(chrono.ChVectorD(0, -9.81, 0))






chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.005)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.005)




pendulum_length = 2.0  
pendulum_mass = 1.0    
bob_radius = 0.1       
initial_angle_deg = 45 


pivot_point_pos = chrono.ChVectorD(0, pendulum_length + 0.5, 0) 




ground_body = chrono.ChBody()
ground_body.SetBodyFixed(True) 
my_system.Add(ground_body)


ground_box = chrono.ChBoxShape(4, 0.2, 4)
ground_box.SetColor(chrono.ChColor(0.4, 0.4, 0.4))
ground_body.AddVisualShape(ground_box, chrono.ChFrameD(chrono.ChVectorD(0, -0.1, 0))) 




pendulum_bob = chrono.ChBody()
pendulum_bob.SetMass(pendulum_mass)


initial_angle_rad = math.radians(initial_angle_deg)
bob_initial_x = pivot_point_pos.x + pendulum_length * math.sin(initial_angle_rad)
bob_initial_y = pivot_point_pos.y - pendulum_length * math.cos(initial_angle_rad)
bob_initial_z = pivot_point_pos.z
pendulum_bob.SetPos(chrono.ChVectorD(bob_initial_x, bob_initial_y, bob_initial_z))


inertia_val = (2.0/5.0) * pendulum_mass * bob_radius**2
pendulum_bob.SetInertiaXX(chrono.ChVectorD(inertia_val, inertia_val, inertia_val))


bob_sphere = chrono.ChSphereShape(bob_radius)
bob_sphere.SetColor(chrono.ChColor(0.8, 0.2, 0.2)) 
pendulum_bob.AddVisualShape(bob_sphere)

my_system.Add(pendulum_bob)










joint_frame = chrono.ChFrameD(pivot_point_pos, chrono.QUNIT) 

revolute_joint = chrono.ChLinkRevolute()
revolute_joint.Initialize(pendulum_bob,  
                          ground_body,   
                          joint_frame)   
my_system.Add(revolute_joint)




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(my_system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Simple Pendulum Simulation - PyChrono')
vis.Initialize()


vis.AddCamera(chrono.ChVectorD(1.5, pendulum_length, 3.5),  
              chrono.ChVectorD(0, pendulum_length * 0.7, 0)) 


vis.AddSkyBox()
vis.AddTypicalLights()




time_step = 0.01
simulation_time = 10.0  
log_frequency = 0.1   
next_log_time = 0.0

print("Starting simulation...")
print(f"{'Time (s)':<10} | {'Pos X (m)':<12} | {'Pos Y (m)':<12} | {'Pos Z (m)':<12} | {'Vel X (m/s)':<12} | {'Vel Y (m/s)':<12} | {'Vel Z (m/s)':<12}")
print("-" * 90)

while vis.Run():
    current_time = my_system.GetChTime()

    vis.BeginScene(True, True, chrono.ChColor(0.1, 0.1, 0.15)) 
    vis.Render()
    
    
    
    vis.EndScene()

    my_system.DoStepDynamics(time_step)

    
    if current_time >= next_log_time:
        pos = pendulum_bob.GetPos()
        vel = pendulum_bob.GetPos_dt() 
        

        print(f"{current_time:<10.2f} | "
              f"{pos.x:<12.3f} | {pos.y:<12.3f} | {pos.z:<12.3f} | "
              f"{vel.x:<12.3f} | {vel.y:<12.3f} | {vel.z:<12.3f}")
        next_log_time += log_frequency

    if current_time > simulation_time:
        vis.GetDevice().closeDevice() 

print("Simulation finished.")