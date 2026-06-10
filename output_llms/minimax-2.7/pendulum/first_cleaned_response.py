import pychrono as chrono
import pychrono.irrlicht as chronoirr

def main():
    print("=== PyChrono Simple Pendulum Simulation ===\n")
    
    
    
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  
    
    
    system.SetSolverType(chrono.ChSolver.Type_SOR)
    system.SetMaxItersSolverSpeed(50)
    system.Set timestep(0.001)
    system.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT)
    
    print("Physical system created with gravity: (0, -9.81, 0)")
    
    
    ground = chrono.ChBodyEasyBox(10.0, 1.0, 10.0, 1000, True, False)
    ground.SetPos(chrono.ChVectorD(0, -0.5, 0))
    ground.SetFixed(True)
    ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.png"))
    system.Add(ground)
    print("Ground body created: 10x1x10 box, fixed at (0, -0.5, 0)")
    
    
    
    pendulum_length = 2.0  
    bob_radius = 0.2       
    bob_density = 1000      
    
    
    bob_volume = (4/3) * 3.14159 * bob_radius**3
    bob_mass = bob_density * bob_volume
    bob_inertia = 0.4 * bob_mass * bob_radius**2  
    
    
    pendulum = chrono.ChBody()
    
    
    sphere_shape = chrono.ChSphereShape(bob_radius)
    pendulum.AddVisualShape(sphere_shape)
    
    
    collision_model = chrono.ChCollisionModel()
    collision_model.SetSafeMargin(0.01)
    pendulum.AttachCollisionModel(collision_model)
    
    
    pendulum.SetMass(bob_mass)
    pendulum.SetInertiaXX(chrono.ChVectorD(bob_inertia, bob_inertia, bob_inertia))
    
    
    initial_angle = 45.0  
    initial_angle_rad = chrono.CH_DEG_TO_RAD * initial_angle
    pendulum.SetPos(chrono.ChVectorD(
        pendulum_length * chrono.cos(initial_angle_rad),
        pendulum_length * chrono.sin(initial_angle_rad),
        0
    ))
    pendulum.SetFixed(False)
    
    
    system.Add(pendulum)
    print(f" Pendulum bob created:")
    print(f"   - Mass: {bob_mass:.3f} kg")
    print(f"   - Radius: {bob_radius} m")
    print(f"   - Length from pivot: {pendulum_length} m")
    print(f"   - Initial angle: {initial_angle} degrees")
    
    
    
    pivot = chrono.ChVectorD(0, 0, 0)
    
    
    joint = chrono.ChLinkLockRevolute()
    joint.Initialize(ground, pendulum, 
                     chrono.ChCoordsysD(pivot, chrono.Q_ROTATE_Z_TO_X))
    system.AddLink(joint)
    print(f"\nRevolute joint created at pivot point: ({pivot.x}, {pivot.y}, {pivot.z})")
    
    
    print("\nInitializing Irrlicht visualization...")
    
    
    vis = chronoirr.ChIrrApp(
        system,
        "PyChrono Pendulum Simulation",
        chronoirr.dimension2du(1280, 720)
    )
    
    
    vis.AddTypicalCamera(chronoirr.vector3d(4, 3, 5), chronoirr.vector3d(0, 1.5, 0))
    vis.AddTypicalLight()
    vis.AddTypicalSky()
    
    
    ground_plane = chronoirr.ChIrrCableRobot(ground)
    
    
    vis.AssetBindAll()
    vis.AssetUpdateAll()
    
    
    print("\n" + "=" * 60)
    print("Starting simulation with logging...")
    print("=" * 60)
    print(f"{'Time (s)':<12} {'Angle (deg)':<15} {'Velocity (rad/s)':<18} {'Position (m)':<15}")
    print("-" * 60)
    
    
    total_time = 5.0  
    output_interval = 0.05  
    next_log_time = 0.0
    
    
    simulation_step = 0
    while vis.Run():
        
        if system.GetChTime() >= total_time:
            print("\nSimulation complete!")
            break
        
        
        vis.BeginScene()
        vis.DrawAll()
        
        
        vis.AddLogo(chronoirr.vector3d(1.5, 1.0, 0))
        
        
        vis.DoStep()
        
        
        if system.GetChTime() >= next_log_time:
            
            pos = pendulum.GetPos()
            angle = chrono.CH_RAD_TO_DEG * chrono.atan2(pos.x, pos.y)
            angular_vel = joint.GetRelWvel().z
            pendulum_vel = pendulum.GetPos_dt()
            speed = pendulum_vel.Length()
            
            
            print(f"{system.GetChTime():<12.3f} {angle:<15.2f} {angular_vel:<18.4f} "
                  f"({pos.x:<8.3f}, {pos.y:<8.3f}, {pos.z:<8.3f})")
            
            next_log_time += output_interval
        
        vis.EndScene()
        simulation_step += 1
    
    
    print("\n" + "=" * 60)
    print("Simulation Statistics:")
    print("=" * 60)
    print(f"Total simulation time: {system.GetChTime():.3f} seconds")
    print(f"Total simulation steps: {simulation_step}")
    print(f"Average timestep: {system.GetChTime()/simulation_step:.6f} seconds")
    
    
    final_pos = pendulum.GetPos()
    final_angle = chrono.CH_RAD_TO_DEG * chrono.atan2(final_pos.x, final_pos.y)
    print(f"\nFinal pendulum angle: {final_angle:.2f} degrees")
    print(f"Final pendulum position: ({final_pos.x:.3f}, {final_pos.y:.3f}, {final_pos.z:.3f}) m")
    
    print("\nSimulation finished successfully!")

if __name__ == "__main__":
    main()