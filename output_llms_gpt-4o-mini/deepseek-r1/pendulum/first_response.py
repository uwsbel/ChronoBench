import pychrono as chrono
import pychrono.irrlicht as irrlicht
import math

# System setup
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Gravity in -Y direction

# Create ground body
ground = chrono.ChBodyEasyBox(0.2, 0.2, 0.2, 1000, True, False)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground.SetBodyFixed(True)
ground.GetVisualShape(0).SetColor(chrono.ChColor(0.8, 0.2, 0.2))
system.Add(ground)

# Pendulum parameters
length = 2.0    # Full length of pendulum
radius = 0.05   # Radius of cylindrical pendulum
density = 1000  # kg/m³

# Create pendulum body (cylinder oriented along Y axis)
pendulum = chrono.ChBodyEasyCylinder(radius, length, density, True, False)
pendulum.SetPos(chrono.ChVectorD(0, -length/2, 0))  # Position center at half-length
pendulum.SetRot(chrono.Q_from_AngZ(math.radians(30)))  # Initial 30° displacement
pendulum.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.8, 0.2))
system.Add(pendulum)

# Create revolute joint between ground and pendulum
revolute = chrono.ChLinkRevolute()
revolute.Initialize(ground, pendulum, 
                    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0),  # Joint position
                                     chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 0, 1))))  # Z axis
system.Add(revolute)

# Create Irrlicht visualization
vis = irrlicht.ChIrrApp(system, "Simple Pendulum", irrlicht.dimension2du(800, 600))
vis.AddTypicalLights()
vis.AddTypicalCamera(irrlicht.vector3df(0, -1, 3), irrlicht.vector3df(0, 0, 0))
vis.AssetBindAll()
vis.AssetUpdateAll()

# Simulation loop setup
time_step = 0.01
log_interval = 0.1
next_log_time = 0

while vis.GetDevice().run():
    vis.BeginScene()
    vis.DrawAll()
    vis.DoStepDynamics(time_step)
    vis.EndScene()
    
    current_time = system.GetChTime()
    
    # Log data at specified intervals
    if current_time >= next_log_time:
        # Get joint state
        angle = math.degrees(revolute.GetRelAngle())
        ang_vel = revolute.GetRelWvel().z
        
        # Calculate pendulum end position
        local_end = chrono.ChVectorD(0, -length/2, 0)  # Bottom end in local coordinates
        global_end = pendulum.TransformPointLocalToParent(local_end)
        
        print(f"Time: {current_time:.2f}s | "
              f"Angle: {angle:.2f}° | "
              f"Angular Velocity: {ang_vel:.2f} rad/s | "
              f"End Position: ({global_end.x:.2f}, {global_end.y:.2f}, {global_end.z:.2f})")
        
        next_log_time += log_interval