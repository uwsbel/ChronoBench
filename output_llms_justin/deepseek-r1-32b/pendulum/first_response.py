importchrono
import chronoirr

# Initialize the Chrono system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravity

# Create Irrlicht application
application = chronoirr.ChIrrApp(system, "Simple Pendulum", chronoirr.dimension2du(800, 600))
application.AddTypicalIrrlichtLight()
application.AddTypicalIrrlichtCamera(chrono.ChVectorD(0, 5, 5), chrono.ChVectorD(0, 0, 0))

# Create ground body
ground = chrono.ChBody()
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground.SetMass(0)  # Static body
ground.GetCollisionModel().AddBox(chrono.ChCollisionModel(), chrono.ChVectorD(10, 1, 10))
ground.GetVisualShape().SetMaterial(chrono.ChVisualMaterial(chrono.ChColor(0.5, 0.5, 0.5)))
system.Add(ground)

# Create pendulum body
pendulum = chrono.ChBody()
pendulum.SetPos(chrono.ChVectorD(0, 2, 0))
pendulum.SetMass(1.0)  # Mass of 1 kg
pendulum.GetInertia().SetIdentity(0.1)  # Inertia tensor
pendulum.GetCollisionModel().AddSphere(chrono.ChCollisionModel(), 0.2)
pendulum.GetVisualShape().SetMaterial(chrono.ChVisualMaterial(chrono.ChColor(0.8, 0, 0)))
system.Add(pendulum)

# Create revolute joint between ground and pendulum
joint = chrono.ChLinkRevolute()
joint.Initialize(ground, pendulum, chrono.ChCoordsysD(
    chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33D(chrono.ChVectorD(1, 0, 0), chrono.ChVectorD(0, 1, 0), chrono.ChVectorD(0, 0, 1))))
joint.SetAngleX(45 * chrono.CH_C_DEG_TO_RAD)  # Initial angle of 45 degrees
system.Add(joint)

# Add visualization assets
pendulum.GetVisualShape().AddSphere(0.2, chrono.ChColor(0.8, 0, 0))

# Add constraint to prevent vertical movement
constraint = chrono.ChLinkLockPointToLine()
constraint.Initialize(ground, pendulum, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))
system.Add(constraint)

# Simulation parameters
time_step = 0.001
total_time = 10.0

# Run simulation
application.Begin()
while application.GetDevice().Run():
    time = system.GetChTime()
    
    if time >= total_time:
        break
        
    # Update system
    system.DoStepDynamics(time_step)
    
    # Render scene
    application.Render()
    
    # Log position and velocity every 0.1 seconds
    if time % 0.1 == 0:
        pos = pendulum.GetPos()
        vel = pendulum.GetPos_dt()
        print(f"Time: {time:.2f}s | Position: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}) | Velocity: ({vel.x:.2f}, {vel.y:.2f}, {vel.z:.2f})")
        
    # Check for key press to exit
    if application.GetDevice().IsKeyDown(chronoirr.KEY_ESCAPE):
        break

application.End()