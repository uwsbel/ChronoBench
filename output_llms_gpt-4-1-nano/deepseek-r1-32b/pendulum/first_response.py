importchrono
import chronoirr

# Initialize the Chrono system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Gravity

# Create Irrlicht application
application = chronoirr.ChIrrApp(system, "Simple Pendulum", chronoirr.dimension2du(800, 600))
application.AddTypicalIrrlichtLogo()
application.AddTypicalLights()
application.SetCameraPosition(chrono.ChVectorD(0, 3, 3))
application.SetCameraLookAt(chrono.ChVectorD(0, 0, 0))

# Create ground body
ground = chrono.ChBody()
ground.SetMass(0)  # Static body
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground.GetCollisionModel().AddBox(chrono.ChCollisionModel.BoxData(4, 1, 4))
ground.GetMaterialSurface().SetFriction(0.5)
system.AddBody(ground)

# Create pendulum body
pendulum = chrono.ChBody()
pendulum.SetMass(1)  # Mass of 1 kg
pendulum.SetInertiaXX(chrono.ChVectorD(0.2, 0.2, 0.2))  # Inertia tensor for a sphere
pendulum.SetPos(chrono.ChVectorD(0, 1, 0))  # Start position above ground

# Add collision and visualization shapes
pendulum.GetCollisionModel().AddCylinder(chrono.ChCollisionModel.CylinderData(0.1, 0.2))
pendulum.GetVisualShape().AddCylinder(chrono.ChColor(0.8, 0, 0), 0.1, 0.2)

system.AddBody(pendulum)

# Create revolute joint between ground and pendulum
joint = chrono.ChLinkRevolute()
joint.Initialize(ground, pendulum, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(1, 0, 0)))
system.AddLink(joint)

# Add damping force for realism
damping = chrono.ChDampingForce()
damping.SetCoefficient(0.1)
pendulum.AddForce(damping)

# Visualization settings
application.SetRenderMode(chronoirr.RENDER_MODE_OPENGL)
application.SetBackgroundColor(chrono.ChColor(0.1, 0.1, 0.1))
application.GetCamera().SetFov(60)
application.GetCamera().SetNearClip(0.1)

# Function to log pendulum state
def log_state(system, time):
    pos = pendulum.GetPos()
    vel = pendulum.GetPos_dt()
    print(f"Time: {time:.2f}s | Position: {pos} | Velocity: {vel}")

# Run simulation
application.SetTimestep(0.001)
application.SetPostStepCallback(log_state, 0.1)  # Log every 0.1 seconds

while application.Run():
    application.BeginScene()
    application.DrawAll()
    application.EndScene()