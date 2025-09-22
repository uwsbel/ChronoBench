import pychrono as chrono
import pychrono.irrlicht as chronoirr

# Initialize the PyChrono system
system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, -3.71, 0))  # Mars gravity

# Create materials for collision
mat_ground = chrono.ChContactMaterialNSC()
mat_ground.SetFriction(0.9)
mat_ground.SetRestitution(0.1)

mat_rover = chrono.ChContactMaterialNSC()
mat_rover.SetFriction(0.8)
mat_rover.SetRestitution(0.1)

# Create the ground
ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetCollide(True)
ground.GetCollisionModel().ClearModel()
chrono.ChCollisionModel.AddBox(ground, 100, 1, 100, chrono.ChVector3d(0, -0.5, 0), mat_ground)
ground.GetCollisionModel().BuildModel()
ground.SetPos(chrono.ChVector3d(0, 0, 0))

# Add visualization for the ground
ground_shape = chrono.ChVisualShape()
ground_shape.SetMesh(chrono.ChMesh().CreateBox(chrono.ChVector3d(100, 1, 100)))
ground_shape.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
ground.AddVisualShape(ground_shape)
system.Add(ground)

# Create the rover
rover = chrono.ChBody()
rover.SetMass(100)
rover.SetInertiaXX(chrono.ChVector3d(10, 10, 10))
rover.SetPos(chrono.ChVector3d(0, 1, 0))
rover.SetCollide(True)
rover.GetCollisionModel().ClearModel()
chrono.ChCollisionModel.AddBox(rover, 1, 0.5, 2, chrono.ChVector3d(0, 0, 0), mat_rover)
rover.GetCollisionModel().BuildModel()

# Add visualization for the rover
rover_shape = chrono.ChVisualShape()
rover_shape.SetMesh(chrono.ChMesh().CreateBox(chrono.ChVector3d(1, 0.5, 2)))
rover_shape.SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
rover.AddVisualShape(rover_shape)
system.Add(rover)

# Set up Irrlicht visualization
app = chronoirr.ChIrrApp(system, 'Curiosity Rover Simulation', chronoirr.dimension2du(1024, 768))
app.AddTypicalLogo()
app.AddTypicalCamera(chronoirr.vector3df(0, 5, -10), chronoirr.vector3df(0, 0, 0))
app.AddLightWithShadow(chronoirr.vector3df(5, 10, 5), chronoirr.vector3df(0, 0, 0), 10, 0.2, 10, 120, 512)
app.EnableShadows()

# Custom event receiver for input
class MyEventReceiver(chronoirr.IEventReceiver):
    def __init__(self):
        self.keys = {}
    def OnEvent(self, event):
        if event.EventType == chronoirr.EET_KEY_INPUT_EVENT:
            key = event.KeyInput.Key
            pressed = event.KeyInput.PressedDown
            self.keys[key] = pressed
        return False

receiver = MyEventReceiver()
app.SetUserEventReceiver(receiver)

# Simulation loop
step_size = 1e-2  # Adjusted for smoother simulation
while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()

    # Handle input
    throttle = 0
    steer = 0
    if receiver.keys.get(chronoirr.KEY_UP, False):
        throttle = 1
    if receiver.keys.get(chronoirr.KEY_DOWN, False):
        throttle = -1
    if receiver.keys.get(chronoirr.KEY_LEFT, False):
        steer = 1
    if receiver.keys.get(chronoirr.KEY_RIGHT, False):
        steer = -1

    # Apply forces/torques to rover
    force_magnitude = 200
    torque_magnitude = 100

    # Compute forward force direction (assuming Z-axis is forward)
    force_dir = rover.GetRot().GetZaxis()
    throttle_force = force_dir * (throttle * force_magnitude)
    rover.Accumulate_force(throttle_force, rover.GetPos(), chrono.VECT_PARALLEL)

    # Apply steering torque (around Y-axis)
    steer_torque = chrono.ChVector3d(0, steer * torque_magnitude, 0)
    rover.Accumulate_torque(steer_torque, chrono.VECT_PARALLEL)

    app.EndScene()
    system.DoStepDynamics(step_size)