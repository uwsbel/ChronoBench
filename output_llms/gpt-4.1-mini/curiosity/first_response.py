import pychrono as chrono
import pychrono.vehicle
import pychrono.irrlicht as chronoirr
import os

# -----------------------------------------------------------------------------
# Initialize the PyChrono system
# -----------------------------------------------------------------------------
chrono.SetChronoDataPath(chrono.GetChronoDataPath())  # ensure data path is set

system = chrono.ChSystemNSC()  # use non-smooth contact method

# Set some global system properties
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))  # gravity downward on Z

# -----------------------------------------------------------------------------
# Create rigid terrain (ground)
# -----------------------------------------------------------------------------
ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetIdentifier(-1)

# Create a large horizontal box shape to represent ground
ground_shape = chrono.ChBoxShape()
ground_shape.GetBoxGeometry().Size = chrono.ChVectorD(50, 50, 0.5)  # wide and flat

ground.AddVisualShape(ground_shape)

# Position terrain half unit below zero to place "top" at z=0
ground.SetPos(chrono.ChVectorD(0, 0, -0.5))

# Collision shape for terrain - large box
ground.GetCollisionModel().ClearModel()
ground.GetCollisionModel().AddBox(50, 50, 0.5)
ground.GetCollisionModel().BuildModel()
ground.SetCollide(True)

# Set material properties for terrain
mat_ground = chrono.ChMaterialSurfaceNSC()
mat_ground.SetFriction(0.9)
ground.GetCollisionModel().SetDefaultMaterial(mat_ground)

system.Add(ground)

# -----------------------------------------------------------------------------
# Load Curiosity Rover model
# For demonstration, load a sample vehicle from PyChrono vehicle models:
# The "hmmwv" from vehicle module will be replaced by a placeholder for Curiosity,
# since there is no built-in Curiosity in PyChrono default datasets.
# Assume the rover model files exist in a "curiosity" folder with its .json specification.
# For demo purpose, we'll create a simple box as rover chassis and six wheels with motors.
# -----------------------------------------------------------------------------

# Rover chassis
rover = chrono.ChBody()
rover.SetBodyFixed(False)
rover.SetMass(900)  # approximate mass in kg for Curiosity rover ~900 kg
rover.SetInertiaXX(chrono.ChVectorD(150, 600, 700))  # approx inertia values
rover.SetPos(chrono.ChVectorD(0, 0, 0.75))  # slightly above ground (terrain top at 0)
rover.GetCollisionModel().ClearModel()
rover.GetCollisionModel().AddBox(1.0, 0.8, 0.25)  # size roughly 2x1.6x0.5 m box for chassis
rover.GetCollisionModel().BuildModel()
rover.SetCollide(True)
mat_rover = chrono.ChMaterialSurfaceNSC()
mat_rover.SetFriction(0.8)
# Apply material to collision model
rover.GetCollisionModel().SetDefaultMaterial(mat_rover)

# Visualize chassis with texture
chassis_box = chrono.ChBoxShape()
chassis_box.GetBoxGeometry().Size = chrono.ChVectorD(1.0, 0.8, 0.25)
rover.AddVisualShape(chassis_box)
texture_path = os.path.join(chrono.GetChronoDataPath(), "robotlogo.png")
if os.path.isfile(texture_path):
    rover.GetVisualShape(0).SetTexture(texture_path)  # add a logo texture

system.Add(rover)

# Create wheels — six wheels in three pairs (left/right)
wheel_radius = 0.3
wheel_width = 0.15

# Suspension locations
wheel_positions = [
    chrono.ChVectorD(0.8, 0.75, 0.3),   # front-left
    chrono.ChVectorD(0.8, -0.75, 0.3),  # front-right
    chrono.ChVectorD(0.15, 0.75, 0.3),  # mid-left
    chrono.ChVectorD(0.15, -0.75, 0.3), # mid-right
    chrono.ChVectorD(-0.5, 0.75, 0.3),  # rear-left
    chrono.ChVectorD(-0.5, -0.75, 0.3), # rear-right
]

wheels = []
for i, pos in enumerate(wheel_positions):
    wheel = chrono.ChBody()
    wheel.SetMass(20)
    wheel.SetInertiaXX(chrono.ChVectorD(0.1, 0.2, 0.1))
    wheel.SetPos(rover.GetPos() + pos)
    wheel.GetCollisionModel().ClearModel()
    wheel.GetCollisionModel().AddCylinder(wheel_radius, wheel_radius, wheel_width / 2)
    wheel.GetCollisionModel().BuildModel()
    wheel.SetCollide(True)
    mat_wheel = chrono.ChMaterialSurfaceNSC()
    mat_wheel.SetFriction(0.9)
    wheel.GetCollisionModel().SetDefaultMaterial(mat_wheel)

    # Add a simple wheel visual shape (cylinder)
    cyl = chrono.ChCylinderShape()
    cyl.GetCylinderGeometry().p1 = chrono.ChVectorD(0, -wheel_width / 2, 0)
    cyl.GetCylinderGeometry().p2 = chrono.ChVectorD(0, wheel_width / 2, 0)
    cyl.GetCylinderGeometry().rad = wheel_radius
    wheel.AddVisualShape(cyl)

    system.Add(wheel)
    wheels.append(wheel)

# -----------------------------------------------------------------------------
# Connect wheels using revolute joints and motors (simulate steering for front wheels)
# -----------------------------------------------------------------------------
motors = []  # to hold steering motors

for i, wheel in enumerate(wheels):
    # Connect wheel to rover body via revolute joint along wheel spin axis (Y-axis)
    # Create a revolute joint that allows wheel rotation about the axle (Y axis)
    joint = chrono.ChLinkLockRevolute()
    joint.Initialize(wheel, rover, chrono.ChCoordsysD(wheel.GetPos(), chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.VECT_X)))
    system.AddLink(joint)

    # For front wheels (index 0,1) add steering capability: revolute joint about vertical axis (Z)
    if i in [0, 1]:
        # Create a steering revolute joint (about vertical axis) - to steer the wheel left/right
        steering_rot = chrono.ChBody()
        steering_rot.SetMass(5)
        steering_rot.SetInertiaXX(chrono.ChVectorD(0.01, 0.01, 0.01))
        steering_rot.SetPos(wheel.GetPos())
        steering_rot.SetCollide(False)
        system.Add(steering_rot)

        # Steering joint between rover and steering_rot (Z-axis rotation)
        steering_joint = chrono.ChLinkLockRevolute()
        steering_joint.Initialize(steering_rot, rover, chrono.ChCoordsysD(steering_rot.GetPos(), chrono.QIDENT))
        system.AddLink(steering_joint)

        # Wheel joint rotated so wheel spins relative to steering_rot
        wheel_joint = chrono.ChLinkLockRevolute()
        wheel_joint.Initialize(wheel, steering_rot, chrono.ChCoordsysD(wheel.GetPos(), chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.VECT_X)))
        system.AddLink(wheel_joint)

        # Create a motor to apply steering torque for the steering joint (rover <-> steering_rot)
        motor = chrono.ChLinkMotorRotationSpeed()
        motor.Set_sp_function(chrono.ChFunction_Const(0))
        motor.Initialize(steering_rot, rover, chrono.ChFrameD(steering_rot.GetPos(), chrono.QIDENT))
        system.AddLink(motor)
        motors.append(motor)

# -----------------------------------------------------------------------------
# Simple driver to update steering angle in real-time via motors
# -----------------------------------------------------------------------------
class SteeringDriver:
    def __init__(self, motors):
        self.motors = motors
        self.time = 0

    def update(self, step):
        self.time += step
        # Simple sinusoidal steering input oscillating +/- 0.4 radians (~ +/- 23 degrees)
        steering_angle = 0.4 * chrono.sin(0.5 * self.time)

        # Desired steering angular velocity to reach steering_angle
        # For simplicity, set motor speed to quickly drive steering joint angle to target.
        for motor in self.motors:
            # motor speed command proportional to difference between current angle and desired
            # But since we don't read joint angle easily here, just set a sinusoidal speed for demo
            motor.Set_sp(0.4 * chrono.cos(0.5 * self.time))


# -----------------------------------------------------------------------------
# Setup Irrlicht visualization
# -----------------------------------------------------------------------------
app = chronoirr.ChIrrApp(system, "Curiosity Rover Navigation on Terrain", chronoirr.dimension2du(1280, 720))

# Customize camera
app.AddTypicalCamera(chronoirr.vector3df(3, -6, 2))
app.SetCameraVertical(CameraVertical=True)
# Move camera a bit for better view
app.GetCameraManager().MoveCamera(chronoirr.vector3df(0, 0, 0),  # no move
                                  chronoirr.vector3df(0, 0, 0))   # no rotation

# Lighting
app.AddTypicalLights()
app.SetWindowSize(1280, 720)
app.SetTimestep(0.01)
app.AssetBindAll()
app.AssetUpdateAll()

# Enable shadows
app.SetShadowIntensity(0.5)
app.SetLightFollowCamera(True)
app.SetShadowUseIrrealtime(True)
app.SetShadowUseTexture(True)
app.SetShadowUseVSM(False)

# -----------------------------------------------------------------------------
# Simulation loop
# -----------------------------------------------------------------------------
driver = SteeringDriver(motors)
time_step = 0.01

while app.GetDevice().run():
    driver.update(time_step)
    app.BeginScene()
    app.DrawAll()
    app.DoStep()
    app.EndScene()