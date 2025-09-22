import pychrono.core as chrono
import pychrono.irrlicht as irr
import math

# Initialize the Chrono system
system = chrono.ChSystemNSC()

# Create and add the ground (rigid terrain)
ground = chrono.ChBodyEasyBox(20, 1, 20, 1000, True, True)  
# Size: 20x1x20 meters, density: 1000 kg/m^3
ground.SetPosition(chrono.ChVectorD(0, -0.5, 0))
ground.SetBodyFixed(True)

# Assign collision shape and material
ground.GetCollisionModel().ClearModel()
ground.GetCollisionModel().AddBox(20, 1, 20)
ground.GetCollisionModel().BuildModel()
ground.SetCollide(True)

system.Add(ground)

# Create the rover body (simplified)
rover = chrono.ChBodyEasyBox(2, 1, 3, 800, True, True)  # size: 2x1x3 meters
rover.SetPosition(chrono.ChVectorD(0, 0.5, 0))
# Optional: add texture or color
rover.GetVisualShape(0).SetTextureFilename("textures/rover_texture.jpg")  # Placeholder texture path

system.Add(rover)

# Setup driver controls
driver = chrono.ChModHSIC(this=system)
# For simplicity, using keys for steering and throttle
# Note: Implementing a full driver requires event handling; here is a basic placeholder
def get_user_input():
    # Placeholder for real-time input; replace with event handling as needed
    steering = 0.0  # Range: [-1, 1]
    throttle = 0.0  # Range: [0, 1]
    return steering, throttle

# Create wheels (simplified as spheres or cylinders)
wheel_radius = 0.4
wheel_width = 0.2
wheel_positions = [
    chrono.ChVectorD(1, 0.4, 1.2),   # Front right
    chrono.ChVectorD(-1, 0.4, 1.2),  # Front left
    chrono.ChVectorD(1, 0.4, -1.2),  # Rear right
    chrono.ChVectorD(-1, 0.4, -1.2)  # Rear left
]
wheels = []

for pos in wheel_positions:
    wheel = chrono.ChBodyEasyCylinder(wheel_radius, wheel_width, 20, True, True)
    wheel.SetPos(rover.GetPos() + pos)
    wheel.SetRot(chrono.Q_from_AngX(math.pi/2))  # Align wheel along X
    # Assign visual texture if available
    # wheel.GetVisualShape(0).SetTextureFilename("textures/wheel_texture.jpg")
    system.Add(wheel)
    wheels.append(wheel)
    # Create a motor or joint to connect the wheel to rover body
    rev_motor = chrono.ChLinkMotorRotation()
    rev_motor.SetServo(chrono.ChSharedPtr(chrono.ChFunction_Ramp()), True)
    rev_motor.Initialize(wheel, rover, chrono.ChFrameD(wheel.GetPos(), wheel.GetRot()))
    system.Add(rev_motor)

# Gravity
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Visualization setup with Irrlicht
application = irr.ChIrrApp(system, "Curiosity Rover Navigation Simulation", irr.dimension2du(1024, 768))
application.AddTypicalSky()
application.AddTypicalSun()

# Lighting and shadows
application.GetActiveCamera().SetChaseCamera(chrono.ChVectorD(0, 2, -5),  # Camera position
                                             chrono.ChVectorD(0, 0, 0),   # Look-at point
                                             0.5)                        # Chase distance
application.SetFeelUnuo(true)
application.AssetBindAll()
application.AssetUpdateAll()

# Configure visual style
application.GetVideoDriver().setTextureFiltering(irr.irr::video::ECT_BILINEAR)
application.SetSkyBox()
application.ShowLogo()

# Camera settings
cam_pos = chrono.ChVectorD(0, 3, -10)
application.GetCamera()->SetPosition(cam_pos)
application.GetCamera()->SetTarget(chrono.ChVectorD(0, 0, 0))
application.GetCamera()->SetFocalLength(45)

# Add user interaction and animation loop
while application.GetDevice().run():
    # Get user input for steering and throttle
    steering_input, throttle_input = get_user_input()

    # Apply steering to the wheels (simplified)
    # For more realistic behavior, connect steering angles to the wheels' orientations
    # and control motor velocities based on throttle.
    # Example:
    max_steering_angle = math.radians(30)  # Max steering angle
    steering_angle = max_steering_angle * steering_input

    # For each wheel, set steering and motor speed
    # Note: Implement actual steering joints if using a more detailed model.
    # Here, assume the front wheels are steered
    front_left_wheel = wheels[1]
    front_right_wheel = wheels[0]

    # Placeholder: rotate wheels based on throttle
    wheel_speed = 10 * throttle_input  # radians/sec

    for wheel in wheels:
        # Set wheel rotation speed
        # For actual motor control, connect to joint motors
        # As placeholder:
        pass

    # Advance the simulation
    application.BeginScene()
    application.DrawAll()
    application.DoStep()

    # Optional: add controls, logging, or other behaviors

application.GetDevice().close()