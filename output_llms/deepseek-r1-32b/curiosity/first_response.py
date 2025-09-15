importchrono
import numpy as np
import matplotlib.pyplot as plt

# Initialize the Chrono system
system = chrono.ChSystemNSC()
system.Set_Gravity(chrono.ChVectorD(0, -9.81, 0))

# Visualization settings
visualizer = chrono.ChVisualSystemIrrlicht()
visualizer.SetWindowTitle('Curiosity Rover Simulation')
visualizer.SetWindowSize(1024, 768)
visualizer.SetClearColor(chrono.ChColor(0.9, 0.9, 0.9))
visualizer.SetLightPosition(chrono.ChVectorD(0, 10, 10))
visualizer.SetLightIntensity(1)
visualizer.SetCameraPosition(chrono.ChVectorD(10, 10, 10))
visualizer.SetCameraRotation(chrono.ChVectorD(45, 45, 0))
system.SetVisualSystem(visualizer)

# Create ground
ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground.GetCollisionModel().AddBox(chrono.ChCollisionModel(), 100, 100, 1)
ground.GetVisualShape().SetTexture(chrono.GetChronoDataPath() + 'textures/ground.jpg')
system.Add(ground)

# Create rover
rover = chrono.ChVehicle()
rover.SetChassisMass(100)
rover.SetChassisInertia(chrono.ChMatrix33D(100))
rover.SetChassisDimensions(1, 0.5, 0.5)
rover.SetChassisPos(chrono.ChVectorD(0, 0.25, 0))

# Add wheels
wheel_mass = 10
wheel_inertia = chrono.ChMatrix33D(10)
wheel_radius = 0.2
wheel_width = 0.1

for i in range(4):
    wheel = chrono.ChWheel()
    wheel.SetMass(wheel_mass)
    wheel.SetInertia(wheel_inertia)
    wheel.SetRadius(wheel_radius)
    wheel.SetWidth(wheel_width)
    wheel.SetVisualShape(chrono.ChVisualShapeType.SPHERE)
    
    # Position wheels
    x_pos = [0.3, -0.3, 0.3, -0.3]
    z_pos = [0.3, 0.3, -0.3, -0.3]
    wheel.SetPos(chrono.ChVectorD(x_pos[i], 0, z_pos[i]))
    
    # Add revolute joint
    joint = chrono.ChRevoluteJoint()
    joint.Initialize(rover.GetChassis(), wheel, chrono.ChCoordsysD(wheel.GetPos(), chrono.ChQuaternionD(1, 0, 0, 0)))
    system.Add(joint)
    
    rover.AddWheel(wheel)

# Add driver
driver = chrono.ChDriver()
driver.SetSteering(0)
rover.SetDriver(driver)

# Add collision properties
rover.GetChassis().GetCollisionModel().AddBox(chrono.ChCollisionModel(), 0.5, 0.5, 0.5)
for wheel in rover.GetWheels():
    wheel.GetCollisionModel().AddCylinder(chrono.ChCollisionModel(), wheel_radius, wheel_width/2)

# Add texture to rover
rover.GetChassis().GetVisualShape().SetTexture('path_to_texture_file.jpg')  # Replace with actual texture path

# Add lighting and shadows
visualizer.AddLight(chrono.ChVectorD(0, 10, 10), 1, True)
visualizer.AddLight(chrono.ChVectorD(0, -10, 10), 1, True)
visualizer.SetAmbientLight(chrono.ChColor(0.5, 0.5, 0.5))

# Add sky background
visualizer.AddSky()

# Add NASA logo (replace with actual logo path)
# logo = chrono.ChVisualShape()
# logo.SetTexture('path_to_nasa_logo.png')
# logo.SetPos(chrono.ChVectorD(0, 0.5, 0))
# system.Add(logo)

# Simulation settings
system.SetMaxPenetration(0.001)
system.SetSolverType(chrono.ChSolverType.SOR)
system.SetTolerance(1e-6)
system.SetIterMax(100)

# Real-time simulation
simulation_time = 0
step_size = 0.001

# Store position data for plotting
positions = []
times = []

# Run simulation
while simulation_time < 10:
    # Update system
    system.DoStepDynamics(step_size)
    
    # Get rover position
    pos = rover.GetChassis().GetPos()
    positions.append(pos)
    times.append(simulation_time)
    
    # Update visualization
    visualizer.Render()
    
    # Handle user input
    if visualizer.GetKeyboard().IsKeyDown(chrono.KEY_ESCAPE):
        break
    if visualizer.GetKeyboard().IsKeyDown(chrono.KEY_LEFT):
        driver.SetSteering(-0.1)
    if visualizer.GetKeyboard().IsKeyDown(chrono.KEY_RIGHT):
        driver.SetSteering(0.1)
    if visualizer.GetKeyboard().IsKeyDown(chrono.KEY_SPACE):
        driver.SetThrottle(1)
    else:
        driver.SetThrottle(0)
    
    simulation_time += step_size

# Plot position data
plt.plot(times, [p.x for p in positions], label='X Position')
plt.plot(times, [p.y for p in positions], label='Y Position')
plt.plot(times, [p.z for p in positions], label='Z Position')
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.title('Rover Position Over Time')
plt.legend()
plt.show()

# Cleanup
chrono.ChDeleteAll()