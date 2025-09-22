# Initialize PyChrono
import PyChrono as pc

# Create simulation
sim = pc.Simulation(pc.ODESolver())
sim.set_time_step(0.001)  # 1ms time step

# Create ground
ground_body = pc.Body(pc.Vec3(0, 0, 0), pc.Vec3(0, 0, 0))
ground_body.set_mass(0)  # Ground is static
ground_link = pc.ChLinkTSDA(sim, ground_body)

# Create mass and spring
mass_body = pc.Body(pc.Vec3(0, 1, 0))  # Position at (0,1,0)
spring_link = pc.ChLinkTSDA(sim, mass_body, ground_body)

# Set spring parameters
spring_constant = 1.0 / (2 * pc.PI) ** 2  # Natural frequency 2π Hz
damping_ratio = 0.5  # Damping ratio
spring_link.set_stiffness(spring_constant)
spring_link.set_damping(damping_ratio * spring_constant)

# Set up Irrlicht visualization
scene = pc.IrrlichtScene()
camera = scene.create_camera(pc.Vec3(5, 5, 5), pc.Vec3(0, 0, 0), pc.Vec3(-1, -1, -1), 45, 16/9, 0.1)
light = scene.add_light(pc.Vec3(10, 10, 10), pc.Vec3(0, 0, 0), pc.Color(255, 255, 255))

# Add visual shapes
mass_shape = scene.add_sphere(mass_body, radius=0.5, color=pc.Color(0, 0.5, 0.5))
spring_shape = scene.add_cylinder(spring_link, radius=0.1, length=1.0, color=pc.Color(0, 1, 0))

# Set camera position and lighting
scene.set_camera_position(camera.get_position())
scene.set_light_position(light.get_position())

# Finalize simulation setup
spring_link.set_ground(ground_body)
sim.add_body(mass_body)
sim.add_link(spring_link)

# Run simulation
sim.run(1000)  # Let system reach equilibrium