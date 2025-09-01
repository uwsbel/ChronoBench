import pychrono as pychrono
import pychrono visualize as pyvis
import pychrono.components as pc

# Initialize PyChrono environment and components
pychrono.init_pychrono()
pychrono.set_physics_backend("euler")
pychrono.set_visualization_backend("irrlicht")
pychrono.set_default_gravity(pc.GRAVITY_Z, 9.81)

# Create simulation environment
env = pychrono.Environment()
env.set_ground_body(pc.Body(pc.BodyType.STATIC, "ground", pychrono.Vec3(0, 0, 0), pychrono.Vec3(0, 0, 0), 1e9, pc.Inertia(pc.INF, pc.INF, pc.INF)))

# Create a joint to connect the pendulum to the ground
joint = pc.RevoluteJoint(env, "joint", pc.JointType.REVOLUTE, pc.Axis3d(0, 0, 1), pychrono.Vec3(0, 0, 0), pychrono.Vec3(0, 0, 0), 0, 0, 0)

# Create pendulum mass
pendulum_mass = pc.Body(pc.BodyType.DYNAMIC, "pendulum", pychrono.Vec3(0, 5, 0), pychrono.Vec3(0, 0, 0), 0.1, pc.Inertia(0.1 * 0.1, 0, 0))  # I = m * r^2, r = 0.1m, I=0.01 kg m^2

# Set initial angle and velocity
pendulum_mass.set_initial_angle(0)
pendulum_mass.set_initial_velocity(pychrono.Vec3(0, 0, 0))

# Add pendulum to the joint
joint.add_attached_body(pendulum_mass)

# Set up gravity
env.set_gravity(pc.GRAVITY_Z, 9.81)

# Set up visualization
renderer = pyvis.IrrlichtRenderer.create_window(800, 600, "Pendulum Simulation")
renderer.set_background_color(pychrono.Visualize.Color(0, 0, 0))
renderer.set_wireframe_mode(True)
renderer.set_camera_position(pychrono.Vec3(0, 10, 20))

# Function to log data
def log_data():
    global data_file
    data_file.write(f"{env.get_time()/1000} , {pendulum_mass.get_position().x} , {pendulum_mass.get_position().y} , {pendulum_mass.get_velocity().x} \n")

# Function to create simulation loop
def simulate():
    global data_file
    data_file = open("pendulum_data.txt", "w")
    timer = pychrono.Timer(0.1)
    while True:
        timer.update()
        env.step()
        renderer.update()
        log_data()
        if pychrono.Keyboard.is_pressed(pychrono.Keyboard.EVENT_KEY 'q'):
            break
    data_file.close()

# Run the simulation
simulate()