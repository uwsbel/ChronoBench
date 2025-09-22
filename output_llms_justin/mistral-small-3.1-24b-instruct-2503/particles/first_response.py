import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import random

# Initialize the PyChrono environment
chrono.SetChronoDataPath('path/to/chrono/data')  # Set the path to the Chrono data directory

# Create the simulation system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravity

# Create an Irrlicht visualization system
visualization = chronoirr.ChVisualSystemIrrlicht()
visualization.AttachSystem(system)
visualization.SetWindowSize(1024, 768)
visualization.SetWindowTitle('Gravitational Attraction Simulation')
visualization.Initialize()
visualization.AddTypicalSky()
visualization.AddTypicalLogo()
visualization.AddLightWithShadow(chrono.ChCoordsysD(chrono.ChVectorD(2, 2, 5)), chrono.ChVectorD(1, 1, -1), 50, 50, 10, 50, 50)

# Function to create a random particle
def create_random_particle(system):
    mass = random.uniform(0.1, 1.0)
    radius = random.uniform(0.05, 0.1)

    # Create a sphere shape
    sphere_shape = chrono.ChSphereShape()
    sphere_shape.GetSphereGeometry().rad = radius

    # Create a body and attach the shape
    body = chrono.ChBodyEasySphere(mass, radius, material=chrono.ChMaterialSurfaceNSC())
    body.SetPos(chrono.ChVectorD(random.uniform(-1, 1), random.uniform(0, 2), random.uniform(-1, 1)))
    body.SetRot(chrono.ChQuaternionD(1, 0, 0, random.uniform(0, 2 * 3.141592653589793)))
    body.SetVel(chrono.ChVectorD(random.uniform(-0.5, 0.5), random.uniform(-0.5, 0.5), random.uniform(-0.5, 0.5)))
    body.SetAngVel(chrono.ChVectorD(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1)))

    # Add the body to the system
    system.Add(body)

    # Add visualization shape
    visualization.Add(body, sphere_shape)

# Create a particle emitter
def emit_particles(system, num_particles):
    for _ in range(num_particles):
        create_random_particle(system)

# Emit 50 particles
emit_particles(system, 50)

# Define gravitational force function
def gravitational_force(body1, body2, G=6.67430e-11):
    r = body2.GetPos() - body1.GetPos()
    distance = r.Length()
    if distance == 0:
        return chrono.ChVectorD(0, 0, 0)

    force_magnitude = G * (body1.GetMass() * body2.GetMass()) / (distance ** 2)
    force_direction = r.Normalize()
    force = force_direction * force_magnitude
    return force

# Apply gravitational forces
def apply_gravitational_forces(system):
    bodies = system.Get_bodylist()
    for i in range(len(bodies)):
        for j in range(i + 1, len(bodies)):
            body1 = bodies[i]
            body2 = bodies[j]
            force = gravitational_force(body1, body2)
            body1.AddForce(force)
            body2.AddForce(-force)

# Simulation loop
def run_simulation(system, visualization):
    while visualization.Run():
        system.DoStepDynamics(0.01)  # Step size of 0.01 seconds
        apply_gravitational_forces(system)
        visualization.Render()

# Run the simulation
run_simulation(system, visualization)