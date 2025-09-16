import pychrono as chrono
import pychrono.irrlicht as chronoirr
import random
import math

# Initialize PyChrono system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  # Disable default gravity

# Create Irrlicht application for visualization
application = chronoirr.ChIrrApp(system, 'Gravitational Particles', chronoirr.dimension2du(1024, 768))
application.AddTypicalLogo()
application.AddTypicalCamera(chronoirr.vector3df(0, 0, 10))
application.AddTypicalLights()
application.DrawAll()

# List to hold particle bodies
particles = []

def create_particle(system, particles):
    """Create a particle with random shape, position, velocity, and orientation."""
    body = chrono.ChBody()
    body.SetCollide(False)  # Disable collisions for simplicity

    # Random shape selection
    shape_type = random.choice(['sphere', 'box', 'cylinder'])

    if shape_type == 'sphere':
        radius = random.uniform(0.1, 0.5)
        body.SetMass(1.0)
        inertia = (2/5.0) * radius**2  # Inertia for sphere (mass=1.0)
        body.SetInertiaXX(chrono.ChVectorD(inertia, inertia, inertia))
        vis_shape = chrono.ChSphereShape()
        vis_shape.GetSphereGeometry().rad = radius
        body.GetAssets().push_back(vis_shape)

    elif shape_type == 'box':
        size = chrono.ChVectorD(random.uniform(0.2, 0.5), random.uniform(0.2, 0.5), random.uniform(0.2, 0.5))
        body.SetMass(1.0)
        # Inertia for box (mass=1.0)
        inertia_x = (1.0 / 6.0) * size.x**2
        inertia_y = (1.0 / 6.0) * size.y**2
        inertia_z = (1.0 / 6.0) * size.z**2
        body.SetInertiaXX(chrono.ChVectorD(inertia_x, inertia_y, inertia_z))
        vis_shape = chrono.ChBoxShape()
        vis_shape.GetBoxGeometry().Size = size
        body.GetAssets().push_back(vis_shape)

    elif shape_type == 'cylinder':
        radius = random.uniform(0.1, 0.3)
        height = random.uniform(0.2, 0.5)
        body.SetMass(1.0)
        # Inertia for cylinder (mass=1.0)
        inertia_y = 0.5 * radius**2  # Around Y-axis
        inertia_x = (1.0 / 12.0) * (3 * radius**2 + height**2)  # Around X/Z
        body.SetInertiaXX(chrono.ChVectorD(inertia_x, inertia_y, inertia_x))
        vis_shape = chrono.ChCylinderShape()
        vis_shape.GetCylinderGeometry().rad = radius
        vis_shape.GetCylinderGeometry().p1 = chrono.ChVectorD(0, -height/2, 0)
        vis_shape.GetCylinderGeometry().p2 = chrono.ChVectorD(0, height/2, 0)
        body.GetAssets().push_back(vis_shape)

    # Random position
    pos = chrono.ChVectorD(
        random.uniform(-5, 5),
        random.uniform(-5, 5),
        random.uniform(-5, 5)
    )
    body.SetPos(pos)

    # Random orientation
    angle = random.uniform(0, 2 * math.pi)
    axis = chrono.ChVectorD(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1)).GetNormalized()
    quat = chrono.ChQuaternionD()
    quat.Q_from_AngAxis(angle, axis)
    body.SetRot(quat)

    # Random velocity
    vel = chrono.ChVectorD(
        random.uniform(-1, 1),
        random.uniform(-1, 1),
        random.uniform(-1, 1)
    )
    body.SetPos_dt(vel)

    # Add to system and particle list
    system.Add(body)
    particles.append(body)
    return body

# Create multiple particles
for _ in range(10):
    create_particle(system, particles)

# Simulation loop
while application.GetDevice().run():
    # Apply gravitational forces between particles
    G = 1e-3  # Gravitational constant (tuned for simulation)
    for body in particles:
        total_force = chrono.ChVectorD(0, 0, 0)
        for other in particles:
            if other == body:
                continue
            r = other.GetPos() - body.GetPos()
            distance = r.Length()
            if distance < 1e-3:  # Avoid division by zero
                continue
            force_magnitude = G / (distance**2)
            force_vector = r * (force_magnitude / distance)  # Normalize and scale
            total_force += force_vector
        body.SetForce(total_force)

    # Step the simulation
    system.DoStepDynamics(1e-3)

    # Update Irrlicht visualization
    application.BeginScene()
    application.DrawAll()
    application.EndScene()