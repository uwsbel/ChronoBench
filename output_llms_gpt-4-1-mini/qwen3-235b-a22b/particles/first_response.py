import pychrono as chrono
import pychrono.irrlicht as irrlicht
import random
import math

# Initialize the Chrono system
sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, 0, 0))  # Disable default gravity

# Initialize Irrlicht visualization
vis = irrlicht.ChVisualSystemIrrlicht()
sys.SetVisualSystem(vis)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Particle Gravitational Attraction')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 0, 5), chrono.ChVectorD(0, 0, 0))

def create_particle():
    # Random position within [-2, 2] in x, y, z
    pos = chrono.ChVectorD(
        random.uniform(-2, 2),
        random.uniform(-2, 2),
        random.uniform(-2, 2)
    )

    # Random velocity
    vel = chrono.ChVectorD(
        random.uniform(-1, 1),
        random.uniform(-1, 1),
        random.uniform(-1, 1)
    )

    # Random orientation
    axis = chrono.ChVectorD(
        random.uniform(-1, 1),
        random.uniform(-1, 1),
        random.uniform(-1, 1)
    ).GetNormalized()
    angle = random.uniform(0, 2 * math.pi)
    rot = chrono.ChQuaternionD()
    rot.Q_from_AngAxis(angle, axis)

    # Create the rigid body
    body = chrono.ChBody()
    body.SetPos(pos)
    body.SetPos_dt(vel)
    body.SetRot(rot)
    body.SetMass(1.0)  # Default mass
    body.SetInertiaXX(chrono.ChVectorD(1, 1, 1))  # Default inertia

    # Set material properties
    material = chrono.ChMaterialSurfaceNSC()
    material.SetFriction(0.2)
    material.SetRestitution(0.5)
    body.SetMaterialSurface(material)

    # Random shape: sphere or box
    shape_type = random.choice(['sphere', 'box'])
    if shape_type == 'sphere':
        radius = 0.1
        body.GetCollisionModel().AddSphere(radius)
        sphere = chrono.ChSphereShape()
        sphere.GetSphereGeometry().rad = radius
        body.AddAsset(sphere)
    else:
        box_size = chrono.ChVectorD(0.2, 0.2, 0.2)
        body.GetCollisionModel().AddBox(box_size.x, box_size.y, box_size.z)
        box = chrono.ChBoxShape()
        box.GetBoxGeometry().Size = box_size
        body.AddAsset(box)

    # Finalize the collision model
    body.GetCollisionModel().Build()

    # Add the body to the system
    sys.Add(body)

    return body

# Create initial particles
for _ in range(5):
    create_particle()

# Simulation loop parameters
timestep = 0.01
emission_interval = 1.0  # Emit a new particle every 1 second
last_emission_time = 0.0

# Simulation loop
while vis.Run():
    current_time = sys.GetChTime()

    # Emit a new particle at intervals
    if current_time - last_emission_time >= emission_interval:
        create_particle()
        last_emission_time = current_time

    # Compute gravitational forces between particles
    bodies = list(sys.Get_bodylist())
    for body_a in bodies:
        force_on_a = chrono.ChVectorD(0, 0, 0)
        for body_b in bodies:
            if body_a == body_b:
                continue
            r_vec = body_b.GetPos() - body_a.GetPos()
            r_mag = r_vec.Length()
            if r_mag < 0.01:  # Avoid division by zero
                continue
            G = 1.0
            m_a = body_a.GetMass()
            m_b = body_b.GetMass()
            force_mag = G * m_a * m_b / (r_mag ** 2)
            force_dir = r_vec.GetNormalized()
            force_on_a += force_dir * force_mag
        body_a.SetF(force_on_a)  # Apply the calculated force

    # Advance the simulation
    sys.DoStepDynamics(timestep)

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()