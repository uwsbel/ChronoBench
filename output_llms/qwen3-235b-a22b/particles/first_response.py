import pychrono as chrono
import pychrono.irrlicht as irr
import random
import math

# Initialize PyChrono system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  # Disable default gravity

# Initialize Irrlicht visualization
application = irr.ChIrrApp(system, 'Gravitational Particle Attraction', irr.dimension2du(1024, 768))
application.AddLogo()
application.AddSkyBox()
application.AddCamera(chrono.ChVectorD(0, 0, -40))
application.AddTypicalLights()
application.AssetBindAll()
application.AssetUpdateAll()

# Particle emitter settings
emitter_interval = 0.5  # seconds between emissions
last_emitted_time = 0.0
G = 0.1  # Gravitational constant (adjustable)

# Simulation loop
while application.GetDevice().run():
    # Emit new particle if enough time has passed
    current_time = system.GetChTime()
    if current_time - last_emitted_time > emitter_interval:
        # Create a new particle
        mass = random.uniform(0.5, 2.0)
        pos = chrono.ChVectorD(
            random.uniform(-10, 10),
            random.uniform(-10, 10),
            random.uniform(-10, 10)
        )
        vel = chrono.ChVectorD(
            random.uniform(-1, 1),
            random.uniform(-1, 1),
            random.uniform(-1, 1)
        )
        # Random orientation
        angle = random.uniform(0, 2 * math.pi)
        axis = chrono.ChVectorD(0, 1, 0)
        rot = chrono.ChQuaternionD()
        rot.Q_from_AngAxis(angle, axis)
        
        # Create body
        body = chrono.ChBody()
        body.SetMass(mass)
        body.SetPos(pos)
        body.SetPos_dt(vel)
        body.SetRot(rot)
        body.SetBodyFixed(False)
        body.SetCollide(True)
        
        # Random shape: sphere or box
        shape = random.choice(['sphere', 'box'])
        if shape == 'sphere':
            radius = random.uniform(0.2, 0.5)
            # Collision shape
            body.GetCollisionModel().ClearModel()
            sphere_shape = chrono.ChCollisionShape(chrono.ChCollisionShape.eSphere, radius)
            body.GetCollisionModel().AddShape(sphere_shape, chrono.ChFrameD())
            body.GetCollisionModel().BuildModel()
            # Visualization shape
            vis_shape = chrono.ChSphereShape(radius)
            body.AddAsset(vis_shape)
            # Inertia
            inertia = (2.0/5.0) * mass * radius * radius
            body.SetInertiaXX(chrono.ChVectorD(inertia, inertia, inertia))
        else:
            size = chrono.ChVectorD(
                random.uniform(0.5, 1.0),
                random.uniform(0.5, 1.0),
                random.uniform(0.5, 1.0)
            )
            # Collision shape
            body.GetCollisionModel().ClearModel()
            box_shape = chrono.ChCollisionShape(chrono.ChCollisionShape.eBox, size.x, size.y, size.z)
            body.GetCollisionModel().AddShape(box_shape, chrono.ChFrameD())
            body.GetCollisionModel().BuildModel()
            # Visualization shape
            vis_shape = chrono.ChBoxShape(size)
            body.AddAsset(vis_shape)
            # Inertia
            ixx = (mass / 12.0) * (size.y**2 + size.z**2)
            iyy = (mass / 12.0) * (size.x**2 + size.z**2)
            izz = (mass / 12.0) * (size.x**2 + size.y**2)
            body.SetInertiaXX(chrono.ChVectorD(ixx, iyy, izz))
        
        # Random color
        color = chrono.ChColorAsset()
        color.SetColor(chrono.ChColor(random.random(), random.random(), random.random()))
        body.AddAsset(color)
        
        # Add body to system
        system.AddBody(body)
        
        last_emitted_time = current_time
    
    # Apply gravitational forces
    bodies = system.Get_bodylist()
    num_bodies = len(bodies)
    for i in range(num_bodies):
        body_i = bodies[i]
        for j in range(i+1, num_bodies):
            body_j = bodies[j]
            pos_i = body_i.GetPos()
            pos_j = body_j.GetPos()
            r = pos_j - pos_i
            distance_sq = r.x**2 + r.y**2 + r.z**2
            distance = math.sqrt(distance_sq)
            if distance < 1e-3:
                continue  # Avoid division by zero or large forces
            force_mag = G * body_i.GetMass() * body_j.GetMass() / distance_sq
            force_dir = r * (1.0 / distance)
            force_i = force_dir * force_mag
            force_j = -force_dir * force_mag
            
            # Apply forces to each body
            body_i.Accumulate_force(force_i, chrono.ChVectorD(0,0,0), False)  # Force at COM
            body_j.Accumulate_force(force_j, chrono.ChVectorD(0,0,0), False)
    
    # Advance simulation
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    system.DoStepDynamics(0.01)