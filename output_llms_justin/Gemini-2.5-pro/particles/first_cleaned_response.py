import pychrono as chrono
import pychrono.irrlicht as chronoirr
import random
import math




TIME_STEP = 0.005
SIM_DURATION = 20.0  
OUTPUT_FPS = 60


G_CONSTANT = 50.0  
MIN_DIST_SQ_FOR_FORCE = 0.1**2 


MAX_PARTICLES = 50
EMIT_INTERVAL_STEPS = 20  
EMISSION_BOX_HALF_DIMS = chrono.ChVectorD(2, 2, 2) 
PARTICLE_DENSITY = 1000  


MIN_PARTICLE_SIZE = 0.1 
MAX_PARTICLE_SIZE = 0.3
MAX_INITIAL_LINEAR_VEL = 0.5 
MAX_INITIAL_ANGULAR_VEL = 0.5 




def create_random_particle(system, particle_list, irr_app):
    
    global particle_id_counter
    particle_id_counter +=1

    
    contact_material = chrono.ChMaterialSurfaceNSC()
    contact_material.SetFriction(0.4)
    contact_material.SetRestitution(0.1)

    
    pos_x = random.uniform(-EMISSION_BOX_HALF_DIMS.x, EMISSION_BOX_HALF_DIMS.x)
    pos_y = random.uniform(-EMISSION_BOX_HALF_DIMS.y, EMISSION_BOX_HALF_DIMS.y)
    pos_z = random.uniform(-EMISSION_BOX_HALF_DIMS.z, EMISSION_BOX_HALF_DIMS.z)
    position = chrono.ChVectorD(pos_x, pos_y, pos_z)

    
    angle_x = random.uniform(0, 2 * math.pi)
    angle_y = random.uniform(0, 2 * math.pi)
    angle_z = random.uniform(0, 2 * math.pi)
    rotation = chrono.Q_from_Euler123(chrono.ChVectorD(angle_x, angle_y, angle_z))

    
    vel_x = random.uniform(-MAX_INITIAL_LINEAR_VEL, MAX_INITIAL_LINEAR_VEL)
    vel_y = random.uniform(-MAX_INITIAL_LINEAR_VEL, MAX_INITIAL_LINEAR_VEL)
    vel_z = random.uniform(-MAX_INITIAL_LINEAR_VEL, MAX_INITIAL_LINEAR_VEL)
    linear_velocity = chrono.ChVectorD(vel_x, vel_y, vel_z)

    
    ang_vel_x = random.uniform(-MAX_INITIAL_ANGULAR_VEL, MAX_INITIAL_ANGULAR_VEL)
    ang_vel_y = random.uniform(-MAX_INITIAL_ANGULAR_VEL, MAX_INITIAL_ANGULAR_VEL)
    ang_vel_z = random.uniform(-MAX_INITIAL_ANGULAR_VEL, MAX_INITIAL_ANGULAR_VEL)
    angular_velocity = chrono.ChVectorD(ang_vel_x, ang_vel_y, ang_vel_z)

    
    particle = chrono.ChBody()
    particle.SetPos(position)
    particle.SetRot(rotation)
    particle.SetPos_dt(linear_velocity)
    particle.SetWvel_par(angular_velocity) 
    particle.SetBodyFixed(False)
    particle.SetCollide(True)
    particle.SetId(particle_id_counter) 

    
    shape_type = random.choice(['sphere', 'box', 'cylinder'])
    mass = 0
    inertia = chrono.ChVectorD(0,0,0)

    particle.GetCollisionModel().ClearModel()

    if shape_type == 'sphere':
        radius = random.uniform(MIN_PARTICLE_SIZE, MAX_PARTICLE_SIZE)
        particle.GetCollisionModel().AddSphere(contact_material, radius)
        
        sphere_shape = chrono.ChSphereShape()
        sphere_shape.GetSphereGeometry().rad = radius
        particle.AddVisualShape(sphere_shape)
        mass = PARTICLE_DENSITY * (4/3) * math.pi * radius**3
        inertia_scalar = (2/5) * mass * radius**2
        inertia = chrono.ChVectorD(inertia_scalar, inertia_scalar, inertia_scalar)

    elif shape_type == 'box':
        hx = random.uniform(MIN_PARTICLE_SIZE, MAX_PARTICLE_SIZE)
        hy = random.uniform(MIN_PARTICLE_SIZE, MAX_PARTICLE_SIZE)
        hz = random.uniform(MIN_PARTICLE_SIZE, MAX_PARTICLE_SIZE)
        particle.GetCollisionModel().AddBox(contact_material, hx, hy, hz)
        
        box_shape = chrono.ChBoxShape()
        box_shape.GetBoxGeometry().Size = chrono.ChVectorD(hx, hy, hz)
        particle.AddVisualShape(box_shape)
        mass = PARTICLE_DENSITY * (2*hx) * (2*hy) * (2*hz)
        inertia.x = (1/12) * mass * ( (2*hy)**2 + (2*hz)**2 )
        inertia.y = (1/12) * mass * ( (2*hx)**2 + (2*hz)**2 )
        inertia.z = (1/12) * mass * ( (2*hx)**2 + (2*hy)**2 )

    elif shape_type == 'cylinder': 
        radius = random.uniform(MIN_PARTICLE_SIZE, MAX_PARTICLE_SIZE)
        height_half = random.uniform(MIN_PARTICLE_SIZE, MAX_PARTICLE_SIZE)
        
        
        
        rot_cyl = chrono.Q_from_AngX(math.pi/2)
        particle.GetCollisionModel().AddCylinder(contact_material,
                                                 radius, radius, height_half,
                                                 chrono.ChVectorD(0,0,0), rot_cyl)
        
        cyl_shape = chrono.ChCylinderShape()
        cyl_shape.GetCylinderGeometry().rad = radius
        cyl_shape.GetCylinderGeometry().p1 = chrono.ChVectorD(0, -height_half, 0)
        cyl_shape.GetCylinderGeometry().p2 = chrono.ChVectorD(0,  height_half, 0)
        particle.AddVisualShape(cyl_shape) 
        mass = PARTICLE_DENSITY * math.pi * radius**2 * (2*height_half)
        inertia.x = (1/12) * mass * (3*radius**2 + (2*height_half)**2)
        inertia.y = (1/2) * mass * radius**2 
        inertia.z = (1/12) * mass * (3*radius**2 + (2*height_half)**2)

    particle.SetMass(mass)
    particle.SetInertiaXX(inertia)

    particle.GetCollisionModel().BuildModel() 

    system.Add(particle)
    particle_list.append(particle)

    
    if irr_app:
        irr_app.AssetBind(particle)
        irr_app.AssetUpdate(particle)

    print(f"Emitted particle {particle.GetId()} of type {shape_type} with mass {mass:.2f}kg")
    return particle








system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVectorD(0, 0, 0)) 


system.SetSolverType(chrono.ChSolver.Type.SOR) 
system.SetSolverMaxIterations(50)
system.SetTimestepperType(chrono.ChTimestepper.Type.EULER_IMPLICIT_LINEARIZED) 


particles = []
particle_id_counter = 0 


application = chronoirr.ChIrrApp(system, "Particle Gravity Simulation", chronoirr.dimension2du(1024, 768))
application.AddTypicalLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chronoirr.vector3df(4, 4, -6), chronoirr.vector3df(0, 1, 0)) 
application.SetContactsDrawMode(chronoirr.IrrContactsDrawMode_CONTACT_DISTANCES) 



application.AssetBindAll()
application.AssetUpdateAll()


current_time = 0
step_number = 0
output_frame = 0

application.SetTimestep(TIME_STEP)

print("Starting simulation...")

while application.GetDevice().run():
    application.BeginScene(True, True, chronoirr.SColor(255, 140, 160, 190))
    application.DrawAll()

    
    if len(particles) < MAX_PARTICLES and step_number % EMIT_INTERVAL_STEPS == 0:
        create_random_particle(system, particles, application)

    
    
    for p_i in particles:
        p_i.ClearForce()
        p_i.ClearTorque() 

    
    
    num_p = len(particles)
    for i in range(num_p):
        body_A = particles[i]
        mass_A = body_A.GetMass()
        pos_A = body_A.GetPos()

        for j in range(i + 1, num_p): 
            body_B = particles[j]
            mass_B = body_B.GetMass()
            pos_B = body_B.GetPos()

            
            dir_vector = pos_B - pos_A
            dist_sq = dir_vector.Length2()

            if dist_sq < MIN_DIST_SQ_FOR_FORCE: 
                dist_sq = MIN_DIST_SQ_FOR_FORCE

            dist = math.sqrt(dist_sq)
            force_magnitude = (G_CONSTANT * mass_A * mass_B) / dist_sq

            
            force_vec_on_A = (dir_vector / dist) * force_magnitude
            
            force_vec_on_B = -force_vec_on_A

            
            body_A.AccumulateForce(force_vec_on_A, pos_A, False) 
            body_B.AccumulateForce(force_vec_on_B, pos_B, False)

    
    system.DoStepDynamics(TIME_STEP)
    current_time += TIME_STEP
    step_number += 1

    
    if step_number % int(1.0 / TIME_STEP) == 0:
        print(f"Time: {current_time:.2f} s, Particles: {len(particles)}")

    application.EndScene()

    if current_time > SIM_DURATION:
        break

print("Simulation finished.")
application.GetDevice().closeDevice()