import sys
import math
import random

import pychrono as chrono
import pychrono.irrlicht as irr






G_CONSTANT = 0.5            


NUM_PARTICLES = 60


MIN_SIZE = 0.2
MAX_SIZE = 0.8


DENSITY = 1000.0


EMIT_RADIUS = 12.0


DT = 0.01


EMIT_EVERY_STEPS = 500
NEW_PARTICLES_PER_EMIT = 10





def random_unit_vector():
    
    u = random.uniform(-1.0, 1.0)
    theta = random.uniform(0.0, 2.0 * math.pi)
    r = math.sqrt(1.0 - u * u)
    return chrono.ChVectorD(r * math.cos(theta),
                           r * math.sin(theta),
                           u)


def compute_volume_and_inertia(shape_type, params):
    
    if shape_type == "sphere":
        r = params["radius"]
        vol = (4.0 / 3.0) * math.pi * r**3
        I = 0.4 * DENSITY * vol * r**2          
        return vol, I, I, I

    elif shape_type == "box":
        sx, sy, sz = params["sx"], params["sy"], params["sz"]
        vol = sx * sy * sz
        Ixx = DENSITY * vol / 12.0 * (sy**2 + sz**2)
        Iyy = DENSITY * vol / 12.0 * (sx**2 + sz**2)
        Izz = DENSITY * vol / 12.0 * (sx**2 + sy**2)
        return vol, Ixx, Iyy, Izz

    elif shape_type == "cylinder":
        r = params["radius"]
        h = params["height"]
        vol = math.pi * r**2 * h
        Ixx = DENSITY * vol / 12.0 * (3.0 * r**2 + h**2)
        Iyy = Ixx
        Izz = 0.5 * DENSITY * vol * r**2
        return vol, Ixx, Iyy, Izz

    elif shape_type == "ellipsoid":
        a, b, c = params["a"], params["b"], params["c"]
        vol = (4.0 / 3.0) * math.pi * a * b * c
        Ixx = 0.2 * DENSITY * vol * (b**2 + c**2)   
        Iyy = 0.2 * DENSITY * vol * (a**2 + c**2)
        Izz = 0.2 * DENSITY * vol * (a**2 + b**2)
        return vol, Ixx, Iyy, Izz

    else:
        raise ValueError("Unknown shape type: " + shape_type)


def create_random_particle(system):
    
    shape_type = random.choice(["sphere", "box", "cylinder", "ellipsoid"])
    body = chrono.ChBody()

    
    
    
    if shape_type == "sphere":
        radius = random.uniform(MIN_SIZE, MAX_SIZE)
        body.GetCollisionModel().SetSafeTolerance(1e-3)
        body.GetCollisionModel().AddSphere(radius)
        params = {"radius": radius}

    elif shape_type == "box":
        sx = random.uniform(MIN_SIZE, MAX_SIZE)
        sy = random.uniform(MIN_SIZE, MAX_SIZE)
        sz = random.uniform(MIN_SIZE, MAX_SIZE)
        body.GetCollisionModel().SetSafeTolerance(1e-3)
        
        body.GetCollisionModel().AddBox(sx / 2.0, sy / 2.0, sz / 2.0)
        params = {"sx": sx, "sy": sy, "sz": sz}

    elif shape_type == "cylinder":
        radius = random.uniform(MIN_SIZE, MAX_SIZE / 1.5)
        height = random.uniform(MIN_SIZE * 2.0, MAX_SIZE * 2.0)
        body.GetCollisionModel().SetSafeTolerance(1e-3)
        body.GetCollisionModel().AddCylinder(radius, height / 2.0)
        params = {"radius": radius, "height": height}

    elif shape_type == "ellipsoid":
        a = random.uniform(MIN_SIZE, MAX_SIZE)
        b = random.uniform(MIN_SIZE, MAX_SIZE)
        c = random.uniform(MIN_SIZE, MAX_SIZE)
        body.GetCollisionModel().SetSafeTolerance(1e-3)
        body.GetCollisionModel().AddEllipsoid(a, b, c)
        params = {"a": a, "b": b, "c": c}

    
    
    
    volume, Ixx, Iyy, Izz = compute_volume_and_inertia(shape_type, params)
    mass = DENSITY * volume
    body.SetMass(mass)
    body.SetInertiaXX(chrono.ChVectorD(Ixx, Iyy, Izz))

    
    
    
    pos = chrono.ChVectorD(random.uniform(-EMIT_RADIUS, EMIT_RADIUS),
                           random.uniform(-EMIT_RADIUS, EMIT_RADIUS),
                           random.uniform(-EMIT_RADIUS, EMIT_RADIUS))
    body.SetPos(pos)

    
    axis = random_unit_vector()
    angle = random.uniform(0.0, 2.0 * math.pi)
    q = chrono.ChQuaternion()
    q.SetFromAngleAxis(angle, axis)
    body.SetRot(q)

    
    
    
    vel = chrono.ChVectorD(random.uniform(-0.5, 0.5),
                           random.uniform(-0.5, 0.5),
                           random.uniform(-0.5, 0.5))
    body.SetPos_dt(vel)

    
    
    
    body.SetCollide(False)   
    body.SetBodyFixed(False) 

    system.Add(body)
    return body


def emit_particles(system, count):
    
    return [create_random_particle(system) for _ in range(count)]






def update_gravity(particles, G):
    
    n = len(particles)
    forces = [chrono.ChVectorD(0, 0, 0) for _ in range(n)]

    for i in range(n):
        for j in range(i + 1, n):
            bi = particles[i]
            bj = particles[j]

            
            r_vec = bj.GetPos() - bi.GetPos()
            dist = r_vec.Length()
            if dist < 1e-6:            
                continue

            
            f_mag = G * bi.GetMass() * bj.GetMass() / (dist * dist)

            
            unit = r_vec.GetNormalized()

            
            f_vec = unit * f_mag

            forces[i] += f_vec
            forces[j] -= f_vec   

    
    for idx, body in enumerate(particles):
        body.PutForce(forces[idx])






def main():
    
    
    
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, 0))   
    system.SetSolverType(chrono.ChSolver.SOR)
    system.SetMaxItersSpeed(500)
    system.SetStep(DT)

    
    
    
    app = irr.ChIrrApp(system,
                       u"Gravitational N‑body Simulation",
                       irr.dimension2d_u32(1280, 720))

    
    app.AddTypicalCamera(chrono.ChVectorD(0, 30, -45),
                         chrono.ChVectorD(0, 0, 0))
    app.AddTypicalLights()
    app.AddTypicalSky()
    app.SetShowDemoInfo(True)
    app.SetTimestep(DT)
    app.SetVideoframeRate(30)

    
    
    
    particles = emit_particles(system, NUM_PARTICLES)

    
    
    
    step_counter = 0
    while app.GetDevice().run():

        
        if step_counter % EMIT_EVERY_STEPS == 0:
            new_ones = emit_particles(system, NEW_PARTICLES_PER_EMIT)
            particles.extend(new_ones)

        
        update_gravity(particles, G_CONSTANT)

        
        app.BeginScene()
        app.DrawAll()
        app.DoStep()
        app.EndScene()

        step_counter += 1

    return 0


if __name__ == "__main__":
    sys.exit(main())