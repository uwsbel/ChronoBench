import pychrono as chrono
import pychrono.irrlicht as chronoirr
import random
import math

def create_particle():
    
    pos = chrono.ChVectorD(
        random.uniform(-2, 2),
        random.uniform(-2, 2),
        random.uniform(0, 2)
    )
    vel = chrono.ChVectorD(
        random.uniform(-1, 1),
        random.uniform(-1, 1),
        random.uniform(-1, 1)
    )
    mass = random.uniform(0.1, 1.0)
    shape_type = random.choice(['sphere', 'box', 'cylinder'])

    
    if shape_type == 'sphere':
        radius = random.uniform(0.1, 0.3)
        body = chrono.ChBodyEasySphere(radius, mass)
    elif shape_type == 'box':
        sx = random.uniform(0.2, 0.5)
        sy = random.uniform(0.2, 0.5)
        sz = random.uniform(0.2, 0.5)
        body = chrono.ChBodyEasyBox(chrono.ChVectorD(sx, sy, sz), mass)
    elif shape_type == 'cylinder':
        radius = random.uniform(0.1, 0.3)
        height = random.uniform(0.2, 0.5)
        body = chrono.ChBodyEasyCylinder(radius, height, mass)

    
    body.SetPos(pos)
    body.SetPos_dt(vel)
    angle = random.uniform(0, 2 * math.pi)
    axis = chrono.ChVectorD(
        random.uniform(-1, 1),
        random.uniform(-1, 1),
        random.uniform(-1, 1)
    ).GetNormalized()
    body.SetRot(chrono.Q_from_AngAxis(angle, axis))

    
    material = chrono.ChMaterialSurface()
    material.SetFriction(0.5)
    material.SetRestitution(0.5)
    body.SetMaterialSurface(material)

    return body

def main():
    
    my_system = chrono.ChSystemNSC()
    my_system.Set_G_acc(chrono.ChVectorD(0, 0, 0))

    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('Gravitational Particle Simulation')
    vis.SetSymbolsScale(0.01)
    vis.SetCameraPosition(chrono.ChVectorD(5, 5, 5))
    vis.SetCameraRotation(chrono.Q_from_Euler_123(chrono.ChVectorD(-0.2, -0.5, 0)))
    my_system.SetVisualSystem(vis)

    particles = []
    emit_time = 0.0
    emit_interval = 0.5  
    step_size = 0.005

    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        current_time = my_system.GetChTime()

        
        if current_time >= emit_time:
            emit_time += emit_interval
            new_part = create_particle()
            my_system.Add(new_part)
            particles.append(new_part)

        
        for part in particles:
            part.SetForce(chrono.ChVectorD(0, 0, 0))
            part.SetTorque(chrono.ChVectorD(0, 0, 0))

        for i in range(len(particles)):
            for j in range(i + 1, len(particles)):
                part_i = particles[i]
                part_j = particles[j]

                pos_i = part_i.GetPos()
                pos_j = part_j.GetPos()
                mass_i = part_i.GetMass()
                mass_j = part_j.GetMass()

                diff = pos_j - pos_i
                dist_sq = diff.x**2 + diff.y**2 + diff.z**2
                if dist_sq < 1e-6:
                    continue
                dist = math.sqrt(dist_sq)
                G = 0.001  
                force_mag = G * mass_i * mass_j / dist_sq
                direction = diff / dist
                force = direction * force_mag

                part_i.SetForce(part_i.GetForce() + force)
                part_j.SetForce(part_j.GetForce() - force)

        
        my_system.DoStepDynamics(step_size)

    vis.End()

if __name__ == '__main__':
    main()