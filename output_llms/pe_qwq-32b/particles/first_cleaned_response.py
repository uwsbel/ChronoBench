import math
import random
import pychrono as chrono
from pychrono import irrlicht as chronoirr

def main():
    
    my_system = chrono.ChSystemNSC()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(my_system)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle("Gravitational Particle Simulation")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(0, 5, 10))
    vis.AddTypicalLights()

    
    time_step = 0.001
    G = 1000.0  
    emission_interval = 0.5  
    last_emission_time = 0.0
    particles = []

    def create_particle():
        
        new_body = chrono.ChBody()
        new_body.SetMass(1.0)
        radius = 0.1
        inertia = chrono.ChVectorD(
            (2/5)*1.0 * radius**2,
            (2/5)*1.0 * radius**2,
            (2/5)*1.0 * radius**2
        )
        new_body.SetInertiaXX(inertia)

        
        pos = chrono.ChVectorD(
            random.gauss(0, 0.5),
            random.gauss(0, 0.5),
            random.gauss(0, 0.5)
        )
        new_body.SetPos(pos)

        
        vel = chrono.ChVectorD(
            random.uniform(-1, 1),
            random.uniform(-1, 1),
            random.uniform(-1, 1)
        )
        new_body.SetPos_dt(vel)

        
        axis = chrono.ChVectorD(
            random.uniform(-1, 1),
            random.uniform(-1, 1),
            random.uniform(-1, 1)
        ).Get_normalized()
        angle = random.uniform(0, 2 * math.pi)
        new_body.SetRot(chrono.Q_from_AngAxis(angle, axis))

        
        shape_type = random.choice(['sphere', 'box', 'cylinder'])
        if shape_type == 'sphere':
            shape = chrono.ChVisualShapeSphere(radius)
        elif shape_type == 'box':
            shape = chrono.ChVisualShapeBox(chrono.ChVectorD(0.2, 0.2, 0.2))
        elif shape_type == 'cylinder':
            shape = chrono.ChVisualShapeCylinder(0.1, 0.4)
        else:
            shape = chrono.ChVisualShapeSphere(radius)
        new_body.AddVisualShape(shape)
        new_body.SetCollide(False)
        return new_body

    
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        current_time = my_system.GetChTime()

        
        if current_time - last_emission_time >= emission_interval:
            new_body = create_particle()
            particles.append(new_body)
            my_system.Add(new_body)
            last_emission_time = current_time

        
        for body in particles:
            body.SetForce(chrono.ChVectorD(0, 0, 0))

        for i in range(len(particles)):
            body_i = particles[i]
            for j in range(i + 1, len(particles)):
                body_j = particles[j]
                pos_i = body_i.GetPos()
                pos_j = body_j.GetPos()
                delta = pos_j - pos_i
                r_sq = delta.x**2 + delta.y**2 + delta.z**2
                if r_sq < 1e-6:
                    continue
                r = math.sqrt(r_sq)
                unit_dir = delta / r
                m_i = body_i.GetMass()
                m_j = body_j.GetMass()
                force_magnitude = G * m_i * m_j / r_sq
                force = unit_dir * force_magnitude
                body_i.AddForce(force)
                body_j.AddForce(-force)

        
        my_system.DoStepDynamics(time_step)

    vis.End()

if __name__ == "__main__":
    main()