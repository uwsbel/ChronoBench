import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.physics as chronophys
import random


chrono.SetChronoDataPath('path/to/chrono/data')  


system = chrono.ChSystemNSC()


visualization = chronoirr.ChIrrApp()
visualization.AddTypicalSky()
visualization.AddTypicalLogo()
visualization.AddTypicalLights()
visualization.AddLightWithShadow(chrono.ChVectorD(1, 1, 1), chrono.ChVectorD(0, 0, 0), 10, 5, 50, 40, 50)
visualization.SetSymbolScale(1.0)
visualization.SetContactDrawSize(0.01)
visualization.SetContactForceDrawSize(0.001)
visualization.SetFrameSpeed(60)
visualization.SetTryRealTimeStep(True)
visualization.AssetBindAll()
visualization.AssetUpdateAll()


system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))  


def create_random_particle():
    particle = chrono.ChParticle()
    particle.SetPos(chrono.ChVectorD(random.uniform(-5, 5), random.uniform(-5, 5), random.uniform(5, 10)))
    particle.SetMass(1)
    particle.SetVel(chrono.ChVectorD(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1)))
    particle.SetCollide(True)
    particle.SetBodyFixed(False)
    particle.SetName("Particle")
    return particle


def add_gravitational_force(particle1, particle2):
    G = 6.67430e-11  
    r = particle1.GetPos() - particle2.GetPos()
    distance = r.Length()
    if distance == 0:
        return
    force_magnitude = G * (particle1.GetMass() * particle2.GetMass()) / (distance ** 2)
    force_direction = r.Normalize()
    force = force_direction * force_magnitude
    particle1.AddForce(force)
    particle2.AddForce(-force)


particles = []
emission_rate = 1  
emission_interval = 1 / emission_rate


def main():
    visualization.SetWindowSize(800, 600)
    visualization.SetWindowTitle('Particle Gravity Simulation')
    visualization.Initialize()
    visualization.AddSystem(system)

    
    emission_timer = 0

    while visualization.Run():
        current_time = visualization.GetTime()

        
        if current_time - emission_timer >= emission_interval:
            new_particle = create_random_particle()
            system.Add(new_particle)
            particles.append(new_particle)
            emission_timer = current_time

        
        system.DoStepDynamics(0.01)

        
        for i in range(len(particles)):
            for j in range(i + 1, len(particles)):
                add_gravitational_force(particles[i], particles[j])

        
        visualization.BeginScene()
        visualization.Render()
        visualization.EndScene()

if __name__ == "__main__":
    main()