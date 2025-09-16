import pychrono as chrono
import pychrono.irrlicht as chronoirr
import random
import math


chrono.SetChronoDataPath('path_to_chrono_data')


system = chrono.ChSystemNSC()


visualization = chronoirr.ChVisualSystemIrrlicht()
visualization.AttachSystem(system)
visualization.SetWindowSize(1280, 720)
visualization.SetWindowTitle('PyChrono Particle Simulation')
visualization.Initialize()
visualization.AddTypicalSky()
visualization.AddTypicalLogo(chrono.GetChronoDataFile('logo_pychrono_small.png'))
visualization.AddLightWithShadow(chrono.ChVectorD(1, 1, 1), chrono.ChVectorD(0, 0, 0), 1, 1, 50, 50, 50)
visualization.SetCamera(chrono.ChVectorD(0, 0, 10), chrono.ChVectorD(0, 0, 0))


class ParticleEmitter:
    def __init__(self, system):
        self.system = system

    def emit_particle(self):
        
        pos = chrono.ChVectorD(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(0, 2))

        
        vel = chrono.ChVectorD(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1))

        
        quat = chrono.ChQuaternionD(random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1))
        quat.Normalize()

        
        particle = chrono.ChParticle()
        particle.SetPos(pos)
        particle.SetMass(1)
        particle.SetPos_dt(vel)
        particle.SetRot(quat)

        
        system.Add(particle)

        
        shape = chrono.ChSphereShape()
        shape.GetRadius() = random.uniform(0.1, 0.5)
        visual = chrono.ChVisualShapeSphere()
        visual.GetSphereGeometry().rad = shape.GetRadius()
        visual.SetColor(chrono.ChColor(1, 0, 0))
        particle.AddAsset(visual)


emitter = ParticleEmitter(system)


for _ in range(100):
    emitter.emit_particle()


def custom_gravity(particle, system):
    force = chrono.ChVectorD(0, 0, 0)
    for other in system.Get_physics_items():
        if isinstance(other, chrono.ChParticle) and other != particle:
            distance = (other.GetPos() - particle.GetPos()).Length()
            if distance > 0:
                direction = (other.GetPos() - particle.GetPos()).Normalize()
                force += direction * (1 / distance ** 2)  
    particle.ApplyForce(force)


def apply_custom_forces(system):
    for item in system.Get_physics_items():
        if isinstance(item, chrono.ChParticle):
            custom_gravity(item, system)


while visualization.Run():
    system.DoStepDynamics(0.01)  
    apply_custom_forces(system)
    visualization.Render()


visualization.Close()