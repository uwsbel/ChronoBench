import pychrono as chrono
import pychrono.irrlicht as chronoirr
import random
import math


chrono.SetChronoDataPath('path/to/chrono/data')


system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


visualization = chronoirr.ChVisualSystemIrrlicht()
visualization.AttachSystem(system)
visualization.SetWindowSize(800, 600)
visualization.SetWindowTitle('Gravitational Attraction Simulation')
visualization.Initialize()
visualization.AddTypicalSky()
visualization.AddTypicalLogo(chrono.GetChronoDataFile('logo_pychrono_sdk.png'))
visualization.AddLightWithShadow(chrono.ChVectorD(1, 1, 1), chrono.ChVectorD(0, 0, 0), 1, 1, 10, 40, 50)


class ParticleEmitter:
    def __init__(self, system):
        self.system = system

    def emit_particle(self):
        
        x = random.uniform(-1, 1)
        y = random.uniform(5, 10)
        z = random.uniform(-1, 1)
        position = chrono.ChVectorD(x, y, z)

        
        vx = random.uniform(-0.1, 0.1)
        vy = random.uniform(-0.1, 0.1)
        vz = random.uniform(-0.1, 0.1)
        velocity = chrono.ChVectorD(vx, vy, vz)

        
        quaternion = chrono.ChQuaternionD(1, 0, 0, 0)
        quaternion.Q_from_AngAxis(random.uniform(0, 2 * math.pi), chrono.ChVectorD(0, 1, 0))

        
        particle = chrono.ChParticle()
        particle.SetPos(position)
        particle.SetWvel(velocity)
        particle.SetRot(quaternion)

        
        system.Add(particle)

        
        body = chrono.ChBodyEasySphere(0.1, 1000, True, True)
        body.SetPos(position)
        body.SetRot(quaternion)
        body.SetWvel(velocity)
        system.Add(body)


emitter = ParticleEmitter(system)


for _ in range(50):
    emitter.emit_particle()


def apply_gravitational_forces(system):
    bodies = system.Get_body_list()
    G = 6.67430e-11  

    for i in range(len(bodies)):
        for j in range(i + 1, len(bodies)):
            body1 = bodies[i]
            body2 = bodies[j]

            pos1 = body1.GetPos()
            pos2 = body2.GetPos()
            distance = (pos1 - pos2).Length()

            if distance > 0:
                force_magnitude = G * body1.GetMass() * body2.GetMass() / (distance ** 2)
                force_direction = (pos2 - pos1).GetUnitScaled(force_magnitude)

                body1.AddForce(force_direction)
                body2.AddForce(-force_direction)


while visualization.Run():
    system.DoStepDynamics(0.01)
    apply_gravitational_forces(system)
    visualization.Render()


visualization.Close()