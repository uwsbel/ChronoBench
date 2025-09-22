import pychrono as chrono
import pychrono.irrlicht as chronoirr


physics_system = chrono.ChSystemSMC()


visualization = chronoirr.ChIrrApp(physics_system, 'PyChrono Particle Attraction', chronoirr.dimension2du(800, 600))


class ParticleEmitter:
    def __init__(self, system, position, rate, shape_generator):
        self.system = system
        self.position = position
        self.rate = rate
        self.shape_generator = shape_generator
        self.next_emission_time = 0

    def emit(self, time):
        if time > self.next_emission_time:
            self.next_emission_time = time + 1 / self.rate
            shape = self.shape_generator()
            particle = chrono.ChBodyEasySphere(shape.radius, 1000, True, True, self.position)
            particle.SetPos(self.position + chrono.ChVectorD(shape.offset))
            particle.SetRot(chrono.ChQuaternionD(shape.orientation))
            particle.SetVel(chrono.ChVectorD(shape.velocity))
            self.system.Add(particle)


import random

class RandomShape:
    def __init__(self):
        self.radius = random.uniform(0.1, 0.5)
        self.offset = chrono.ChVectorD(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1)) * 0.5
        self.orientation = chrono.ChQuaternionD(chrono.ChRandom())
        self.velocity = chrono.ChVectorD(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1)) * 0.1


emitter = ParticleEmitter(physics_system, chrono.ChVectorD(0, 0, 0), 10, RandomShape)


def apply_gravity(body1, body2, distance):
    force = -chrono.ChVectorD(0, 0, 1) * 10 * body1.GetMass() * body2.GetMass() / distance**2
    body1.Accumulate_force(force, body1.GetPos(), False)
    body2.Accumulate_force(-force, body2.GetPos(), False)


physics_system.SetInteractionFunction(apply_gravity)


time_step = 0.01
while visualization.GetDevice().run():
    time = physics_system.GetChTime()
    emitter.emit(time)
    physics_system.DoStepDynamics(time_step)
    visualization.DrawAll()