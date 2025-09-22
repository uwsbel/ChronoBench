import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


chrono.SetChronoDataPath('./data/')


mysystem = chrono.ChSystemNSC()
visualizer = chronoirr.ChVisualSystemIrrlicht(mysystem)


chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.001)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.001)
mysystem.SetMaxItersSolverSpeed(100)
mysystem.SetTol(1e-6)
mysystem.SetTolSpeed(1e-6)
mysystem.SetTimestep(0.01)


class ParticleEmitter:
    def __init__(self, system, num_particles):
        self.system = system
        self.num_particles = num_particles
        self.particles = []

    def create_particle(self, pos, vel, rot):
        particle = chrono.ChBodyEasySphere(self.system, 0.1, 1000, True, pos)
        particle.SetMass(1)
        particle.SetInertiaXX(chrono.ChVectorD(1, 1, 1))
        particle.SetPos(pos)
        particle.SetRot(chrono.Q_from_AngX(rot[0]) * chrono.Q_from_AngY(rot[1]) * chrono.Q_from_AngZ(rot[2]))
        particle.SetVel(chrono.ChVectorD(vel[0], vel[1], vel[2]))
        self.particles.append(particle)
        return particle

    def emit_particles(self):
        for _ in range(self.num_particles):
            pos = chrono.ChVectorD(np.random.uniform(-10, 10), np.random.uniform(-10, 10), np.random.uniform(-10, 10))
            vel = chrono.ChVectorD(np.random.uniform(-1, 1), np.random.uniform(-1, 1), np.random.uniform(-1, 1))
            rot = np.random.uniform(-np.pi, np.pi, 3)
            self.create_particle(pos, vel, rot)

emitter = ParticleEmitter(mysystem, 100)
emitter.emit_particles()


for particle in emitter.particles:
    mysystem.Add(particle)


class GravitationalForce(chrono.ChForce):
    def __init__(self, particle):
        super().__init__()
        self.particle = particle

    def ComputeF(self, xnew):
        force = chrono.ChVectorD(0, 0, 0)
        for other_particle in emitter.particles:
            if other_particle != self.particle:
                r = other_particle.GetPos() - self.particle.GetPos()
                dist = r.Length()
                force += (other_particle.GetMass() * self.particle.GetMass() / (dist ** 2)) * r / dist
        return force


for particle in emitter.particles:
    force = GravitationalForce(particle)
    particle.AddForce(force)


visualizer.SetWindowTitle("PyChrono Simulation")
visualizer.SetCamera(chrono.ChVectorD(0, 0, 10), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))
visualizer.SetTypicalSize(1)


while visualizer.Run():
    mysystem.DoStepDynamics(0.01)
    visualizer.BeginScene()
    visualizer.DrawAll()
    visualizer.EndScene()