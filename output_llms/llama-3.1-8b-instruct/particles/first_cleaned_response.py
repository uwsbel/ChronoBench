import chrono as c
from chrono import Irlicht


c.init()


sim = c.ChronoSimulation()


visualizer = Irlicht(sim, width=800, height=600)


def generate_particle():
    particle = c.ChronoBodyDynamicsObject(sim)
    particle.SetMass(1.0)
    particle.SetPosition(c.Vec3(0, 0, 0))
    particle.SetVelocity(c.Vec3(0, 0, 0))
    particle.SetOrientation(c.Mat3.Identity)
    particle.SetShape(c.ChronoSphere(0.1))
    particle.SetEulerAngleXYZ(0, 0, 0)
    return particle


emitter = c.ChronoParticleEmitter(sim)
emitter.SetPosition(c.Vec3(0, 0, 0))
emitter.SetVelocity(c.Vec3(0, 0, 0))
emitter.SetOrientation(c.Mat3.Identity)
emitter.SetShape(c.ChronoSphere(0.1))


emitter.SetParticleRadius(0.1)
emitter.SetParticleVelocity(c.Vec3(0, 0, 0))
emitter.SetParticleOrientation(c.Mat3.Identity)
emitter.SetParticleMass(1.0)
emitter.SetParticleEulerAngleXYZ(0, 0, 0)


force = c.ChronoForceGravity(sim)
force.SetGravity(c.Vec3(0, 0, -9.81))


sim.SetGravity(c.Vec3(0, 0, -9.81))
sim.SetTimeStep(0.01)
sim.SetMaxStep(1e-6)


sim.AddBodyDynamicsObject(emitter)
sim.AddForce(force)


sim.NewStep()


while True:
    
    sim.DoStep()

    
    visualizer.Render()

    
    if c.is_key_pressed(c.KEY_ESCAPE):
        break


del sim
c.final()