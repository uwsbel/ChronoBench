import pychrono as chrono
import chrono.irrlicht as irrlicht
import random


app = chrono.ChSystemNSC()
vis = irrlicht.ChIrrApp(app, 'Gravitational Particle Simulation')


emitter = chrono.ChParticleEmitter()
emitter.SetParticleType(chrono.ChParticle.Type.PARTICLE_SPHERE)
emitter.SetParticleRadius(0.05)
emitter.SetParticleMass(0.1)
emitter.SetParticleLifetime(10)
emitter.SetEmissionRate(100)
emitter.SetEmissionRadius(1)


emitter.SetParticleVelocity(chrono.ChVectorD(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1)))
emitter.SetParticleOrientation(chrono.Q_from_Euler(random.uniform(0, 2*3.14159), random.uniform(0, 2*3.14159), random.uniform(0, 2*3.14159)))


app.Add(emitter)


vis.AddCamera(chrono.ChVectorD(5, 5, 5))
vis.AddLight(chrono.ChVectorD(10, 10, 10))
vis.SetCamera(vis.GetActiveCamera())
vis.SetTimestep(1/240)


def apply_gravitational_force(particle, other_particle):
    distance = chrono.ChVectorD.distance(particle.GetPos(), other_particle.GetPos())
    force = chrono.ChVectorD(0, 0, 0)
    if distance > 0:
        force = chrono.ChVectorD(0, 0, -1) * (particle.GetMass() * other_particle.GetMass()) / (distance**2)
    return force


while vis.Run():
    app.DoStepDynamics(1/240)

    
    for particle in app.GetParticleList():
        for other_particle in app.GetParticleList():
            if particle != other_particle:
                force = apply_gravitational_force(particle, other_particle)
                particle.AddForce(force)

    vis.Render()