import chrono as ch
import chrono.irrlicht as chirr
import numpy as np
import random

# Initialize PyChrono environment
ch.ChEngine.Initialize()

# Create the Irrlicht application
app = chirr.ChIrrApp()

# Create a Chrono system
system = ch.ChSystem()

# Set gravity (not necessary as it is enabled by default)
# system.Set_Gravity(0, -9.81, 0)

# Create a particle emitter
class ParticleEmitter(ch.ChParticleEmitter):
    def __init__(self):
        super(ParticleEmitter, self).__init__()
        self.SetParticleMass(0.1)
        self.SetParticleRadius(0.1)
        self.SetEmissionRate(10)
        self.SetInitialVelocity(ch.ChVector3f(0, 0, 0))
        self.SetInitialAngularVelocity(ch.ChVector3f(0, 0, 0))

    def EmitParticles(self, timeStep):
        for _ in range(self.GetEmissionRate()):
            particle = ch.ChParticle()
            particle.SetMass(self.GetParticleMass())
            particle.SetRadius(self.GetParticleRadius())
            particle.SetPos(ch.ChVector3f(
                random.uniform(-5, 5),
                random.uniform(0, 10),
                random.uniform(-5, 5)
            ))
            particle.SetLinVel(ch.ChVector3f(
                random.uniform(-1, 1),
                random.uniform(-2, 0),
                random.uniform(-1, 1)
            ))
            particle.SetAngVel(ch.ChVector3f(
                random.uniform(-0.1, 0.1),
                random.uniform(-0.1, 0.1),
                random.uniform(-0.1, 0.1)
            ))
            system.Add(particle)

emitter = ParticleEmitter()
system.Add(emitter)

# Custom gravitational force callback
class CustomGravity(ch.ChForceTorque):
    def __init__(self, body):
        super(CustomGravity, self).__init__()
        self.body = body

    def ComputeForce(self, body, M, H, timeStep):
        F = ch.ChVector3f()
        for other in system.Get_particles():
            if other != body:
                distance = (body.GetPos() - other.GetPos()).Length()
                if distance > 0:
                    grav_force = 0.1 / (distance ** 2)
                    direction = (other.GetPos() - body.GetPos()).Normalize()
                    F += grav_force * direction
        return F

# Add custom gravity to each particle
for _ in range(100):  # Assuming we start with 100 particles for simplicity
    particle = ch.ChParticle()
    particle.SetMass(0.1)
    particle.SetRadius(0.1)
    particle.SetPos(ch.ChVector3f(
        random.uniform(-5, 5),
        random.uniform(0, 10),
        random.uniform(-5, 5)
    ))
    particle.AddForce(CustomGravity(particle))
    system.Add(particle)

# Create the Irrlicht scene node
scene = app.AddScene()

# Add a camera
camera = chirr.ChIrrCamera()
camera.SetPosition(ch.ChVector3f(0, 5, -10))
camera.LookAt(ch.ChVector3f(0, 0, 0))
scene.Add(camera)

# Simulation loop
while app.Run():
    system.DoStepDynamics(1 / 60.0)
    app.Render()
    scene.Update()