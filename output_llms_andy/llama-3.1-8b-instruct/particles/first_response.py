import chrono as chrono
from chrono import ChColor, ChVector
import math
import random

# Initialize the PyChrono environment
chrono.Initialize()

# Create a scene
scene = chrono.Scene()

# Create a ground body
ground_body = chrono.ChBody()
ground_body.SetName("Ground")
ground_body.SetBodyFixed(True)
ground_body.SetPos(ChVector(0, -1, 0))
ground_body.SetMass(10000)
ground_body.SetBodyInertia(1)
scene.Add(ground_body)

# Create a particle emitter
class ParticleEmitter:
    def __init__(self, position, radius, velocity, orientation):
        self.position = position
        self.radius = radius
        self.velocity = velocity
        self.orientation = orientation

    def emit_particles(self, scene, num_particles):
        for _ in range(num_particles):
            # Generate random particle properties
            particle_shape = random.choice(["sphere", "cube", "cylinder"])
            particle_radius = random.uniform(0.1, 1.0)
            particle_color = ChColor(random.random(), random.random(), random.random())
            particle_mass = random.uniform(1.0, 10.0)
            particle_pos = ChVector(
                self.position.x + random.uniform(-1.0, 1.0),
                self.position.y + random.uniform(-1.0, 1.0),
                self.position.z + random.uniform(-1.0, 1.0),
            )
            particle_vel = ChVector(
                self.velocity.x + random.uniform(-1.0, 1.0),
                self.velocity.y + random.uniform(-1.0, 1.0),
                self.velocity.z + random.uniform(-1.0, 1.0),
            )
            particle_orient = ChVector(
                self.orientation.x + random.uniform(-1.0, 1.0),
                self.orientation.y + random.uniform(-1.0, 1.0),
                self.orientation.z + random.uniform(-1.0, 1.0),
            )

            # Create a particle body
            particle_body = chrono.ChBody()
            particle_body.SetName("Particle")
            particle_body.SetMass(particle_mass)
            particle_body.SetBodyInertia(1)
            particle_body.SetPos(particle_pos)
            particle_body.SetVelocity(particle_vel)
            particle_body.SetOrientation(particle_orient)
            particle_body.SetShape(chrono.ChShape.ChSphere(particle_radius))
            particle_body.SetColor(particle_color)

            # Add a force to attract particles to the emitter
            force = chrono.ChForce()
            force.SetForce(ChVector(0, 0, 0))
            force.SetType(chrono.ChForce.FT_Fixed)
            particle_body.AddForce(force)

            # Add the particle to the scene
            scene.Add(particle_body)

# Create an emitter at the origin with a velocity of 0 and an orientation of (0, 0, 0)
emitter = ParticleEmitter(ChVector(0, 0, 0), 0.5, ChVector(0, 0, 0), ChVector(0, 0, 0))

# Emitter parameters
num_particles = 100
emitter_radius = 1.0

# Emitter emission
emitter.emit_particles(scene, num_particles)

# Visualize the simulation
chrono.Visualize()