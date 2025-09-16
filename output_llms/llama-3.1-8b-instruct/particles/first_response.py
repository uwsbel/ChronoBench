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
            # Generate random shape, position, velocity, and orientation
            shape_type = random.choice(["sphere", "cube", "cylinder"])
            particle_shape = None
            if shape_type == "sphere":
                particle_shape = chrono.ChSphereShape(0.1)
            elif shape_type == "cube":
                particle_shape = chrono.ChBoxShape(0.1, 0.1, 0.1)
            elif shape_type == "cylinder":
                particle_shape = chrono.ChCylinderShape(0.1, 0.2)

            particle_body = chrono.ChBody()
            particle_body.SetName("Particle")
            particle_body.SetPos(self.position + ChVector(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1)))
            particle_body.SetMass(random.uniform(1, 10))
            particle_body.SetBodyInertia(1)
            particle_body.AddAsset(particle_shape)

            # Apply custom gravitational force
            particle_force = chrono.ChForce()
            particle_force.SetForce(ChVector(0, 0, -1))
            particle_body.AddForce(particle_force)

            # Set initial velocity and orientation
            particle_body.SetAngularVel(ChVector(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1)))
            particle_body.SetPos(ChVector(self.position.x + random.uniform(-1, 1), self.position.y + random.uniform(-1, 1), self.position.z + random.uniform(-1, 1)))

            scene.Add(particle_body)

# Create a particle emitter
emitter = ParticleEmitter(ChVector(0, 1, 0), 0.1, ChVector(0, 0, 0), ChQuaternion(1, 0, 0, 0))

# Emit particles
emitter.emit_particles(scene, 100)

# Visualize the simulation using Irrlicht
chrono.SetCameraAutoScale(True)
chrono.SetCameraAutoPan(True)
chrono.SetCameraPosition(ChVector(0, 0, 5))
chrono.SetCameraTarget(ChVector(0, 0, 0))
chrono.SetRenderMode(chrono.RENDER_MODE_SHADE)
chrono.SetRenderDevice("Irrlicht")

# Run the simulation
chrono.Run(10)

# Clean up
chrono Finalize()