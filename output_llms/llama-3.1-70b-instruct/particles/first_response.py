import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import numpy as np

# Create the simulation system
mysystem = chrono.ChSystemNSC()

# Set the global gravity
mysystem.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create the particle emitter
class ParticleEmitter:
    def __init__(self, system, num_particles):
        self.system = system
        self.num_particles = num_particles
        self.particles = []

    def emit_particles(self):
        for _ in range(self.num_particles):
            # Random shape (sphere or box)
            shape_type = np.random.choice(['sphere', 'box'])

            if shape_type == 'sphere':
                # Random radius
                radius = np.random.uniform(0.1, 0.5)
                # Create a sphere particle
                particle = chrono.ChBodyEasySphere(self.system, radius, 1000, True)
            else:
                # Random box dimensions
                length = np.random.uniform(0.1, 0.5)
                width = np.random.uniform(0.1, 0.5)
                height = np.random.uniform(0.1, 0.5)
                # Create a box particle
                particle = chrono.ChBodyEasyBox(self.system, length, width, height, 1000, True)

            # Random position
            position = chrono.ChVectorD(np.random.uniform(-5, 5), np.random.uniform(-5, 5), np.random.uniform(-5, 5))
            particle.SetPos(position)

            # Random velocity
            velocity = chrono.ChVectorD(np.random.uniform(-1, 1), np.random.uniform(-1, 1), np.random.uniform(-1, 1))
            particle.SetVel(velocity)

            # Random orientation
            orientation = chrono.Q_from_AngX(np.random.uniform(-np.pi, np.pi))
            particle.SetRot(orientation)

            # Add particle to the system
            self.system.Add(particle)
            self.particles.append(particle)

# Create the particle emitter and emit particles
emitter = ParticleEmitter(mysystem, 10)
emitter.emit_particles()

# Create the custom gravitational force
class CustomGravitationalForce(chrono.ChForce):
    def __init__(self, particle1, particle2):
        super().__init__()
        self.particle1 = particle1
        self.particle2 = particle2

    def ComputeF(self, time):
        # Calculate the distance between particles
        distance = (self.particle2.GetPos() - self.particle1.GetPos()).Length()

        # Calculate the gravitational force
        force = 6.67430e-11 * self.particle1.GetMass() * self.particle2.GetMass() / (distance ** 2)

        # Apply the force to the particles
        self.particle1.ApplyForceToCOF(chrono.ChForce(chrono.ChVectorD(0, 0, force)))
        self.particle2.ApplyForceToCOF(chrono.ChForce(chrono.ChVectorD(0, 0, -force)))

# Add the custom gravitational force to the system
for i in range(len(emitter.particles)):
    for j in range(i + 1, len(emitter.particles)):
        force = CustomGravitationalForce(emitter.particles[i], emitter.particles[j])
        mysystem.AddForce(force)

# Create the Irrlicht visualization
myapplication = chronoirr.ChVisualSystemIrrlicht(mysystem)

# Set the camera position and look-at point
myapplication.SetCamera(chrono.ChVectorD(0, 0, 10), chrono.ChVectorD(0, 0, 0))

# Set the simulation step and time
mysystem.SetStep(0.01)
mysystem.SetMaxTime(10)

# Run the simulation
while mysystem.GetChTime() < mysystem.GetMaxTime():
    mysystem.DoStep()
    myapplication.BeginScene()
    myapplication.DrawAll()
    myapplication.EndScene()