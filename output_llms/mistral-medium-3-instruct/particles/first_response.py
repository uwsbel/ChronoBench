import pychrono as chrono
import pychrono.irrlicht as chronoirr
import random
import math

# 1. Initialize the PyChrono environment and core components
chrono.SetChronoDataPath("PATH_TO_CHRONO_DATA")  # Set your Chrono data path here

# Create the physical system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  # Disable default gravity

# 2. Add the required physical systems and objects
# Create a material for the particles
particle_material = chrono.ChMaterialSurfaceNSC()
particle_material.SetFriction(0.3)
particle_material.SetRestitution(0.5)

# Particle emitter class
class ParticleEmitter:
    def __init__(self, system, emission_rate=5, max_particles=100):
        self.system = system
        self.emission_rate = emission_rate
        self.max_particles = max_particles
        self.particles = []
        self.time_since_emission = 0

    def emit_particle(self):
        if len(self.particles) >= self.max_particles:
            return

        # Random position in a sphere
        radius = 2.0
        theta = random.uniform(0, 2 * math.pi)
        phi = random.uniform(0, math.pi)
        x = radius * math.sin(phi) * math.cos(theta)
        y = radius * math.sin(phi) * math.sin(theta)
        z = radius * math.cos(phi)
        pos = chrono.ChVectorD(x, y, z)

        # Random velocity
        vel = chrono.ChVectorD(
            random.uniform(-0.5, 0.5),
            random.uniform(-0.5, 0.5),
            random.uniform(-0.5, 0.5)
        )

        # Random shape (sphere or box)
        if random.random() > 0.5:
            # Create a sphere
            radius = 0.1
            sphere = chrono.ChBodyEasySphere(radius, 1.0, True, True, particle_material)
            sphere.SetPos(pos)
            sphere.SetPos_dt(vel)
            self.system.Add(sphere)
            self.particles.append(sphere)
        else:
            # Create a box
            size = 0.1
            box = chrono.ChBodyEasyBox(size, size, size, 1.0, True, True, particle_material)
            box.SetPos(pos)
            box.SetPos_dt(vel)
            # Random orientation
            rot = chrono.ChQuaternionD()
            rot.Q_from_AngAxis(random.uniform(0, 2*math.pi), chrono.ChVectorD(1, 0, 0))
            box.SetRot(rot)
            self.system.Add(box)
            self.particles.append(box)

    def update(self, time_step):
        self.time_since_emission += time_step
        if self.time_since_emission >= 1.0/self.emission_rate:
            self.emit_particle()
            self.time_since_emission = 0

# Create the emitter
emitter = ParticleEmitter(system, emission_rate=10, max_particles=50)

# 3. Set necessary default parameters and custom gravitational forces
class GravitationalForceCallback(chrono.ChBodyForceCallback):
    def __init__(self, particles):
        super().__init__()
        self.particles = particles
        self.G = 0.1  # Gravitational constant

    def ForceCallback(self, body, time, force, torque, mode):
        if body in self.particles:
            for other in self.particles:
                if other != body:
                    # Calculate distance vector
                    delta = body.GetPos() - other.GetPos()
                    distance = delta.Length()

                    # Avoid division by zero and very close particles
                    if distance > 0.1:
                        # Calculate gravitational force
                        force_dir = delta.GetNormalized()
                        force_magnitude = self.G * body.GetMass() * other.GetMass() / (distance * distance)
                        force += force_dir * force_magnitude

# 4. Visualization setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Particle Gravity Simulation")
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(5, 5, 5))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(5, 10, 5), chrono.ChVectorD(0, 0, 0), 10, 1, 10, 40, 512, 512)

# Simulation parameters
time_step = 0.01
simulation_time = 20.0
current_time = 0

# Main simulation loop
while vis.Run() and current_time < simulation_time:
    # Update the emitter
    emitter.update(time_step)

    # Update the gravitational forces
    force_callback = GravitationalForceCallback(emitter.particles)
    for body in emitter.particles:
        body.AddForceCallback(force_callback)

    # Advance the simulation
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    system.DoStepDynamics(time_step)
    current_time += time_step

    # Print simulation time
    print(f"Simulation time: {current_time:.2f}s", end='\r')