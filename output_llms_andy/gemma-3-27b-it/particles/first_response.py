import pychrono as chrono
import pychrono.irrlicht as chronoirr
import random

# --- Simulation setup ---
chrono.SetChronoDataPath("../data/")  # Adjust path if needed

# Create a Chrono physical system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Earth gravity

# --- Particle emitter ---
class ParticleEmitter(chrono.ChBehavior):
    def __init__(self, body, rate, min_radius, max_radius):
        chrono.ChBehavior.__init__(self, body)
        self.rate = rate
        self.min_radius = min_radius
        self.max_radius = max_radius
        self.time_since_last_emission = 0

    def DoStep(self, step):
        time = step.GetTime()
        delta_time = step.GetDeltaTime()
        self.time_since_last_emission += delta_time

        if self.time_since_last_emission >= 1.0 / self.rate:
            self.time_since_last_emission = 0

            # Create a new particle
            particle = chrono.ChBodyEasy()
            particle.SetBodyFixed(False)
            particle.SetCollide(True)
            particle.EnableShadowCasting(True)

            # Random shape (sphere)
            radius = random.uniform(self.min_radius, self.max_radius)
            sphere_shape = chrono.ChSphereShape()
            sphere_shape.GetSphereGeometry().rad = radius
            particle.AddAsset(sphere_shape)
            particle.SetMass(1.0)

            # Random position
            x = random.uniform(-5, 5)
            y = random.uniform(5, 10)  # Emit from above
            z = random.uniform(-5, 5)
            particle.SetPos(chrono.ChVectorD(x, y, z))

            # Random velocity
            vx = random.uniform(-1, 1)
            vy = random.uniform(-1, 1)
            vz = random.uniform(-1, 1)
            particle.SetWVel(chrono.ChVectorD(vx, vy, vz))

            # Random orientation (quaternion)
            qx = random.uniform(0, 1)
            qy = random.uniform(0, 1)
            qz = random.uniform(0, 1)
            qw = random.uniform(0, 1)
            particle.SetQuaternion(chrono.ChQuaternion(qx, qy, qz, qw))

            # Add the particle to the system
            system.Add(particle)

            # Apply custom gravitational force
            attractor = CustomGravityForce(particle)
            particle.AddForce(attractor)


# --- Custom gravitational force ---
class CustomGravityForce(chrono.ChForce):
    def __init__(self, body):
        chrono.ChForce.__init__(self, body)
        self.gravity_constant = 10.0  # Adjust for attraction strength

    def OnStep(self, step):
        # Apply attraction force towards the origin (0,0,0)
        pos = self.GetBody().GetPos()
        direction = -pos.Normalize()
        force_magnitude = self.gravity_constant / (pos.Length() + 0.1)  # Add a small value to avoid division by zero
        force = direction * force_magnitude
        self.GetBody().ApplyForce(force)

# --- Create a container body (ground) ---
ground = chrono.ChBodyEasy()
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, -1, 0))
ground.SetMass(0)
ground.AddAsset(chrono.ChBoxShape())
ground.SetCollide(True)
system.Add(ground)


# --- Create the particle emitter ---
emitter = ParticleEmitter(ground, 5, 0.2, 0.5)  # Emit 5 particles per second
system.Add(emitter)

# --- Visualization ---
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Particle Gravity Simulation')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 5, -10), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()

# --- Simulation loop ---
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(0.005)