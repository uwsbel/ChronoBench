import pychrono as ch
import pychrono.irrlicht as irr
import math
import random

# Create the Chrono::Engine physical system
sys = ch.ChSystemNSC()
sys.SetCollisionSystemType(ch.ChCollisionSystem.Type_BULLET)
sys.SetGravitationalAcceleration(ch.ChVector3d(0, 0, 0))  # No global gravity

# Create a particle emitter to generate particles
class ParticleEmitter:
    def __init__(self, system, num_particles):
        self.system = system
        self.num_particles = num_particles

    def emit_particles(self):
        for _ in range(self.num_particles):
            # Create a particle body
            particle = ch.ChBodyEasySphere(0.5, 1000, True, True)
            particle.SetPos(ch.ChVector3d(random.uniform(-10, 10), random.uniform(-10, 10), random.uniform(-10, 10)))
            particle.SetRot(ch.ChQuaterniond(1, 0, 0, 0))
            particle.SetLinVel(ch.ChVector3d(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1)))
            particle.SetAngVel(ch.ChVector3d(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1)))
            particle.GetMaterialSurfaceNSC().SetFriction(0.1)
            particle.GetMaterialSurfaceNSC().SetRestitution(0.1)
            particle.SetFixed(False)
            particle.SetCollide(True)
            self.system.Add(particle)

# Create the particle emitter
emitter = ParticleEmitter(sys, 10)

# Create the Irrlicht visualization
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Particle Emitter with Custom Gravity')
vis.Initialize()
vis.AddLogo(ch.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(ch.ChVector3d(0, 0, 10))
vis.AddTypicalLights()

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Emit particles at the beginning of the simulation
    if sys.GetChTime() < 0.1:
        emitter.emit_particles()

    # Apply custom gravitational forces to particles
    for body in sys.Get_bodylist():
        if body.IsFixed():
            continue
        force = ch.ChVector3d(0, 0, 0)
        for other in sys.Get_bodylist():
            if other is body or other.IsFixed():
                continue
            r = body.GetPos() - other.GetPos()
            distance = r.Length()
            if distance < 1e-3:
                continue  # Avoid singularity
            force += (r.Normalize() * (body.GetMass() * other.GetMass()) / (distance * distance))
        body.AddForce(force)

    # Advance the simulation
    sys.DoStepDynamics(1e-3)