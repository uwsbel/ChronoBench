import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import numpy as np

# Set up the simulation system
sys = chrono.ChSystemNSC()

# Create a particle emitter
class ParticleEmitter:
    def __init__(self, sys, num_particles):
        self.sys = sys
        self.num_particles = num_particles

    def emit(self):
        for _ in range(self.num_particles):
            body = chrono.ChBodyEasySphere(0.1, 1000, True, True)
            body.SetPos(chrono.ChVector3d(np.random.uniform(-1, 1), np.random.uniform(-1, 1), np.random.uniform(-1, 1)))
            body.SetVel(chrono.ChVector3d(np.random.uniform(-1, 1), np.random.uniform(-1, 1), np.random.uniform(-1, 1)))
            body.SetAcc(chrono.ChVector3d(0, 0, 0))
            body.SetQuat(chrono.Q_from_AngX(np.random.uniform(0, 360)))
            self.sys.Add(body)

# Create a custom gravitational force
class GravitationalForce(chrono.ChForce):
    def __init__(self, G):
        super().__init__()
        self.G = G

    def CalcForce(self, body1, body2):
        r = body2.GetPos() - body1.GetPos()
        dist = r.Length()
        force = self.G * body1.GetMass() * body2.GetMass() / (dist**2)
        return force * r.Normalize()

# Create a simulation
def create_simulation():
    # Create a particle emitter
    emitter = ParticleEmitter(sys, 100)
    emitter.emit()

    # Create a custom gravitational force
    force = GravitationalForce(1e-6)

    # Add the force to the system
    sys.AddForce(force)

    # Create a visualization system
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('Particle Attraction Simulation')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0, 3, 6))
    vis.AddTypicalLights()

    # Simulate the system
    while vis.Run():
        sys.DoStepDynamics(0.01)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

create_simulation()