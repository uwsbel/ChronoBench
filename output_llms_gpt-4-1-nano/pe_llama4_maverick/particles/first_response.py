import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np

# Set the path to Chrono's data directory
chrono.SetChronoDataPath('/path/to/chrono/data')

# Create a Chrono system
sys = chrono.ChSystemNSC()

# Create a particle emitter
class ParticleEmitter:
    def __init__(self, sys):
        self.sys = sys
        self.particle_count = 0

    def EmitParticle(self):
        # Randomly choose a shape (sphere or box)
        shape_type = np.random.choice(['sphere', 'box'])

        # Create a new body
        if shape_type == 'sphere':
            body = chrono.ChBodyEasySphere(np.random.uniform(0.1, 0.5), 1000, True, True)
        else:
            body = chrono.ChBodyEasyBox(np.random.uniform(0.1, 0.5), np.random.uniform(0.1, 0.5), np.random.uniform(0.1, 0.5), 1000, True, True)

        # Set random position, velocity, and orientation
        body.SetPos(chrono.ChVector3d(np.random.uniform(-5, 5), np.random.uniform(0, 10), np.random.uniform(-5, 5)))
        body.SetPos_dt(chrono.ChVector3d(np.random.uniform(-1, 1), np.random.uniform(-1, 1), np.random.uniform(-1, 1)))
        body.SetRot(chrono.Q_from_Euler123(chrono.ChVector3d(np.random.uniform(-np.pi, np.pi), np.random.uniform(-np.pi, np.pi), np.random.uniform(-np.pi, np.pi))))

        # Add the body to the system
        self.sys.Add(body)
        self.particle_count += 1

# Create an instance of the particle emitter
emitter = ParticleEmitter(sys)

# Set up the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Gravitational Attraction Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 10, 20))
vis.AddTypicalLights()

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()

    # Emit particles at regular intervals
    if np.random.uniform(0, 1) < 0.1:
        emitter.EmitParticle()

    # Apply custom gravitational forces to simulate particle attraction
    for body1 in sys.Get_bodylist():
        for body2 in sys.Get_bodylist():
            if body1 != body2:
                dist = (body1.GetPos() - body2.GetPos()).Length()
                if dist > 0:
                    force_dir = (body2.GetPos() - body1.GetPos()).GetNormalized()
                    force_mag = 0.1 * body1.GetMass() * body2.GetMass() / (dist * dist)
                    body1.AddForce(force_dir * force_mag)

    # Step the simulation
    sys.DoStepDynamics(0.01)

    vis.EndScene()