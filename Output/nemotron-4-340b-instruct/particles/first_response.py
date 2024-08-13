import pychrono as chrono
import pychrono.irrlicht as chronoirr

# Initialize the physics system
physics_system = chrono.ChSystemSMC()

# Initialize the Irrlicht visualization
visualization = chronoirr.ChIrrApp(physics_system, 'PyChrono Particle Attraction', chronoirr.dimension2du(800, 600))