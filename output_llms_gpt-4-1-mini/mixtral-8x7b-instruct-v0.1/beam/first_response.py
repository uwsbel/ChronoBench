import chrono
from chrono import ChBody, ChSystem, ChBeamSection, ChLinkMate, ChLinkLockRevolute
from chrono.irrlicht import ChIrrApp

# Create a new ChronoEngine physical system
my_system = ChSystem()

# Set the gravity
my_system.set_gravity(chrono.ChVectorD(0, -9.81, 0))