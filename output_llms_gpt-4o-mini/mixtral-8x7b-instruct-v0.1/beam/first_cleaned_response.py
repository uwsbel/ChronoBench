import chrono
from chrono import ChBody, ChSystem, ChBeamSection, ChLinkMate, ChLinkLockRevolute
from chrono.irrlicht import ChIrrApp


my_system = ChSystem()


my_system.set_gravity(chrono.ChVectorD(0, -9.81, 0))