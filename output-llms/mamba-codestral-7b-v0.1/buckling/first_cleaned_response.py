import pychrono
from pychrono import irrlicht
from pychrono.core import *
from pychrono.irrlicht import *


system = ChSystemSMC()



beam = ChBeam()
system.Add(beam)



beam.SetLength(1.0)
beam.SetWidth(0.1)
beam.SetYoungModulus(2e5)






joint = ChLinkLockRevolute()
joint.Initialize(beam.GetNode(1), beam.GetNode(2))
system.Add(joint)


irrlicht_app = ChIrrlichtApplication(system, L"Beam Buckling Simulation")



timestepper = TimestepperHHT()
timestepper.SetStepSize(1e-3)
system.SetTimestepper(timestepper)
solver = ChSolverSparseQR()
system.SetSolver(solver)


irrlicht_app.Run()