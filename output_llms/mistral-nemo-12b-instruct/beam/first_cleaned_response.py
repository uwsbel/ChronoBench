import pychrono as chrono
from pychrono.irrlicht import *


simulation = chrono.ChSimulation()
system = chrono.ChSystemSMC()
system.Set_Gravity(chrono.ChVectorD(0, -9.81, 0))
simulation.AddSystem(system)


mesh = chrono.ChMesh()
beam = chrono.ChBeam()
beam.SetMaterial(chrono.ChMaterialSurfaceNSC())
beam.SetSection(chrono.ChBeamSectionEmpty())
beam.SetLength(1.0)
beam.SetRadius(0.05)
beam.SetYoungModulus(2e7)
beam.SetGursonModel(chrono.ChGursonModel())
beam.SetYieldStrength(350e6)
beam.SetPoissonRatio(0.3)
mesh.Add(beam)

node = chrono.ChNodeFEAxyzD()
node.SetPos(chrono.ChVectorD(0, 0, 0))
mesh.AddNode(node)

node = chrono.ChNodeFEAxyzD()
node.SetPos(chrono.ChVectorD(1, 0, 0))
mesh.AddNode(node)

system.Add(mesh)


system.SetSolverType(chrono.ChSolverType.Solver_BDF)
system.SetSolverMaxIterations(100)
system.SetSolverTolerance(1e-3)


visualization = IrrlichtApplication(system, chrono.ChVectorD(0, 0, -10))
visualization.AddTypicalCamera()
visualization.AddTypicalLights()
visualization.AttachSystem(system)
visualization.SetWindowSize(800, 600)


while visualization.GetDevice().run():
    simulation.DoStepDynamics(0.01)
    visualization.DoStep()