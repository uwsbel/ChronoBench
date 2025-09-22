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


vis = IrrlichtApplication(simulation, chrono.ChVectorD(3.4, 2.3, 1.3))
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("Beam Finite Elements")
vis.AttachSystem(system)
vis.AddMesh(mesh)
vis.Initialize()


while vis.GetDevice().run():
    simulation.DoStepDynamics(0.01)
    vis.BeginScene(True, True, chrono.ChColor(0.3, 0.3, 0.3))
    vis.Render()
    vis.EndScene()
    vis.GetDevice().driver.waitForVSync()