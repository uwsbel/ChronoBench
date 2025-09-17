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
beam.SetYieldStrength(1000000.0)
beam.SetYoungModulus(21000000000.0)
beam.SetDensity(8000.0)
beam.SetRadius(0.05)
mesh.Add(beam)

node1 = chrono.ChNodeFEAxyzD()
node1.SetPos(chrono.ChVectorD(-0.5, 0, 0))
mesh.Add(node1)

node2 = chrono.ChNodeFEAxyzD()
node2.SetPos(chrono.ChVectorD(0.5, 0, 0))
mesh.Add(node2)

system.Add(mesh)


beam.SetForce(chrono.ChVectorD(0, 0, 1000.0))
beam.SetTorque(chrono.ChVectorD(0, 0, 0))
beam.SetMaterial(chrono.ChMaterialSurfaceNSC())
beam.SetSection(chrono.ChBeamSectionEmpty())


app = chrono.ChIrrApp(system, simulation)
app.SetWindowSize(800, 600)
app.AddTypicalShadows()
app.AddTypicalLights()
app.AddTypicalCamera(chrono.ChVectorD(1, 1, 1), chrono.ChVectorD(0, 0, 0))

while app.GetDevice().run():
    app.BeginScene(True, True, chrono.ChColor(0.3, 0.3, 0.3))
    app.DrawAll()
    app.EndScene()
    simulation.Advance()
    app.GetDevice().sleep(0.001)