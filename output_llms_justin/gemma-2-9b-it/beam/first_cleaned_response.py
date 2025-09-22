import pychrono as chrono
import pychrono.irrlicht as vis


system = chrono.ChSystemNSC()


ground = chrono.ChBodyEasyBox(10, 10, 5, 1000)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(ground)


beam_material = chrono.ChMaterialSurfaceNSC()
beam_material.SetFriction(0.5)
beam_material.SetRestitution(0.1)


beam = chrono.ChBeamElement()
beam.SetMaterial(beam_material)
beam.SetCrossSection(chrono.ChBeamCrossSectionCircular(0.1))
beam.SetLength(2)


node1 = chrono.ChNodeBodyFixed(system)
node2 = chrono.ChNodeBodyFixed(system)


beam.AddNode(node1)
beam.AddNode(node2)


node1.SetPos(chrono.ChVectorD(0, 0, 0))
node2.SetPos(chrono.ChVectorD(2, 0, 0))


system.Add(beam)


vis_app = vis.ChVisualSystemIrrlicht(
    width=800, height=600,
    vSync=True,
    title="PyChrono Beam Simulation",
)


vis_app.Add(system)


while vis_app.Run():
    system.DoStepDynamics(chrono.ChTime(0.01))
    vis_app.Render()