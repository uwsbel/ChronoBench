import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.beam as beam
import math


chrono.SetChairsInCameraPos(True)
chrono.SetChronoDataPath("data/")


sys = chrono.ChSystemNSC()


mesh = beam.ChBeamFEAmesh(beam.ElemType.ELEMENT_1D, 3)


mesh.SetNdiv(10)
mesh.SetBeamWidth(0.05)
mesh.SetSectionType(beam.SectionType.SECTION_CIRCULAR)
mesh.SetSectionParams(0.01)
mesh.SetMaterial(chrono.ChContactMaterialNSC(0.6, 0.3, 0.2, 0.2, 0.2))


nodeA = mesh.AddNode(chrono.ChNodeFEAbase(chrono.ChFrameD(0, 0, 0)))
nodeB = mesh.AddNode(chrono.ChNodeFEAbase(chrono.ChFrameD(1, 0, 0)))
nodeC = mesh.AddNode(chrono.ChNodeFEAbase(chrono.ChFrameD(2, 0, 0)))
nodeD = mesh.AddNode(chrono.ChNodeFEAbase(chrono.ChFrameD(3, 0, 0)))


elem1 = mesh.AddElement(chrono.ChElementBeamTimoshenko(1, 2, 3, 4))
elem2 = mesh.AddElement(chrono.ChElementBeamTimoshenko(2, 3, 3, 4))
elem3 = mesh.AddElement(chrono.ChElementBeamTimoshenko(3, 4, 3, 4))


mesh.SetFixedPoint(1)
mesh.SetFixedPoint(4)


sys.Add(mesh.GetMesh())


app = irr.ChIrrApp(sys, "PyChrono beam demo", 800, 600, irr.DEVICE_KEYBOARD_IRRLICHT_CONTROL_KEYBOARD)


app.SetCameraVertical(chrono.ChVector3d(0, 0, 1))
app.SetCameraDistance(2.5)
app.SetCameraPos(chrono.ChVector3d(0, -2.5, 0.5))


app.SetRenderMode(irr.RenderMode.REALTIME)
app.SetTimestep(0.01)


while app.GetDevice().run():
    app.BeginScene()
    app.Render()
    app.EndScene()
    sys.DoStepDynamics(app.GetTimestep())
    app.GetDevice().sleep(chrono.GetSuggestedSimulationTimestep() * 1000000)