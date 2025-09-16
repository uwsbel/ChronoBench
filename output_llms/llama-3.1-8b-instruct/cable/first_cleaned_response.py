import chrono as chrono
from chrono import Irrlicht


chrono.Initialize()
irr = Irrlicht.CreateWindowedScene('PyChrono Beam Simulation', size=(800, 600))


ground = chrono.ChBodyEasyBox(10, 0.1, 10, 1000)
ground.SetPos(0, 0, 0)
ground.SetRot(0, 0, 0)
ground.SetBodyFixed(True)
irr.Add(irr.ChNodeBody.CreateNodeBody(ground))


beam = chrono.ChBodyEasyBox(0.1, 10, 0.1, 1e-3)
beam.SetPos(0, 5, 0)
beam.SetRot(0, 0, 0)
beam.SetBodyFixed(False)
irr.Add(irr.ChNodeBody.CreateNodeBody(beam))


hinge = chrono.ChHinge2D(beam, ground)
hinge.SetAxis(chrono.ChVectorD(0, 1, 0))
irr.Add(irr.ChNodeHinge.CreateNodeHinge2D(hinge))


force = chrono.ChForce()
force.SetType(chrono.FET_FORCE)
force.SetDir(chrono.ChVectorD(0, -9.81, 0))
force.SetPos(0, 5, 0)
force.SetMagnitude(1e-3)
beam.AddForce(force)


cable = chrono.ChCable(beam, 0, 0, 0, 0, 0, 0)
cable.SetAnchor(chrono.ChVectorD(0, 0, 0))
cable.SetForce(chrono.ChVectorD(0, 1e-3, 0))
cable.SetStiffness(1e3)
irr.Add(irr.ChNodeCable.CreateNodeCable(cable))


irr.SetUserCamera(irr.ChCamera.CreateCamera())
irr.SetUserCamera(irr.ChCamera.GetCamera())


while not irr.IsQuitRequested():
    irr.BeginScene()
    irr.DrawAll()
    irr.EndScene()
    irr.ShowImage()
    chrono.DoStep()

chrono Finalize()