import pychrono
import pychrono.irrlicht
import pychrono.vehicle


system = pychrono.ChSystemSMC()
irr = pychrono.irrlicht.ChIrrApp(system, "Beam Simulation", (1280, 720))


beam = pychrono.vehicle.ChBeam()
beam.SetEndPoint(pychrono.ChVectorD(0, 0, 0))
beam.SetFixedEnd(True)
beam.SetGravityOn(True)
system.Add(beam)


beam.SetPos(pychrono.ChVectorD(0, 0, 0))
beam.SetForce(pychrono.ChVectorD(0, -9.81, 0))


irr.AssetBindAll()
irr.AssetLoadTexture("textures/mud.jpg")

while system.GetChTime() < 10:
    system.DoStepDynamics(0.01)
    irr.BeginScene(True, True, pychrono.ChColor(0.4, 0.4, 0.4))
    irr.DrawAll()
    irr.EndScene()

irr.GetDevice().closeDevice()