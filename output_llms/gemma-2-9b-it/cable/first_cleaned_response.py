from pychrono import ChSystem, ChBody, ChLinkLock, ChMaterialSurface, ChCoordsys, ChFunction
from pychrono.irrlicht import ChIrrApp


system = ChSystem()
app = ChIrrApp(system, 'Beam Simulation', 'data/media')


beam_material = ChMaterialSurface(0.5, 0.5, 0.5)


beam_length = 1.0
beam_width = 0.1
beam_height = 0.1


node1 = ChCoordsys(ChVectorD(0, 0, 0))
node2 = ChCoordsys(ChVectorD(beam_length, 0, 0))


cable_element = ChFunction.CreateANCF(beam_material, node1, node2)
cable_element.SetRestLength(beam_length)


beam_body = ChBody(beam_material)
beam_body.SetPos(node1.pos)
beam_body.SetRot(node1.rot)
beam_body.SetMass(1.0)  


hinge_constraint = ChLinkLock(beam_body, system.GetGround(), ChCoordsys(ChVectorD(0, 0, 0)))
hinge_constraint.SetLockType(ChLinkLock.LOCK_XYZ)


system.Add(beam_body)
system.Add(cable_element)
system.Add(hinge_constraint)


while app.GetTimer() < 10:
    app.DrawAll()
    system.DoStepDynamics(0.01)


app.Close()