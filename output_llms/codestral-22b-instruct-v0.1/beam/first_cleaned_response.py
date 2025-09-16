import pychrono as chrono
import pychrono.irrlicht as chronoirr


my_system = chrono.ChSystemNSC()


mesh = chrono.ChMesh()


node1 = mesh.AddNode(chrono.ChVectorD(0, 0, 0))
node2 = mesh.AddNode(chrono.ChVectorD(1, 0, 0))


node1.SetMass(1)
node2.SetMass(1)


beam = mesh.AddElementBeam(node1, node2)


beam.SetDiameter(0.01)
beam.SetYoungModulus(2e5)
beam.SetSectionCircular(0.005)


beam_body = chrono.ChBeamEulerStatic()


beam_body.Initialize(mesh)


my_system.Add(beam_body)


node2.SetForce(chrono.ChVectorD(0, -10, 0))


application = chronoirr.ChIrrApp(my_system, 'Beam Finite Elements', chronoirr.dimension2du(1024, 768))
application.AddTypicalLights()
application.AddCamera(chronoirr.vector3df(0.5, 0.5, -1.0))
application.AssetBindAll()
application.AssetUpdateAll()


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()