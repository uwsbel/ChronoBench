import pychrono as chrono
import pychrono.irrlicht as chronoirr








my_system = chrono.ChSystemNSC()


my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.3)
material.SetRestitution(0.2)


beam_mesh = chrono.ChMesh()


nodes = [
    chrono.ChNodeFEAxyz(chrono.ChVectorD(-1, 0, 0)),
    chrono.ChNodeFEAxyz(chrono.ChVectorD(1, 0, 0)),
    chrono.ChNodeFEAxyz(chrono.ChVectorD(1, 1, 0)),
    chrono.ChNodeFEAxyz(chrono.ChVectorD(-1, 1, 0)),
]


for node in nodes:
    beam_mesh.AddNode(node)


elements = [
    chrono.ChElementBeamEuler(nodes[0], nodes[1], material),
    chrono.ChElementBeamEuler(nodes[1], nodes[2], material),
    chrono.ChElementBeamEuler(nodes[2], nodes[3], material),
    chrono.ChElementBeamEuler(nodes[3], nodes[0], material),
]


for element in elements:
    beam_mesh.AddElement(element)


my_system.Add(beam_mesh)








myapplication = chronoirr.ChIrrApp(my_system, "Beam FEA Simulation", chronoirr.dimension2du(800, 600))


myapplication.AddTypicalCamera(chrono.ChVectorD(0, 2, -3), chrono.ChVectorD(0, 0, 0))


myapplication.SetSymbolscale(0.05)
myapplication.SetShowInfos(True)
myapplication.SetShowContacts(True)








while myapplication.GetDevice().run():
    myapplication.BeginScene()
    myapplication.Draw3D()
    myapplication.DoStep()
    myapplication.EndScene()