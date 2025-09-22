import pychrono.core as chrono
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBodyEasyBox(1, 0.1, 1, 1000, True, True)
ground.SetBodyFixed(True)
system.Add(ground)


num_nodes = 10  
beam_length = 5.0
node_spacing = beam_length / (num_nodes - 1)
node_radius = 0.05
node_mass = 0.1


nodes = []
for i in range(num_nodes):
    
    pos = chrono.ChVectorD(i * node_spacing, 0, 0)
    node = chrono.ChBodyEasySphere(node_radius, node_mass, True, True)
    node.SetPos(pos)
    system.Add(node)
    nodes.append(node)


nodes[0].SetBodyFixed(True)





cables = []
for i in range(num_nodes - 1):
    link = chrono.ChLinkCable()
    link.Initialize(nodes[i], nodes[i + 1])
    system.Add(link)
    cables.append(link)


for link in cables:
    link.SetMaterial(chrono.ChMaterialSurfaceNSC())
    link.SetColor(chrono.ChColor(0.3, 0.3, 0.8))
    link.SetCableRadius(0.02)


application = irr.ChIrrApp(system, 'ANCF Cable Beam Simulation', irr.dimension2d(1024, 768))
application.AddTypicalCamera(irr.vector3df(2, 2, 10))
application.SetContactMethod(irr.ConTACT_METHOD_NSC)
application.AssetBindAll()
application.AssetUpdateAll()


time_step = 1e-3
while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()

    
    for node in nodes:
        pos = node.GetPos()
        
        application.GetVideoDriver().draw3DPoint(
            irr.vector3df(pos.x, pos.y, pos.z),
            irr.SColor(255, 255, 0, 0),
            5
        )

    application.DoStep()
    application.EndScene()