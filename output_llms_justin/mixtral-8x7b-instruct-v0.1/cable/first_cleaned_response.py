import chrono
from chrono import (ChSystemNSC, ChFrame, ChVector, ChMatrix33, ChQuaternion, ChCoordsys)
from chrono.irrlicht import (ChIrrApp)
from chrono.fea import (ChElementCableANCF, ChLinkHinge)


my_system = ChSystemNSC()
my_application = ChIrrApp(my_system)


num_elements = 10
num_nodes = num_elements + 1


nodes = []
for i in range(num_nodes):
    x = i * 1.0
    nodes.append(ChNodeFEAxyzrot(x, 0, 0, 0, 0, 0))
my_system.Add(nodes)


elements = []
for i in range(num_elements):
    elements.append(ChElementCableANCF(nodes[i], nodes[i + 1], 1.0, 0.1))
    my_system.Add(elements[-1])


hinge = ChLinkHinge(nodes[0], ChCoordsys(ChVector(0, 0, 0)))
my_system.Add(hinge)


for node in nodes:
    node.SetPos(ChVector(node.GetPos().x, 0, 0))

for element in elements:
    element.SetSolverForceType(ChElementCableANCF.FORCE_TYPE_AXIAL)

my_system.Set_G_acc(ChVector(0, -9.81, 0))


my_application.AddTypicalLogo()
my_application.AddTypicalSky()
my_application.AddTypicalLights()
my_application.AddCamera(ChVectorD(5, 5, 5), ChVectorD(0, 0, 0))


my_application.SetTimestep(1e-4)
my_application.SetTryRealtime(True)
while my_application.GetDevice().run():
    my_system.DoStepDynamics(my_application.GetTimestep())

    
    for i, node in enumerate(nodes):
        if i == 0:
            continue
        node_pos = node.GetPos()
        my_application.assetScene.addLine(
            ChVectorD(nodes[i - 1].GetPos().x, nodes[i - 1].GetPos().y, 0),
            ChVectorD(node_pos.x, node_pos.y, 0),
            chrono.ChColor(0, 1, 0),
            2,
        )

    my_application.DrawAll()
    my_application.AssetBrowser()


my_application.SaveScreenshot("beam_simulation.png")
my_system.SaveCSV("beam_simulation.csv")