import chrono
from chrono import ChBody, ChSystem, ChBeamSection, ChLinkMate, ChLinkLockRevolute
from chrono.irrlicht import ChIrrApp


my_system = ChSystem()


my_system.set_gravity(chrono.ChVectorD(0, -9.81, 0))


beam_section = ChBeamSection()
beam_section.Set(0.1, 0.1, 0.01)  
beam_young_modulus = 210e9  
beam_density = 7800  
beam_material = chrono.ChMaterialSurfaceNSC()
beam_material.SetYoungModulus(beam_young_modulus)
beam_material.SetFriction(0.3)


beam_nodes = []
beam_nodes.append(my_system.NewNodalPointBody(chrono.ChVectorD(0, 0, 0)))
beam_nodes.append(my_system.NewNodalPointBody(chrono.ChVectorD(1, 0, 0)))
beam_nodes.append(my_system.NewNodalPointBody(chrono.ChVectorD(2, 0, 0)))
beam_nodes.append(my_system.NewNodalPointBody(chrono.ChVectorD(3, 0, 0)))
beam_nodes.append(my_system.NewNodalPointBody(chrono.ChVectorD(4, 0, 0)))

beam_elements = []
for i in range(len(beam_nodes) - 1):
    beam_elements.append(my_system.NewBeam(beam_nodes[i], beam_nodes[i + 1], beam_section))
    beam_elements[-1].SetMaterial(beam_material)
    beam_elements[-1].SetFixedBase(True)
    beam_elements[-1].SetVisualizationType(chrono.ChBeamVisualizationType.BAR)


joint = ChLinkLockRevolute()
joint.Initialize(beam_nodes[0], beam_nodes[-1], chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngZ(chrono.CH_C_PI / 2)))
my_system.Add(joint)


for node in beam_nodes:
    node.SetPos(node.GetPos())
    node.SetWvel(chrono.ChVectorD(0, 0, 0))


for element in beam_elements:
    element.SetForce(chrono.ChVectorD(0, 0, 0))
    element.SetTorque(chrono.ChVectorD(0, 0, 0))


my_application = ChIrrApp(my_system, 'Beam_Simulation', 800, 600, False)
my_application.AddTypicalLogo()
my_application.AddTypicalSkyBox()
my_application.AddTypicalCamera(chrono.ChVectorD(3, 3, 3))


for node in beam_nodes:
    my_application.AssetBrowser.AddAsset(node.GetChBody().GetCollisionModel(), 'mesh.obj')


while my_application.GetDevice().Run():
    my_system.DoStepDynamics(my_application.GetTimeStep())
    my_application.DrawAll()
    my_application.GetDevice().Render()