import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr


system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVector3d(0, 0, 0))  


beam_length = 2.0    
disk_radius = 0.3    
disk_thickness = 0.1 
motor_speed = 5.0    


mesh = fea.ChMesh()
system.Add(mesh)


section = fea.ChBeamSectionCosserat()
section.SetAsCircularSection(0.05)  
section.SetYoungModulus(200e9)      
section.SetShearModulus(80e9)
section.SetRayleighDamping(0.01)


order = 3  
nodes = []
for i in range(4):  
    node = fea.ChNodeFEAxyzrot(chrono.ChFramed(
        chrono.ChVector3d(i * beam_length/3, 0, 0),
        chrono.QuatFromAngleY(chrono.CH_PI_2)
    ))
    node.SetMass(0)
    nodes.append(node)
    mesh.AddNode(node)


beam = fea.ChElementBeamIGA()
beam.SetNodes(nodes)
beam.SetOrder(order)
beam.SetSection(section)
mesh.AddElement(beam)


disk = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 
                                disk_radius, 
                                disk_thickness, 
                                7800)  
disk.SetPos(chrono.ChVector3d(beam_length/2, 0, 0))
system.Add(disk)


lock = chrono.ChLinkMateFix()
lock.Initialize(disk, nodes[1])
system.Add(lock)


motor = chrono.ChLinkMotorRotationSpeed()
motor_frame = chrono.ChFramed(nodes[0].GetFrameAbs().GetCoord())
motor.Initialize(nodes[0], chrono.ChFrameD(), False)
motor.SetSpeedFunction(chrono.ChFunction_Const(motor_speed))
system.Add(motor)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Jeffcott Rotor Simulation")
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1, 0.5, 2))
vis.AddTypicalLights()


visualizebeam = fea.ChVisualizationFEAmesh(mesh)
visualizebeam.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_ELEM_BEAM_MZ)
visualizebeam.SetColorscaleMinMax(-500, 500)
visualizebeam.SetSmoothFaces(True)
visualizebeam.SetWireframe(False)
mesh.AddVisualizationFEA(visualizebeam)


dt = 0.001
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(dt)