import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr
import math


system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVectorD(0, 0, 0))  


beam_section = fea.ChIgaBeamSectionEuler()
beam_section.SetYoungModulus(210e9)  
beam_section.SetGshearModulus(81e9)  
beam_section.SetAsCircularSection(0.05)  
beam_section.SetDensity(8000)  


builder = fea.ChBuilderBeamIGA()
beam_length = 1.0
num_elements = 20
builder.BuildBeam(system, beam_section, num_elements, beam_length, 
                  chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(1, 0, 0), chrono.CH_MAT_Z)
beam = builder.GetBeams()[0]
mesh = builder.GetMesh()
system.Add(mesh)


beam_nodes = builder.GetNodes()
mid_node = beam_nodes[len(beam_nodes) // 2]


flywheel_mass = 10.0
flywheel_inertia = chrono.ChVectorD(0.5, 0.5, 0.5)  
flywheel_body = chrono.ChBodyEasySphere(0.1, 8000, True, False)
flywheel_body.SetMass(flywheel_mass)
flywheel_body.SetInertiaXX(flywheel_inertia)
flywheel_body.SetName("Flywheel")
flywheel_body.SetPos(beam.GetNodeCoordinates(mid_node, fea.ChIgaBeamSection.CROSS_X))
system.AddBody(flywheel_body)


flywheel_connector = chrono.ChLinkPointPoint()
flywheel_connector.Initialize(flywheel_body, mid_node)
system.AddLink(flywheel_connector)


end_node = beam_nodes[-1]
dummy_body = chrono.ChBody()
system.AddBody(dummy_body)
dummy_body.SetBodyFixed(False)


motor_connector = chrono.ChLinkPointPoint()
motor_connector.Initialize(dummy_body, end_node)
system.AddLink(motor_connector)


motor = chrono.ChLinkMotorRotSpeed()
motor.Initialize(dummy_body, chrono.ChFrameD(chrono.ChVectorD(1, 0, 0)), 
                 chrono.ChCoordsysD(chrono.ChVectorD(1, 0, 0)))
motor.SetSpeedFunction(chrono.ChFunctionConst(100))  
system.AddLink(motor)


fea.ChVisualShapeFEA.SetFEMDefaultGraphicsWidth(5)
fea.ChVisualShapeFEA.SetFEMDefaultGraphicsThickness(5)
fea.ChVisualShapeFEA.SetFEMDefaultDrawMode(fea.ChVisualShapeFEA.DrawMode.NODES)
mesh.AddVisualShapeFEA()


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Jeffcott Rotor with IGA Beam")
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 2, 5), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()


time_step = 1e-3
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)