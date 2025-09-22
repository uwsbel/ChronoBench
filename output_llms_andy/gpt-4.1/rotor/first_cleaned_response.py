import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.iga as chronoiga
import numpy as np


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
system = chrono.ChSystemSMC()



beam_length = 2.0      
beam_radius = 0.03     
beam_density = 7800    
beam_E = 2.1e11        
beam_G = 8.1e10        
beam_nel = 16          


beam_points = []
for i in range(beam_nel + 1):
    x = beam_length * i / beam_nel
    beam_points.append(chrono.ChVectorD(x, 0, 0))


section = chronoiga.ChBeamSectionIGA()
section.SetDensity(beam_density)
section.SetYoungModulus(beam_E)
section.SetGwithPoissonRatio(beam_E, 0.3)
section.SetAsRectangularSection(beam_radius * 2, beam_radius * 2)


beam_mesh = chronoiga.ChMesh()
beam = chronoiga.ChElementBeamIGA()
beam.SetNodes(beam_points)
beam.SetSection(section)
beam_mesh.AddElement(beam)
system.Add(beam_mesh)


flywheel_mass = 10.0     
flywheel_radius = 0.15   
flywheel_thickness = 0.05 


center_idx = beam_nel // 2
center_pos = beam_points[center_idx]


flywheel = chrono.ChBodyEasyCylinder(flywheel_radius, flywheel_thickness, flywheel_mass, True, True)
flywheel.SetPos(center_pos + chrono.ChVectorD(0, 0, 0))
flywheel.SetBodyFixed(False)
system.Add(flywheel)


joint_flywheel = chrono.ChLinkMateGeneric(True, True, True, False, False, False)
joint_flywheel.Initialize(flywheel, beam_mesh, chrono.ChFrame(center_pos))
system.Add(joint_flywheel)



ground = chrono.ChBody()
ground.SetBodyFixed(True)
system.Add(ground)


left_pos = beam_points[0]
joint_fix = chrono.ChLinkMateGeneric(True, True, True, True, True, False)
joint_fix.Initialize(ground, beam_mesh, chrono.ChFrame(left_pos))
system.Add(joint_fix)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(ground, beam_mesh, chrono.ChFrame(left_pos, chrono.Q_from_AngX(0)))
motor_speed = chrono.ChFunction_Const(20.0)  
motor.SetSpeedFunction(motor_speed)
system.Add(motor)


fem_visual = chrono.ChVisualShapeFEA(beam_mesh)
fem_visual.SetFEMglyphType(chrono.ChVisualShapeFEA.GLYPH_NODE_DOT_POS)
fem_visual.SetFEMdataType(chrono.ChVisualShapeFEA.DATA_NONE)
fem_visual.SetSmoothFaces(True)
fem_visual.SetWireframe(False)
beam_mesh.AddVisualShapeFEA(fem_visual)


application = chronoirr.ChVisualSystemIrrlicht()
application.AttachSystem(system)
application.SetWindowSize(1024, 768)
application.SetWindowTitle('Jeffcott Rotor with IGA Beam')
application.Initialize()
application.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
application.AddSkyBox()
application.AddCamera(chrono.ChVectorD(beam_length/2, 0.5, 1.5), chrono.ChVectorD(beam_length/2, 0, 0))
application.AddTypicalLights()


while application.Run():
    application.BeginScene()
    application.Render()
    application.DoStep()
    application.EndScene()