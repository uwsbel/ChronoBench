import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.iga as chronoiga
import numpy as np

# 1. Initialize the PyChrono environment and core components
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
system = chrono.ChSystemSMC()

# 2. Create the IGA beam (Jeffcott rotor shaft)
# Beam parameters
beam_length = 2.0      # meters
beam_radius = 0.03     # meters
beam_density = 7800    # kg/m^3 (steel)
beam_E = 2.1e11        # Young's modulus (Pa)
beam_G = 8.1e10        # Shear modulus (Pa)
beam_nel = 16          # Number of elements

# Create a straight beam along the X axis
beam_points = []
for i in range(beam_nel + 1):
    x = beam_length * i / beam_nel
    beam_points.append(chrono.ChVectorD(x, 0, 0))

# Create the IGA beam section
section = chronoiga.ChBeamSectionIGA()
section.SetDensity(beam_density)
section.SetYoungModulus(beam_E)
section.SetGwithPoissonRatio(beam_E, 0.3)
section.SetAsRectangularSection(beam_radius * 2, beam_radius * 2)

# Create the IGA beam mesh
beam_mesh = chronoiga.ChMesh()
beam = chronoiga.ChElementBeamIGA()
beam.SetNodes(beam_points)
beam.SetSection(section)
beam_mesh.AddElement(beam)
system.Add(beam_mesh)

# 3. Attach a flywheel at the center of the beam
flywheel_mass = 10.0     # kg
flywheel_radius = 0.15   # meters
flywheel_thickness = 0.05 # meters

# Center node index
center_idx = beam_nel // 2
center_pos = beam_points[center_idx]

# Create the flywheel body
flywheel = chrono.ChBodyEasyCylinder(flywheel_radius, flywheel_thickness, flywheel_mass, True, True)
flywheel.SetPos(center_pos + chrono.ChVectorD(0, 0, 0))
flywheel.SetBodyFixed(False)
system.Add(flywheel)

# Attach flywheel to beam at center node using a revolute joint (Jeffcott rotor model)
joint_flywheel = chrono.ChLinkMateGeneric(True, True, True, False, False, False)
joint_flywheel.Initialize(flywheel, beam_mesh, chrono.ChFrame(center_pos))
system.Add(joint_flywheel)

# 4. Add a rotational motor at one end of the beam
# Fix the left end of the beam to ground
ground = chrono.ChBody()
ground.SetBodyFixed(True)
system.Add(ground)

# Fix the first node of the beam to ground
left_pos = beam_points[0]
joint_fix = chrono.ChLinkMateGeneric(True, True, True, True, True, False)
joint_fix.Initialize(ground, beam_mesh, chrono.ChFrame(left_pos))
system.Add(joint_fix)

# Add a rotational motor at the left end (about X axis)
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(ground, beam_mesh, chrono.ChFrame(left_pos, chrono.Q_from_AngX(0)))
motor_speed = chrono.ChFunction_Const(20.0)  # 20 rad/s
motor.SetSpeedFunction(motor_speed)
system.Add(motor)

# 5. FEM visualization
fem_visual = chrono.ChVisualShapeFEA(beam_mesh)
fem_visual.SetFEMglyphType(chrono.ChVisualShapeFEA.GLYPH_NODE_DOT_POS)
fem_visual.SetFEMdataType(chrono.ChVisualShapeFEA.DATA_NONE)
fem_visual.SetSmoothFaces(True)
fem_visual.SetWireframe(False)
beam_mesh.AddVisualShapeFEA(fem_visual)

# 6. Irrlicht visualization
application = chronoirr.ChVisualSystemIrrlicht()
application.AttachSystem(system)
application.SetWindowSize(1024, 768)
application.SetWindowTitle('Jeffcott Rotor with IGA Beam')
application.Initialize()
application.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
application.AddSkyBox()
application.AddCamera(chrono.ChVectorD(beam_length/2, 0.5, 1.5), chrono.ChVectorD(beam_length/2, 0, 0))
application.AddTypicalLights()

# 7. Simulation loop
while application.Run():
    application.BeginScene()
    application.Render()
    application.DoStep()
    application.EndScene()