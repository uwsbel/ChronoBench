import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr

# Initialize the system
my_system = chrono.ChSystemNSC()
my_system.SetGravity(chrono.ChVectorD(0, 0, 0))  # Horizontal rotor, no gravity

# Beam parameters
beam_L = 1.0  # Length of the beam
beam_radius = 0.05  # Radius of the beam
num_elements = 4  # Even number for a center node

# Beam section properties
msection = fea.ChBeamSectionEulerAdvanced()
msection.SetRadius(beam_radius)
msection.SetYoungModulus(2.1e11)  # Steel
msection.SetShearModulus(8.1e10)
msection.SetDensity(7800)
area = 3.14159 * beam_radius**2
I = 3.14159 * beam_radius**4 / 4
J = 2 * I  # Torsion constant for circular section
msection.SetArea(area)
msection.SetIyy(I)
msection.SetIzz(I)
msection.SetJ(J)
msection.SetKsy(10)  # Shear coefficient (approximate)
msection.SetKsz(10)

# Create mesh and beam using IGA builder
mesh = fea.ChMesh()
builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(mesh, msection, num_elements,
                  chrono.ChVectorD(0, 0, 0),
                  chrono.ChVectorD(beam_L, 0, 0),
                  chrono.ChVectorD(0, 1, 0))  # Up direction

my_system.Add(mesh)

# Flywheel parameters
flywheel_mass = 10
flywheel_radius = 0.3
flywheel_length = 0.2  # Thickness

# Create flywheel body
flywheel = chrono.ChBody()
flywheel.SetMass(flywheel_mass)
flywheel.SetInertiaXX(chrono.ChVectorD(
    0.5 * flywheel_mass * flywheel_radius**2,  # Ixx (polar)
    (flywheel_mass / 12) * (3 * flywheel_radius**2 + flywheel_length**2),  # Iyy
    (flywheel_mass / 12) * (3 * flywheel_radius**2 + flywheel_length**2)   # Izz
))
flywheel.SetPos(chrono.ChVectorD(beam_L / 2, 0, 0))
flywheel.SetBodyFixed(False)
my_system.Add(flywheel)

# Find the center node of the beam
center_node = None
for node in mesh.GetNodes():
    if abs(node.GetPos().x - beam_L / 2) < 1e-6:
        center_node = node
        break

# Link the flywheel to the center node
if center_node:
    flywheel_link = fea.ChLinkNodeBody()
    flywheel_link.Initialize(center_node, flywheel)
    my_system.Add(flywheel_link)

# Create motor at the beam's end (first node)
end_node = mesh.GetNodes()[0]  # First node at (0,0,0)

# Create motor body
motor_body = chrono.ChBody()
motor_body.SetPos(end_node.GetPos())
motor_body.SetBodyFixed(False)
my_system.Add(motor_body)

# Link end node to motor body
motor_link = fea.ChLinkNodeBody()
motor_link.Initialize(end_node, motor_body)
my_system.Add(motor_link)

# Create motor constraint (rotational speed)
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(motor_body, my_system.GetGround(), chrono.ChFrameD(end_node.GetPos()))
my_system.Add(motor)

# Set motor speed (e.g., 10 rad/s)
speed_function = chrono.ChFunction_Const(10)
motor.SetSpeedFunction(speed_function)

# Visualization setup
application = irr.ChIrrApp(my_system, 'Jeffcott Rotor IGA Simulation', irr.dimension2du(1024, 768))
application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chrono.ChVectorD(0, 1, -2))

# Add FEM visualization
vis_shape = fea.ChVisualShapeFEA(mesh)
vis_shape.AddFEMglyphFEA(chrono.fea.ChGlyphType_GLYPH_ELEM_SHAFT, 0.01)
mesh.AddVisualShapeFEA(vis_shape)

# Add visual shapes for the flywheel
flywheel_vis = chrono.ChVisualShapeCylinder(flywheel_radius, flywheel_length)
flywheel_vis.SetColor(chrono.ChColor(1, 0, 0))
flywheel.AddVisualShape(flywheel_vis, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.Q_ROTATE_Z_TO_X))

# Add visual shape for the motor body (small sphere)
motor_vis = chrono.ChVisualShapeSphere(0.05)
motor_vis.SetColor(chrono.ChColor(0, 0, 1))
motor_body.AddVisualShape(motor_vis)

# Bind and update assets
application.AssetBindAll()
application.AssetUpdateAll()

# Simulation loop
application.SetTimestep(0.01)
application.SetTryRealtime(True)

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    my_system.DoStepDynamics(application.GetTimestep())