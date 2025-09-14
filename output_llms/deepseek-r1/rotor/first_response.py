import chrono
import chrono_irrlicht as chronoirr

# Parameters
L = 2.0           # Beam length
radius = 0.05     # Beam cross-sectional radius
flywheel_radius = 0.3
flywheel_thick = 0.1
density = 7850    # Steel density (kg/m³)
E = 2e11          # Young's modulus (Pa)
G = 8e10          # Shear modulus (Pa)
omega = 5.0       # Angular speed (rad/s)
n_elements = 10   # Number of beam elements

# Create system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  # No gravity

# Create FEM mesh
mesh = chrono.ChMesh()
system.Add(mesh)

# Create beam section
section = chrono.ChBeamSectionEulerAdvanced()
section.SetAsCircularSection(radius)
section.SetYoungModulus(E)
section.SetGshearModulus(G)
section.SetDensity(density)
section.SetRayleighDamping(0.01)

# Build IGA beam
builder = chrono.ChBuilderBeamIGA()
start_point = chrono.ChVectorD(0, 0, 0)
end_point = chrono.ChVectorD(0, L, 0)
builder.BuildBeam(mesh,                  # Mesh to store elements
                  section,               # Beam section
                  n_elements,            # Number of elements
                  start_point,           # Start point
                  end_point,             # End point
                  chrono.ChVectorD(1, 0, 0))  # Section direction

nodes = mesh.GetNodes()
node_first = nodes[0]
node_center = nodes[n_elements//2]
node_last = nodes[-1]

# Create and attach flywheel
flywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 
                                    flywheel_radius, 
                                    flywheel_thick, 
                                    density, 
                                    True)
flywheel.SetPos(node_center.GetPos())
system.Add(flywheel)

# Connect flywheel to center node
link_flywheel = chrono.ChLinkNodeBodyPositionRotation(node_center, flywheel)
system.Add(link_flywheel)

# Create rotational motor setup
motor_body = chrono.ChBody()
motor_body.SetBodyFixed(False)
system.Add(motor_body)

# Connect motor to ground with rotation constraint
motor_link = chrono.ChLinkMotorRotationSpeed()
motor_link.Initialize(motor_body, 
                     chrono.ChBody(),  # Ground body
                     chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), 
                                    chrono.Q_from_AngY(0)))
motor_link.SetSpeedFunction(chrono.ChFunction_Const(omega))
system.Add(motor_link)

# Fix first node's position
fixed_body = chrono.ChBody()
fixed_body.SetBodyFixed(True)
system.Add(fixed_body)
link_pos = chrono.ChLinkNodeFrame(node_first, fixed_body)
system.Add(link_pos)

# Connect motor rotation to first node
link_rot = chrono.ChLinkNodeSlopeFrame(node_first, motor_body)
system.Add(link_rot)

# Fix last node's position
fixed_body_end = chrono.ChBody()
fixed_body_end.SetBodyFixed(True)
system.Add(fixed_body_end)
link_pos_end = chrono.ChLinkNodeFrame(node_last, fixed_body_end)
system.Add(link_pos_end)

# Visualization setup
viz = chronoirr.ChVisualSystemIrrlicht()
viz.AttachSystem(system)
viz.Initialize()
viz.AddCamera(chrono.ChVectorD(0, -L*2, L), chrono.ChVectorD(0, L/2, 0))
viz.AddTypicalLights()

# FEM visualization
beam_viz = chronoirr.ChVisualizationFEAmesh(mesh)
beam_viz.SetFEMdataType(chronoirr.ChVisualizationFEAmesh.E_PLOT_ELEM_BEAM_MZ)
beam_viz.SetColorscaleMinMax(-1000, 1000)
beam_viz.SetSmoothFaces(True)
system.Add(beam_viz)

# Flywheel visualization
flywheel_shape = chrono.ChVisualShapeCylinder(flywheel_radius, flywheel_thick/2)
flywheel_shape.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
flywheel.AddVisualShape(flywheel_shape, chrono.ChFrameD(chrono.ChVectorD(0,0,0), 
                                                       chrono.Q_from_AngX(chrono.CH_C_PI_2)))

# Simulation loop
timestep = 0.001
while viz.Run():
    viz.BeginScene()
    viz.Render()
    viz.EndScene()
    system.DoStepDynamics(timestep)