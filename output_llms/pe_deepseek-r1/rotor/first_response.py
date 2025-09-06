import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# System setup
system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVector3d(0, 0, 0))  # Zero gravity for rotor simulation

# Parameters
beam_length = 2.0    # Total beam length
disk_radius = 0.3    # Flywheel radius
disk_thickness = 0.1 # Flywheel thickness
motor_speed = 5.0    # rad/s

# Create IGA beam
mesh = fea.ChMesh()
system.Add(mesh)

# Beam section properties
section = fea.ChBeamSectionCosserat()
section.SetAsCircularSection(0.05)  # Beam radius 5cm
section.SetYoungModulus(200e9)      # Steel
section.SetShearModulus(80e9)
section.SetRayleighDamping(0.01)

# Create nodes using IGA spline
order = 3  # Cubic spline
nodes = []
for i in range(4):  # 4 control points
    node = fea.ChNodeFEAxyzrot(chrono.ChFramed(
        chrono.ChVector3d(i * beam_length/3, 0, 0),
        chrono.QuatFromAngleY(chrono.CH_PI_2)
    ))
    node.SetMass(0)
    nodes.append(node)
    mesh.AddNode(node)

# Create IGA beam element
beam = fea.ChElementBeamIGA()
beam.SetNodes(nodes)
beam.SetOrder(order)
beam.SetSection(section)
mesh.AddElement(beam)

# Create flywheel at beam center
disk = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 
                                disk_radius, 
                                disk_thickness, 
                                7800)  # Steel density
disk.SetPos(chrono.ChVector3d(beam_length/2, 0, 0))
system.Add(disk)

# Connect flywheel to center beam node
lock = chrono.ChLinkMateFix()
lock.Initialize(disk, nodes[1])
system.Add(lock)

# Create rotational motor at first node
motor = chrono.ChLinkMotorRotationSpeed()
motor_frame = chrono.ChFramed(nodes[0].GetFrameAbs().GetCoord())
motor.Initialize(nodes[0], chrono.ChFrameD(), False)
motor.SetSpeedFunction(chrono.ChFunction_Const(motor_speed))
system.Add(motor)

# Visualization setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Jeffcott Rotor Simulation")
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1, 0.5, 2))
vis.AddTypicalLights()

# FEM visualization
visualizebeam = fea.ChVisualizationFEAmesh(mesh)
visualizebeam.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_ELEM_BEAM_MZ)
visualizebeam.SetColorscaleMinMax(-500, 500)
visualizebeam.SetSmoothFaces(True)
visualizebeam.SetWireframe(False)
mesh.AddVisualizationFEA(visualizebeam)

# Simulation loop
dt = 0.001
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(dt)