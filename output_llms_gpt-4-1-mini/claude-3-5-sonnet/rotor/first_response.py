import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# Initialize the ChSystem
system = chrono.ChSystemNSC()

# Create a mesh for the IGA beam
mesh = fea.ChMesh()

# Create the IGA beam
beam_length = 2.0
beam_radius = 0.01
beam_density = 7800  # kg/m^3

beam = fea.ChElementBeamIGA()
beam.SetNodes(fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0))),
              fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(beam_length, 0, 0))))
beam.SetSection(chrono.ChBeamSectionAdvanced())
beam.SetDensity(beam_density)
beam.SetInertiaVx(chrono.ChVectorD(0, 1, 0))
beam.SetInertiaVy(chrono.ChVectorD(0, 0, 1))
beam.SetCircularSection(beam_radius)

mesh.AddElement(beam)

# Create a flywheel and attach it to the center of the beam
flywheel_mass = 1.0  # kg
flywheel_radius = 0.2  # m
flywheel = chrono.ChBodyEasyCylinder(flywheel_radius, 0.05, flywheel_mass)
flywheel.SetPos(chrono.ChVectorD(beam_length / 2, 0, 0))
system.Add(flywheel)

# Create a joint between the flywheel and the beam's center
joint = chrono.ChLinkMateFix()
joint.Initialize(flywheel, beam.GetNodeN(1))
system.AddLink(joint)

# Add a motor to drive one end of the beam
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(beam.GetNodeN(0), system.GetGround(), chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
motor_speed = chrono.ChFunction_Const(5.0)  # 5 rad/s
motor.SetSpeedFunction(motor_speed)
system.AddLink(motor)

# Add the mesh to the system
system.Add(mesh)

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Jeffcott Rotor Simulation")
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(1, 1, 1), chrono.ChVectorD(beam_length / 2, 0, 0))
vis.AddTypicalLights()

# Create the FEM visualization
beam_asset = fea.ChVisualizationFEAmesh(mesh)
beam_asset.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_ELEM_BEAM_MZ)
beam_asset.SetColorscaleMinMax(-0.4, 0.4)
beam_asset.SetSmoothFaces(True)
beam_asset.SetWireframe(False)
mesh.AddAsset(beam_asset)

beam_asset_sec = fea.ChVisualizationFEAmesh(mesh)
beam_asset_sec.SetFEMglyphType(fea.ChVisualizationFEAmesh.E_GLYPH_NODE_CSYS)
beam_asset_sec.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_NONE)
beam_asset_sec.SetSymbolsThickness(0.006)
beam_asset_sec.SetSymbolsScale(0.01)
beam_asset_sec.SetZbufferHide(False)
mesh.AddAsset(beam_asset_sec)

# Simulation loop
time_step = 0.001
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)

vis.GetDevice().closeDevice()