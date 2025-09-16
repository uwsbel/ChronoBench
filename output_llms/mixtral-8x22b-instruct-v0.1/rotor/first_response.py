import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import numpy as np

# Initialize PyChrono environment and core components
my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

# Create the Irrlicht application
myapplication = chronoirr.ChIrrApp(my_system, 'Jeffcott Rotor Simulation', chronoirr.dimension2du(1024, 768))

# Create the beam using IGA
num_elem = 10
num_nodes = num_elem + 1
length = 1.0
width = 0.1
height = 0.1
E = 200e9
nu = 0.3
density = 7850

# Create the beam
beam = fea.ChBeamSectionEulerAdvanced(E, nu, density, width, height)
beam_material = fea.ChMaterialShellANCF(E, nu, density)
beam_mesh = fea.ChMesh()
beam_mesh.SetAutomaticAllocation(True)
beam_mesh.AddElementNodal(fea.ChElementBeamTaperedANCF_3)
beam_mesh.SetupInitial(num_nodes, beam_material)

# Set the geometry and material properties
for i in range(num_nodes):
    node = beam_mesh.Node(i)
    node.SetPos(chrono.ChVectorD(i * length / num_nodes, 0, 0))
    node.SetPos_dt(chrono.ChVectorD(0, 0, 0))
    node.SetPos_dtdt(chrono.ChVectorD(0, 0, 0))

# Create the flywheel
flywheel_mass = 10
flywheel_radius = 0.2
flywheel = chrono.ChBodyEasyCylinder(flywheel_radius, flywheel_mass, True, True)
flywheel.SetPos(chrono.ChVectorD(length / 2, 0, 0))
my_system.Add(flywheel)

# Create the rotational motor
motor_torque = 100
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(beam_mesh.Node(0).GetNodeCoord().node, flywheel, chrono.ChVectorD(0, 0, 1))
motor.Set_speed_dt(chrono.CH_C_PI / 2)
motor.Set_motor_torque(motor_torque)
my_system.Add(motor)

# Add the beam to the system
my_system.Add(beam_mesh)

# Enable FEM visualization
beam_mesh.SetFEMdataType(fea.ChVisualizationFEAtype.NODE_SPEED)
beam_mesh.SetFEMdataType(fea.ChVisualizationFEAtype.NODE_X)
beam_mesh.SetFEMdataType(fea.ChVisualizationFEAtype.NODE_Y)
beam_mesh.SetFEMdataType(fea.ChVisualizationFEAtype.NODE_Z)

# Run the simulation
myapplication.Run()