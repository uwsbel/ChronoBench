import os
import math
import numpy as np
import pychrono as chrono
from pychrono import fea
from pychrono.robots import robosimian
from pychrono.vehicles import veh
from pychrono.irrlicht import chronoirr

# Initialize the PyChrono system with Non-Smooth Contact (NSC)
sys = chrono.ChSystemNSC()

# Set the data path
chrono.SetChronoDataPath(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))) + '/data/'

# Create the beam section for FEA
beam_section = fea.ChBeamSectionEulerAdvanced()
beam_section.SetAsRectangularSection(0.1, 0.05, 0.01)  # Width, Height, Thickness
beam_section.SetYoungModulus(210e9)  # Modulus of Elasticity
beam_section.SetShearModulus(210e9 * 0.3)  # Shear Modulus
beam_section.SetRayleighDamping(0.000)  # Rayleigh Damping

# Create the two supports
support1 = chrono.ChBody()
support1.SetPos(chrono.ChVector3d(0, 0, 0))
support1.EnableCollision(False)
support1.AddVisualShape(chrono.ChVisualShapeBox(chrono.ChVector3d(1, 0.5, 0.05)))
sys.Add(support1)

support2 = chrono.ChBody()
support2.SetPos(chrono.ChVector3d(4, 0, 0))
support2.EnableCollision(False)
support2.AddVisualShape(chrono.ChVisualShapeBox(chrono.ChVector3d(1, 0.5, 0.05)))
sys.Add(support2)

# Create the moving mass
moving_mass = fea.ChMovingMass()
moving_mass.SetMass(0.01)  # Mass
moving_mass.SetSize(0.1, 0.1, 0.01)  # Size (X, Y, Z)
moving_mass.SetVisualShape(chrono.ChVisualShapeSphere(0.05))  # Visual sphere
moving_mass.SetInitialPosition(chrono.ChVector3d(1, 0, 0))
moving_mass.SetCollision(False)
sys.Add(moving_mass)

# Create and initialize the timestepper
timestepper = fea.ChTimeStepper()
timestepper.SetTimeStep(0.001)
timestepper.Initialize(sys)
timestepper.SetStartTime(0)
timestepper.SetStopTime(5)
timestepper.Start()

# Visualization setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Beam Buckling Simulation')
vis.Initialize()
vis.AddSkyBox()
cam_pos = chrono.ChVector3d(2, 2, 2)
vis.AddCamera(cam_pos)

# Create and add the beam elements
beam_node1 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
beam_node2 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(4, 0, 0)))
beam_element = fea.ChElementBeamEuler()
beam_element.SetNodes(beam_node1, beam_node2)
beam_element.SetSection(beam_section)
fea.AddElement(beam_element)
sys.Add(beam_element)

# Create and add the moving load
mfunX = chrono.ChFunction_Sine(0.1, 0.5, 0)  # Horizontal movement
mfunY = chrono.ChFunction_Const(0.01)  # Vertical movement
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(moving_mass, support2, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI / 2)))
motor.SetSpeedFunction(chrono.ChFunction_Sine(100, 0.5))

# Add the motor to the system
sys.Add(motor)

# Add the visualization components
vis.AddLogo(os.path.join(chrono.GetChronoDataFile('pychrono'), 'pychrono.png'))
vis.AddTypicalLights()

# Simulation loop
while vis.Run():
    timestepper.DoStepDynamics()
    sys.DoStepDynamics(timestepper.GetTimeStep())
    
    # Update visualization
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Check for contacts
    num_contacts = sys.GetNumContacts()
    if num_contacts > 0:
        print(f"Number of contacts: {num_contacts}")