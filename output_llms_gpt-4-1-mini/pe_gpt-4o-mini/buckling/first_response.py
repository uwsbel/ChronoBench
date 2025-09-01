import os
import math
import numpy as np
import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# Initialize the PyChrono environment
chrono.SetChronoDataPath('/path/to/data')  # Set the correct path to the Chrono data directory
sys = chrono.ChSystemSMC()  # Create a Non-Smooth Contact system

# Visualization setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Beam Buckling Simulation')
vis.Initialize()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVector3d(5, 5, 5))
vis.AddSkyBox()

# Create the beam using FEA
mesh = fea.ChMesh()

# Define beam properties
beam_length = 5.0  # Length of the beam
beam_height = 0.1  # Height of the beam
beam_width = 0.1   # Width of the beam
density = 1000     # Density of the beam material
E = 210e9         # Young's modulus for steel
nu = 0.3          # Poisson's ratio

# Create beam section
msection = fea.ChBeamSectionEulerAdvanced()
msection.SetAsRectangularSection(beam_width, beam_height)
msection.SetYoungModulus(E)
msection.SetShearModulus(E / (2 * (1 + nu)))  # Shear modulus
msection.SetRayleighDamping(0.01)  # Damping factor

# Create nodes at both ends of the beam
hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
hnode2 = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(beam_length, 0, 0)))
mesh.AddNode(hnode1)
mesh.AddNode(hnode2)

# Create beam element
belement1 = fea.ChElementBeamEuler()
belement1.SetNodes(hnode1, hnode2)
belement1.SetSection(msection)
mesh.AddElement(belement1)

# Add mesh to the system
sys.Add(mesh)

# Create a fixed support at one end of the beam
fixed_support = chrono.ChLinkLock()
fixed_support.Initialize(hnode1, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
sys.Add(fixed_support)

# Create a motor to apply a force for buckling
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(hnode1, hnode2, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
motor.SetSpeedFunction(chrono.ChFunction_Const(0.0))  # Initially no speed
sys.Add(motor)

# Set up a custom function to apply a gradually increasing rotational speed
def apply_buckling_force(time):
    if time < 1.0:  # Apply force for the first second
        motor.SetSpeedFunction(chrono.ChFunction_Const(2.0 * math.pi))  # 1 rotation per second
    else:
        motor.SetSpeedFunction(chrono.ChFunction_Const(0.0))  # Stop the motor after 1 second

# Time step and solver settings
time_step = 0.01
solver = chrono.ChSolverPSOR()  # Use the PSOR solver for better stability
sys.SetSolver(solver)
sys.SetMaxItersSolverSpeed(50)  # Max iterations for speed solver
sys.SetMaxItersSolverStab(50)   # Max iterations for stability solver

# Simulation loop
while vis.Run():
    # Apply buckling force based on time
    apply_buckling_force(sys.GetChTime())
    
    # Advance the simulation by the time step
    sys.DoStepDynamics(time_step)
    
    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

# Cleanup
vis.Close()