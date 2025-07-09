import os
import math
import numpy as np
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea

# Set the path to the Chrono data directory
chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "data"))

# Create the simulation system
sys = chrono.ChSystemSMC()

# Create the Irrlicht visualization module
vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Beam Buckling Simulation")
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))
vis.AddTypicalLights()

# Define the beam properties
beam_L = 10.0  # Length
beam_wy = 1.0  # Width (y-axis)
beam_wz = 1.0  # Width (z-axis)
beam_E = 0.01e9  # Young's modulus
beam_G = 0.01e9 * 0.3  # Shear modulus
beam_rho = 1000  # Density

# Create the beam mesh
mesh = fea.ChMesh()
msection = fea.ChBeamSectionEulerAdvanced()
msection.SetAsRectangularSection(beam_wy, beam_wz)
msection.SetYoungModulus(beam_E)
msection.SetShearModulus(beam_G)
msection.SetRayleighDamping(0.000)

# Create the beam nodes
hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
hnode2 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_L, 0, 0)))

# Add the nodes to the mesh
mesh.AddNode(hnode1)
mesh.AddNode(hnode2)

# Create the beam element
belement1 = fea.ChElementBeamEuler()
belement1.SetNodes(hnode1, hnode2)
belement1.SetSection(msection)

# Add the element to the mesh
mesh.AddElement(belement1)

# Add the mesh to the system
sys.Add(mesh)

# Define the motor function
mfunY = chrono.ChFunctionSine(0.001, 1.5)

# Create the motor
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(mesh, chrono.ChFramed())
motor.SetMotionY(mfunY)

# Add the motor to the system
sys.Add(motor)

# Set the default solver and timestepper
sys.SetSolverType(chrono.ChSolver.Type_MINRES)
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT)

# Initialize the visualization
vis.Initialize()
vis.AttachSystem(sys)

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()
    sys.DoStepDynamics(0.01)

# Clean up
vis.GetSystem().Clear()