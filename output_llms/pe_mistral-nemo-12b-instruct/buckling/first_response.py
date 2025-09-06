import os
import math
import numpy as np
import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.robot as rob
import pychrono.vehicle as veh

# Set Chrono data path
chrono.SetChronoDataPath(os.path.join(os.environ['CHRONO_DATA_PATH'], '..'))

# Initialize the system
sys = chrono.ChSystemSMC()
sys.Set_Gravity(chrono.ChVector3d(0, 0, -9.81))

# Define beam properties
beam_L = 10
beam_wy = 0.1
beam_wz = 0.05
beam_E = 2.1e9  # Young's modulus
beam_G = 0.8e9  # Shear modulus

# Create mesh and beam section
mesh = fea.ChMesh()
msection = fea.ChBeamSectionEulerAdvanced()
msection.SetAsRectangularSection(beam_wy, beam_wz)
msection.SetYoungModulus(beam_E)
msection.SetShearModulus(beam_G)
msection.SetRayleighDamping(0.000)

# Add nodes and elements to the mesh
hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
hnode2 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_L, 0, 0)))
mesh.AddNode(hnode1)
mesh.AddNode(hnode2)

belement1 = fea.ChElementBeamEuler()
belement1.SetNodes(hnode1, hnode2)
belement1.SetSection(msection)
mesh.AddElement(belement1)

# Create a fixed constraint for the left end of the beam
link_fix = chrono.ChLinkLockLock()
link_fix.Initialize(hnode1.GetBody(), None, chrono.ChFramed())
sys.Add(link_fix)

# Create a motor function for applying a force at the right end of the beam
def ForceFunction(t):
    amplitude = 1000  # N
    frequency = 1.0  # Hz
    return amplitude * math.sin(2 * math.pi * frequency * t)

# Create a motor for applying the force
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(hnode2.GetBody(), None, chrono.ChFramed())
motor.SetForceFunction(chrono.ChFunction_Time(ForceFunction))

# Set up visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Beam Buckling Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0, 5))
vis.AddTypicalLights()

# Add visual shapes for the beam and the applied force
beam_shape = chrono.ChVisualShapeCylinder(beam_wz / 2, beam_L)
beam_shape.SetColor(chrono.ChColor(0, 0, 1))  # Blue color
hnode2.GetBody().AddVisualShape(beam_shape)

force_shape = chrono.ChVisualShapeBox(0.1, 0.1, 0.1)
force_shape.SetColor(chrono.ChColor(1, 0, 0))  # Red color
force_body = chrono.ChBody()
force_body.AddVisualShape(force_shape)
force_body.SetPos(chrono.ChVector3d(beam_L, 0, 0))
sys.Add(force_body)

# Simulation loop
while vis.Run():
    sys.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    chrono.ChThreadPoolWait()

vis.GetDevice().closeDevice()