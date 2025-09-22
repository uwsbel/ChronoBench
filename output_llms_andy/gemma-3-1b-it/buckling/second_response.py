import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as pardiso
import pychrono.irrlicht as chronoirr
import os

# Custom function class for motor angle:
class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self):
        chrono.ChFunction.__init__(self)

    def GetVal(self, x):
        # Define a function that returns different angle values based on input x
        if x > 0.4:
            return chrono.CH_PI
        else:
            return -chrono.CH_PI * (1.0 - m.cos(chrono.CH_PI * x / 0.4)) / 2.0


# Define the output directory path
out_dir = chrono.GetChronoOutputPath() + "BEAM_BUCKLING"

# Create a Chrono::Engine physical system
sys = chrono.ChSystemSMC()

# Define key geometrical parameters
L = 1  # Length
H = 0.25  # Height
K = 0.05  # Crank length
vA = chrono.ChVector3d(0, 0, 0)  # Point A
vC = chrono.ChVector3d(L, 0, 0)  # Point C
vB = chrono.ChVector3d(L, -H, 0)  # Point B
vG = chrono.ChVector3d(L - K, -H, 0)  # Point G
vd = chrono.ChVector3d(0, 0, 0.0001)  # Small offset vector

# Create a truss body, fixed in space:
body_truss = chrono.ChBody()
body_truss.SetFixed(True)  # Make the truss immobile
sys.AddBody(body_truss)  # Add the truss to the physical system

# Attach a visualization shape to the truss
boxtruss = chrono.ChVisualShapeBox(0.02, 0.2, 0.1)
body_truss.AddVisualShape(boxtruss, chrono.QUNIT)

# Create a crank body:
body_crank = chrono.ChBody()
body_crank.SetPos((vB + vG) * 0.5)  # Set the position of the crank body
sys.AddBody(body_crank)  # Add the crank to the physical system

# Attach a visualization shape to the crank
boxtruss = chrono.ChVisualShapeBox(K, 0.02, 0.02)
body_crank.AddVisualShape(boxtruss, chrono.QUNIT)

# Create a rotational motor
motor = chrono.ChLinkMotorRotationAngle()
motor.Initialize(body_truss, body_crank, chrono.VECT_Y)

# Create a FEM mesh container:
mesh = fea.ChMesh()

# Define horizontal beam parameters
beam_wy = 0.10  # Width in Y direction
beam_wz = 0.01  # Width in Z direction

# Create section properties for the IGA beam
minertia = fea.ChInertiaCosseratSimple()
minertia.SetAsRectangularSection(beam_wy, beam_wz, 2700)  # Define the rectangular section with density

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(73.0e9)  # Set Young's modulus for elasticity
melasticity.SetShearModulusFromPoisson(0.3)  # Set shear modulus using Poisson's ratio
melasticity.SetAsRectangularSection(beam_wy, beam_wz)  # Define the section dimensions

msection1 = fea.ChBeamSectionEulerAdvanced()
msection1.SetDensity(2700)  # Set density
msection1.SetYoungModulus(73.0e9)  # Set Young’s modulus
msection1.SetShearModulusFromPoisson(0.3)  # Set shear modulus
msection1.SetRayleighDamping(0.000)  # Set Rayleigh damping
msection1.SetAsCircularSection(hbeam_d)  # Define the circular section diameter

# Build the Euler beam
builder_iga = fea.ChBuilderBeamEuler()
builder_iga.BuildBeam(mesh, msection1, 3, vA + vd, vC + vd, chrono.ChVector3d(0, 1, 0), 3)  # Add Euler beam to mesh

# Define nodes at the ends of the crank beam
node_crankG = builderA.GetLastBeamNodes()[0]
node_crankB = builderA.GetLastBeamNodes()[-1]

# Create a constraint between the crank beam and the vertical beam
constr_cbd = chrono.ChLinkMateGeneric()
constr_cbd.Initialize(node_crankG, body_crank, False, node_crankG.Frame(), node_crankG.Frame())
sys.Add(constr_cbd)
constr_cbd.SetConstrainedCoords(True, True, True, True, True, True)

# Create a constraint between the vertical beam and the crank beam
constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(node_down, node_crankB, False, node_crankB.Frame(), node_crankB.Frame())
sys.Add(constr_bc)
constr_bc.SetConstrainedCoords(True, True, True, True, True, False)

# Attach a visualization shape for the constraint
sphereconstr3 = chrono.ChVisualShapeSphere(0.01)
constr_bc.AddVisualShape(sphereconstr3)

# Final touches:
mesh.SetAutomaticGravity(False)  # Disable automatic gravity on the FEA elements

# Add the mesh to the physical system
sys.Add(mesh)

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  # Attach the visual system to the physical system
vis.SetWindowSize(1024, 768)  # Set the window size
vis.SetWindowTitle('Beams and constraints')  # Set the window title
vis.Initialize()  # Initialize the visualization system
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # Add a logo to the window
vis.AddSkyBox()  # Add a skybox for better visualization
vis.AddCamera(chrono.ChVector3d(0.0, 0.6, -1.0))  # Add a camera to the scene
vis.AddTypicalLights()  # Add typical lighting to the scene

# Use a solver that can handle stiffness matrices
pardiso_solver = pardiso.ChSolverPardisoMKL()
sys.SetSolver(pardiso_solver)  # Set the solver to the system

# Use the HHT timestepper for less numerical damping and higher accuracy
ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
sys.SetTimestepper(ts)  # Set the timestepper to the system

# Main simulation loop
while vis.Run():
    vis.BeginScene()  # Begin rendering the scene
    vis.Render()  # Render the scene
    chronoirr.drawGrid(vis, 0.05, 0.05, 20, 20,
                       chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))  # Draw a grid
    vis.EndScene()  # End rendering the scene
    sys.DoStepDynamics(0.001)  # Perform a simulation step with a timestep of 0.001 seconds