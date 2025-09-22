import math as m  # Import the math library for trigonometric functions, constants, etc.
import pychrono as chrono  # Import the main PyChrono library
import pychrono.fea as fea  # Import the finite element analysis module from PyChrono
import pychrono.pardisomkl as mklsolver  # Import the Pardiso solver module from PyChrono
import pychrono.irrlicht as chronoirr  # Import the Irrlicht visualization module from PyChrono (changed alias for clarity)
import os  # Import the OS library for file and directory operations

# Custom function class for motor angle:
class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self):
        super().__init__() # Use super() for cleaner inheritance
    def GetVal(self, x):
        if x > 0.5:
            return chrono.CH_PI
        else:
            # Ensure smooth transition:  -(CH_PI/2) * (1 - cos(2*CH_PI*x)) for x in [0, 0.5]
            # The original function: -CH_PI * (1 - cos(CH_PI * x / 0.3)) / 2.0
            # if 0.3 is period for half cycle, then for x=0.3, val = -CH_PI
            # The current function goes from 0 to -CH_PI as x goes from 0 to 0.3
            # then for x from 0.3 to 0.5, it stays at -CH_PI
            # This seems fine, let's keep original logic if it's intended.
            if x < 0.3: # Original logic for the cos part
                 return -chrono.CH_PI * (1.0 - m.cos(chrono.CH_PI * x / 0.3)) / 2.0
            elif x <=0.5: # Between 0.3 and 0.5, it should reach -CH_PI
                 return -chrono.CH_PI # Assuming it reaches -CH_PI at x=0.3 and stays there until 0.5
            else: # x > 0.5 as per outer if
                 return chrono.CH_PI


# Define the output directory path (not used in this script but good practice)
out_dir = chrono.GetChronoOutputPath() + "BEAM_FAILED"
if not os.path.exists(out_dir):
    os.makedirs(out_dir)

# Create a Chrono::Engine physical system
sys = chrono.ChSystemSMC() # Corrected: ChSystemSMC

# Define key geometrical parameters
L = 1.2
H = 0.4
K = 0.07
vA = chrono.ChVector3d(0, 0, 0)
vC = chrono.ChVector3d(L, 0, 0)
vB = chrono.ChVector3d(L, -H, 0)
vG = chrono.ChVector3d(L - K, -H, 0)
vd = chrono.ChVector3d(0, 0, 0.0001) # Small offset

# Create a truss body, fixed in space:
body_trss = chrono.ChBody()
body_trss.SetFixed(True)
sys.AddBody(body_trss)

# Attach a visualization shape to the truss
boxtruss_vis = chrono.ChVisualShapeBox(0.03, 0.25, 0.15) # Renamed for clarity
body_trss.AddVisualShape(boxtruss_vis, chrono.ChFramed(chrono.ChVector3d(-0.01, 0, 0), chrono.QUNIT))

# Create a crank body:
body_crank = chrono.ChBody()
body_crank.SetPos((vC + vG) * 0.5) # Initial position
sys.AddBody(body_crank)

# Attach a visualization shape to the crank
boxcrank_vis = chrono.ChVisualShapeBox(K, 0.05, 0.03) # Renamed for clarity
body_crank.AddVisualShape(boxcrank_vis)

# Create a rotational motor
motor = chrono.ChLinkMotorRotationAngle() # Corrected: ChLinkMotorRotationAngle for "motor angle"
motor.Initialize(body_trss, body_crank, chrono.ChFramed(vG)) # Corrected: body_trss
myfun = ChFunctionMyFun()
motor.SetAngleFunction(myfun) # Corrected: SetAngleFunction
sys.AddLink(motor) # Using AddLink for clarity

# Create a FEM mesh container:
mesh = fea.ChMesh()

# Define horizontal beam parameters
beam_wy = 0.12 # Section y-width
beam_wz = 0.15 # Section z-height

# Create section properties for the IGA beam
minertia = fea.ChInertiaCosseratSimple() # Corrected: ChInertiaCosseratSimple
minertia.SetAsRectangularSection(beam_wy, beam_wz)
minertia.SetDensity(2700) # Density set on inertia property
melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(72.0e9)
melasticity.SetShearModulusFromPoisson(0.35)
# Corrected: melasticity.SetAsRectangularSection removed as it's not a method and redundant here.
msection1 = fea.ChBeamSectionCosserat(minertia, melasticity) # Corrected: ChBeamSectionCosserat
msection1.SetDrawThickness(beam_wy * 0.5, beam_wz * 0.5) # Adjusted for better visualization

# Build the IGA beam
builder_iga = fea.ChBuilderBeamIGA()
# Corrected: Y_direction changed from VECT_X to something orthogonal to beam axis
builder_iga.BuildBeam(mesh, msection1, 30, vA, vC, chrono.ChVector3d(0, 0, 1), 3)

# Fix the first node of the horizontal beam
builder_iga.GetLastBeamNodes()[0].SetFixed(True) # Corrected: .front() to [0]
# Corrected: node_tip index from 65 to -1 (last element)
node_tip = builder_iga.GetLastBeamNodes()[-1]
# Corrected: node_mid index. For 33 nodes (30 elements, order 3), mid is approx 16. (This node is unused later)
num_iga_nodes = len(builder_iga.GetLastBeamNodes())
node_mid = builder_iga.GetLastBeamNodes()[num_iga_nodes // 2]


# Define vertical beam parameters using Euler beams
section2 = fea.ChBeamSectionEulerAdvanced()
hbeam_d = 0.05
section2.SetDensity(2500)
section2.SetYoungModulus(75.0e9)
section2.SetShearModulusFromPoisson(0.25)
section2.SetRayleighDamping(0.000) # Note: Rayleigh damping might be useful for stability
section2.SetAsCircularSection(hbeam_d)

# Build the vertical beam with Euler elements
builderA = fea.ChBuilderBeamEuler()
builderA.BuildBeam(mesh, section2, 10, vC + vd, vB + vd, chrono.ChVector3d(1, 0, 0)) # Y_dir_BC is X global

# Define nodes at the top and bottom of the vertical beam
node_top = builderA.GetLastBeamNodes()[0] # Assuming connection at the very start of this beam. Original was [1]
node_down = builderA.GetLastBeamNodes()[-1]

# Create a constraint between the horizontal and vertical beams
# Corrected: Changed to ChLinkMateGeneric to use SetConstrainedCoords meaningfully.
constr_bb = chrono.ChLinkMateGeneric()
# Initialize with node_tip (end of horizontal beam) and node_top (start of vertical beam)
# Using a common frame for initialization, e.g. node_tip's frame
constr_bb.Initialize(node_tip, node_top, False, node_tip.Frame(), node_top.Frame())
sys.AddLink(constr_bb) # Using AddLink
# This constrains translations in X and Z, rotations about Y and Z. Free: Ty, Rx.
constr_bb.SetConstrainedCoords(True, False, True, False, False, False) # Constrain Px, Pz. Free Py, Rx, Ry, Rz. (X,Y,Z,Rx,Ry,Rz)

# Attach a visualization shape for the constraint
sphereconstr2_vis = chrono.ChVisualShapeSphere(0.02) # Renamed
constr_bb.AddVisualShape(sphereconstr2_vis)

# Create a crank beam
section3 = fea.ChBeamSectionEulerAdvanced()
crankbeam_d = 0.06
section3.SetDensity(2800)
section3.SetYoungModulus(75.0e9)
section3.SetShearModulusFromPoisson(0.25)
section3.SetRayleighDamping(0.000)
section3.SetAsCircularSection(crankbeam_d)

# Build the crank beam with Euler elements
builderB = fea.ChBuilderBeamEuler() # Corrected: fea
builderB.BuildBeam(mesh, section3, 4, vG + vd, vB + vd, chrono.ChVector3d(0, 1, 0)) # Y_dir_BC is Y global

# Define nodes at the ends of the crank beam
node_crnkG = builderB.GetLastBeamNodes()[0]
node_crankB = builderB.GetLastBeamNodes()[-1]

# Create a constraint between the crank beam and the body crank
constr_cbd = chrono.ChLinkMatePrismatic()
constr_cbd.Initialize(node_crnkG, body_crank, False, node_crnkG.Frame(), node_crnkG.Frame())
sys.AddLink(constr_cbd) # Using AddLink
# Corrected: Removed SetConstrainedCoords for ChLinkMatePrismatic

# Create a constraint between the vertical beam and the crank beam
constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(node_down, node_crankB, False, node_crankB.Frame(), node_crankB.Frame())
sys.AddLink(constr_bc) # Using AddLink
constr_bc.SetConstrainedCoords(True, True, False, True, True, False) # Constrain Px,Py,Rx,Ry. Free Pz,Rz.

# Attach a visualization shape for the constraint
sphereconstr3_vis = chrono.ChVisualShapeSphere(0.01) # Renamed
constr_bc.AddVisualShape(sphereconstr3_vis)

# Final touches:
mesh.SetAutomaticGravity(True) # Gravity is applied to FEA elements
sys.Add(mesh)

# Create visualization for the FEM mesh:
mvisualizebeamA = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MY) # Bending moment about Y
mvisualizebeamA.SetColorscaleMinMax(-400, 400)
mvisualizebeamA.SetSmoothFaces(True) # Usually for shells, but doesn't hurt
mvisualizebeamA.SetWireframe(False)
mesh.AddVisualShape(mvisualizebeamA) # Use AddVisualShape for ChVisualShapeFEA on mesh

mvisualizebeamC = chrono.ChVisualShapeFEA(mesh)
# Corrected: Changed glyph type and removed problematic DataType_FULL
mvisualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS) # Show coordinate systems at nodes
mvisualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE) # DataType_NONE is often used with CSYS glyphs
mvisualizebeamC.SetSymbolsThickness(0.005)
mvisualizebeamC.SetSymbolsScale(0.01) # Adjusted scale for better visibility
mvisualizebeamC.SetZbufferHide(False) # Typically False to see glyphs correctly
mesh.AddVisualShape(mvisualizebeamC) # Use AddVisualShape

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768) # Slightly larger window
vis.SetWindowTitle('Corrected Beam Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png')) # .png often preferred
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.0, 0.6, -1.8)) # Adjusted camera
vis.AddTypicalLights()

# Use a solver that can handle stiffness matrices
# custom_solver = mklsolver.ChSolverPardisoMKL() # MKL is good for FEA
# sys.SetSolver(custom_solver)

# For ChSystemSMC, if MKL is used, it's for the linear solve within the timestepper.
# If contacts were a major part, ChSolverSMC would be default.
# For FEA with HHT, MKL is fine. Set it on the timestepper if HHT requires linear solver config.
# Or set on system if it's a direct sparse solver for the whole system.

sys.SetSolverType(chrono.ChSolver.Type_PARDISO_MKL) # A more modern way to set MKL solver


# Use the HHT timestepper for less numerical damping and higher accuracy
ts = chrono.ChTimestepperHHT(sys) # Corrected: ChTimestepperHHT
# ts.SetStepControl(True) # Corrected: Removed, HHT has internal controls
ts.SetAlpha(-0.2)  # HHT alpha factor, e.g., -0.2 for moderate damping, 0 for Newmark avg. const. acc.
ts.SetMaxiters(10) # Max iterations for HHT Newton-Raphson
ts.SetAbsoler(1e-4) # Absolute tolerance for HHT convergence
ts.SetMode(chrono.ChTimestepperHHT.ACCELERATION) # Solve for accelerations
ts.SetVerbose(False) # Set to True for detailed HHT convergence info
sys.SetTimestepper(ts)

# Simulation parameters
time_step = 0.002
sim_time = 2.0 # Simulate for 2 seconds

# Main simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    chronoirr.drawGrid(vis, 0.2, 0.2, 10, 10, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), chrono.ChColor(0.3,0.3,0.3)) # Grid color
    vis.EndScene()
    sys.DoStepDynamics(time_step)

    if sys.GetChTime() > sim_time:
        vis.GetDevice().closeDevice() # Close window after sim_time

del sys # Explicitly delete system to free resources, especially MKL