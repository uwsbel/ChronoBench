import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemSMC()

mesh = fea.ChMesh()
sys.Add(mesh)

mesh.SetAutomaticGravity(True,
                         2)  # for max precision in gravity of FE, at least 2 integration points per element when using cubic IGA

# Changed gravity vector per instruction #4
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -3.71, 0))  # Mars gravity approx.

beam_L = 10  # Changed from 6 to 10 per instruction #1
beam_ro = 0.060  # Changed from 0.050 per instruction #2
beam_ri = 0.055  # Changed from 0.045 per instruction #2

CH_PI = 3.141592653589793  # Use proper Pi for accuracy

# Create a section, i.e. thickness and material properties
# for beams. This will be shared among some beams.

minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(7800)
minertia.SetArea(CH_PI * (pow(beam_ro, 2) - pow(beam_ri, 2)))
minertia.SetIyy((CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
minertia.SetIzz((CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(210e9)
melasticity.SetShearModulusFromPoisson(0.3)
melasticity.SetIyy((CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
melasticity.SetIzz((CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
melasticity.SetJ((CH_PI / 2.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))

msection = fea.ChBeamSectionCosserat(minertia, melasticity)

msection.SetCircular(True)
msection.SetDrawCircularRadius(beam_ro)  # SetAsCircularSection(..) would overwrite Ixx Iyy J etc.

# Use the ChBuilderBeamIGA tool for creating a straight rod
# divided in Nel elements:

builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(mesh,  # the mesh to put the elements in
                  msection,  # section of the beam
                  20,  # number of sections (spans)
                  chrono.ChVector3d(0, 0, 0),  # start point
                  chrono.ChVector3d(beam_L, 0, 0),  # end point
                  chrono.VECT_Y,  # suggested Y direction of section
                  1)  # order (3 = cubic, etc)

# Correct indexing and size call for node_mid:
nodes = builder.GetLastBeamNodes()
node_mid = nodes[m.floor(len(nodes) / 2)]

# Create the flywheel and attach it to the center of the beam

# Changed flywheel radius per instruction #3
mbodyflywheel = chrono.ChBodyEasyCylinder(
    0.30,  # R (changed from 0.24)
    0.1,  # h
    7800)  # density

# Note: ChBodyEasyCylinder constructor signature changed in recent PyChrono versions:
# The signature is ChBodyEasyCylinder(radius, height, density) by default along Z axis,
# so no axis param. The older code used ChAxis_Y (which is not a parameter here).
# This is a discrepancy in the original code.

# To align the cylinder axis with Y axis, we rotate the flywheel accordingly.

# Set coordinate system and rotation:
mbodyflywheel.SetPos(node_mid.GetPos() + chrono.ChVector3d(0, 0.05, 0))  # flywheel initial center (plus Y offset)

# Rotation: cylinder default axis is Z axis, want it along X axis as original code implies:
# The original code rotated by 90 deg around Z axis to align Y-axis cylinder along X-axis,
# but now easier to rotate from Z to X: rotate -90 deg about Y axis

mbodyflywheel.SetRot(chrono.ChQuaternionD().Q_from_AngAxis(CH_PI / 2.0, chrono.VECT_Y))  # rotate 90 deg around Y axis

sys.Add(mbodyflywheel)

myjoint = chrono.ChLinkMateFix()
myjoint.Initialize(node_mid, mbodyflywheel)
sys.Add(myjoint)

# Create the truss
truss = chrono.ChBody()
truss.SetBodyFixed(True)   # Correct method name from SetFixed to SetBodyFixed
sys.Add(truss)

# Create the end bearing
bearing = chrono.ChLinkMateGeneric(False, True, True, False, True, True)
bearing.Initialize(builder.GetLastBeamNodes()[-1],
                   truss,
                   chrono.ChFrameD(builder.GetLastBeamNodes()[-1].GetPos())
                   )
sys.Add(bearing)

# Create the motor that rotates the beam
rotmotor1 = chrono.ChLinkMotorRotationSpeed()

# Connect the rotor and the stator and add the motor to the system:
rotmotor1.Initialize(builder.GetLastBeamNodes()[0],  # body A (slave)
                     truss,  # body B (master)
                     chrono.ChFrameD(builder.GetLastBeamNodes()[0].GetPos(),
                                     chrono.ChQuaternionD().Q_from_AngAxis(CH_PI / 2.0, chrono.VECT_Y))
                     # motor frame, in abs. coords
                     )
sys.Add(rotmotor1)

# Changed motor function per instruction #5
f_ramp = chrono.ChFunctionSine(60, 0.1)
rotmotor1.SetMotorFunction(f_ramp)

# Attach a visualization of the FEM mesh.

mvisualizebeamA = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
mvisualizebeamA.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(mvisualizebeamA)

mvisualizebeamC = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
mvisualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
mvisualizebeamC.SetSymbolsThickness(0.006)
mvisualizebeamC.SetSymbolsScale(0.01)
mvisualizebeamC.SetZbufferHide(False)
mesh.AddVisualShapeFEA(mvisualizebeamC)

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Test FEA: the Jeffcott rotor with IGA beams')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()

# Changed camera position per instruction #6
vis.AddCamera(chrono.ChVector3d(0, 2, 8), chrono.ChVector3d(beam_L / 2, 0, 0))
vis.AddTypicalLights()

msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)

sys.DoStaticLinear()

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.002)