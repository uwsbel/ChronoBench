"""
Beam buckling simulation with motor-driven crank mechanism.

Models a horizontal IGA beam subject to vertical buckling under motor-driven
crank loading. The crank is driven by a ChLinkMotorRotationAngle with a custom
angle function that ramps to pi. Includes FEA visualization with moment coloring.
"""

import math as m
import os
import csv
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as pardiso
import pychrono.irrlicht as chronoirr


# Custom function class for motor angle:
class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self):
        chrono.ChFunction.__init__(self)

    def GetVal(self, x):
        if x > 0.4:
            return chrono.CH_PI
        else:
            return -chrono.CH_PI * (1.0 - m.cos(chrono.CH_PI * x / 0.4)) / 2.0


# === System & gravity ===
sys = chrono.ChSystemSMC()

# FEA buckling: Y-up, no gravity (forced response)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

# Define key geometrical parameters
L = 1.2
H = 0.3
K = 0.07

vA = chrono.ChVector3d(0, 0, 0)
vC = chrono.ChVector3d(L, 0, 0)
vB = chrono.ChVector3d(L, -H, 0)
vG = chrono.ChVector3d(L - K, -H, 0)
vd = chrono.ChVector3d(0, 0, 0.0001)

# Create a truss body, fixed in space:
body_truss = chrono.ChBody()
body_truss.SetFixed(True)
sys.AddBody(body_truss)

# Attach a visualization shape to the truss
boxtruss = chrono.ChVisualShapeBox(0.03, 0.25, 0.12)
body_truss.AddVisualShape(boxtruss, chrono.ChFramed(chrono.ChVector3d(-0.01, 0, 0), chrono.QUNIT))

# Create a crank body:
body_crank = chrono.ChBody()
body_crank.SetPos((vB + vG) * 0.5)
sys.AddBody(body_crank)

# Attach a visualization shape to the crank
boxcrank = chrono.ChVisualShapeBox(K, 0.03, 0.03)
body_crank.AddVisualShape(boxcrank)

# Create a rotational motor (angle-controlled, not speed+torque)
motor = chrono.ChLinkMotorRotationAngle()
motor.Initialize(body_truss, body_crank, chrono.ChFramed(vG))
myfun = ChFunctionMyFun()
motor.SetAngleFunction(myfun)
sys.Add(motor)

# Create a FEM mesh container:
mesh = fea.ChMesh()

# Define horizontal beam parameters (IGA Cosserat beam)
beam_wy = 0.12
beam_wz = 0.012

# Create section properties for the IGA beam
minertia = fea.ChInertiaCosseratSimple()
minertia.SetAsRectangularSection(beam_wy, beam_wz, 2700)
melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(73.0e9)
melasticity.SetShearModulusFromPoisson(0.3)
melasticity.SetAsRectangularSection(beam_wy, beam_wz)
msection1 = fea.ChBeamSectionCosserat(minertia, melasticity)
msection1.SetDrawThickness(beam_wy, beam_wz)

# Build the IGA beam
builder_iga = fea.ChBuilderBeamIGA()
builder_iga.BuildBeam(mesh, msection1, 32, vA, vC, chrono.VECT_Y, 3)

# Fix the first node of the horizontal beam
builder_iga.GetLastBeamNodes().front().SetFixed(True)
node_tip = builder_iga.GetLastBeamNodes()[-1]
node_mid = builder_iga.GetLastBeamNodes()[17]

# Define vertical beam parameters using Euler beams
section2 = fea.ChBeamSectionEulerAdvanced()
hbeam_d = 0.03
section2.SetDensity(2700)
section2.SetYoungModulus(73.0e9)
section2.SetShearModulusFromPoisson(0.3)
section2.SetRayleighDamping(0.000)
section2.SetAsCircularSection(hbeam_d)

# Build the vertical beam with Euler elements
builderA = fea.ChBuilderBeamEuler()
builderA.BuildBeam(mesh, section2, 6, vC + vd, vB + vd, chrono.ChVector3d(1, 0, 0))

# Define nodes at the top and bottom of the vertical beam
node_top = builderA.GetLastBeamNodes()[0]
node_down = builderA.GetLastBeamNodes()[-1]

# Create a constraint between the horizontal and vertical beams
constr_bb = chrono.ChLinkMateGeneric()
constr_bb.Initialize(node_top, node_tip, False, node_top.Frame(), node_top.Frame())
sys.Add(constr_bb)
constr_bb.SetConstrainedCoords(True, True, True, False, False, False)

# Attach a visualization shape for the constraint
sphereconstr2 = chrono.ChVisualShapeSphere(0.012)
constr_bb.AddVisualShape(sphereconstr2)

# Create a crank beam
section3 = fea.ChBeamSectionEulerAdvanced()
crankbeam_d = 0.054
section3.SetDensity(2700)
section3.SetYoungModulus(73.0e9)
section3.SetShearModulusFromPoisson(0.3)
section3.SetRayleighDamping(0.000)
section3.SetAsCircularSection(crankbeam_d)

# Build the crank beam with Euler elements
builderB = fea.ChBuilderBeamEuler()
builderB.BuildBeam(mesh, section3, 5, vG + vd, vB + vd, chrono.ChVector3d(0, 1, 0))

# Define nodes at the ends of the crank beam
node_crankG = builderB.GetLastBeamNodes()[0]
node_crankB = builderB.GetLastBeamNodes()[-1]

# Create a constraint between the crank beam and the body crank
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
sphereconstr3 = chrono.ChVisualShapeSphere(0.014)
constr_bc.AddVisualShape(sphereconstr3)

# Final touches:
mesh.SetAutomaticGravity(False)
sys.Add(mesh)

# Create visualization for the FEM mesh:
mvisualizebeamA = chrono.ChVisualShapeFEA()
mvisualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MX)
mvisualizebeamA.SetColormapRange(-500, 500)
mvisualizebeamA.SetSmoothFaces(True)
mvisualizebeamA.SetWireframe(False)
mesh.AddVisualShapeFEA(mvisualizebeamA)

mvisualizebeamC = chrono.ChVisualShapeFEA()
mvisualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
mvisualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
mvisualizebeamC.SetSymbolsThickness(0.006)
mvisualizebeamC.SetSymbolsScale(0.015)
mvisualizebeamC.SetZbufferHide(False)
mesh.AddVisualShapeFEA(mvisualizebeamC)

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Beams and constraints')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.0, 0.7, -1.2))
vis.AddTypicalLights()
chronoirr.drawGrid(vis, 0.05, 0.05, 20, 20, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))

# Use a solver that can handle stiffness matrices
pardiso_solver = pardiso.ChSolverPardisoMKL()
sys.SetSolver(pardiso_solver)

# Use Euler implicit for stiff FEA beam
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT)

# === Main loop ===
time_step = 1e-3
sim_end = 3.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))


while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
