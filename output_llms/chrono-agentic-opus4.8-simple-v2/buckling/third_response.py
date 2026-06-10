import math as m                                # trig + CH_PI helpers
import os                                         # filesystem (review dir)
import pychrono as chrono                         # main PyChrono core
import pychrono.fea as fea                        # finite-element module
import pychrono.pardisomkl as mklsolver           # Pardiso direct solver
import pychrono.irrlicht as chronoirr             # Irrlicht visualization

# Custom motor angle function: ramps the crank rotation then holds at CH_PI
class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self):
        chrono.ChFunction.__init__(self)          # base ctor
    def GetVal(self, x):
        if x > 0.5:                               # after the ramp window
            return chrono.CH_PI                    # hold at pi
        else:                                      # smooth cosine ramp up
            return -chrono.CH_PI * (1.0 - m.cos(chrono.CH_PI * x / 0.3)) / 2.0

# Physical system — FEA uses SMC + a direct solver
sys = chrono.ChSystemSMC()

# Key geometric points of the buckling rig
L = 1.2                                            # horizontal beam length
H = 0.4                                            # vertical beam height
K = 0.07                                           # crank offset
vA = chrono.ChVector3d(0, 0, 0)                    # horizontal beam root
vC = chrono.ChVector3d(L, 0, 0)                    # horizontal beam tip
vB = chrono.ChVector3d(L, -H, 0)                   # vertical beam bottom
vG = chrono.ChVector3d(L - K, -H, 0)               # crank pivot
vd = chrono.ChVector3d(0, 0, 0.0001)               # tiny offset to break symmetry

# Fixed truss body anchoring the structure
body_trss = chrono.ChBody()
body_trss.SetFixed(True)                           # ground reference
sys.AddBody(body_trss)
boxtruss = chrono.ChVisualShapeBox(0.03, 0.25, 0.15)            # truss block
body_trss.AddVisualShape(boxtruss, chrono.ChFramed(chrono.ChVector3d(-0.01, 0, 0), chrono.QUNIT))

# Rotating crank body driven by the motor
body_crank = chrono.ChBody()
body_crank.SetPos((vC + vG) * 0.5)                 # midway between tip and pivot
sys.AddBody(body_crank)
boxcrank = chrono.ChVisualShapeBox(K, 0.05, 0.03)              # crank block
body_crank.AddVisualShape(boxcrank)

# Rotational-angle motor between truss and crank, driven by the custom function
motor = chrono.ChLinkMotorRotationAngle()
motor.Initialize(body_trss, body_crank, chrono.ChFramed(vG))    # frame at pivot vG
myfun = ChFunctionMyFun()                          # keep a strong ref
motor.SetAngleFunction(myfun)                      # imposed angle vs time
sys.Add(motor)

# FEA mesh container
mesh = fea.ChMesh()

# Horizontal IGA Cosserat beam cross-section (aluminium-like)
beam_wy = 0.12                                      # section width y
beam_wz = 0.15                                      # section width z
minertia = fea.ChInertiaCosseratSimple()
minertia.SetAsRectangularSection(beam_wy, beam_wz, 2700)        # rho = 2700
melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(72.0e9)                # E = 72 GPa
melasticity.SetShearModulusFromPoisson(0.35)       # G from nu = 0.35
melasticity.SetAsRectangularSection(beam_wy, beam_wz)
msection1 = fea.ChBeamSectionCosserat(minertia, melasticity)
msection1.SetDrawThickness(beam_wy * 0.5, beam_wz)             # render thickness

# Build the horizontal IGA beam (30 spans, cubic order)
builder_iga = fea.ChBuilderBeamIGA()
builder_iga.BuildBeam(mesh, msection1, 30, vA, vC, chrono.VECT_X, 3)
builder_iga.GetLastBeamNodes().front().SetFixed(True)          # fix the root node
iga_nodes = builder_iga.GetLastBeamNodes()         # keep strong ref (SWIG GC)
node_tip = iga_nodes[iga_nodes.size() - 1]         # tip node at vC
node_mid = iga_nodes[iga_nodes.size() // 2]        # mid-span node

# Vertical Euler beam cross-section (circular)
section2 = fea.ChBeamSectionEulerAdvanced()
hbeam_d = 0.05                                      # vertical beam diameter
section2.SetDensity(2500)                          # rho = 2500
section2.SetYoungModulus(75.0e9)                   # E = 75 GPa
section2.SetShearModulusFromPoisson(0.25)          # G from nu = 0.25
section2.SetRayleighDamping(0.000)                 # no damping
section2.SetAsCircularSection(hbeam_d)

# Build the vertical Euler beam from tip down to vB
builderA = fea.ChBuilderBeamEuler()
builderA.BuildBeam(mesh, section2, 10, vC + vd, vB + vd, chrono.ChVector3d(1, 0, 0))
a_nodes = builderA.GetLastBeamNodes()              # keep strong ref
node_top = a_nodes[1]                              # near the top
node_down = a_nodes[len(a_nodes) - 1]              # bottom node

# Constraint coupling the vertical-beam top to the horizontal-beam tip
constr_bb = chrono.ChLinkMateGeneric()
constr_bb.Initialize(node_top, node_tip, False, node_top.Frame(), node_top.Frame())
sys.Add(constr_bb)
constr_bb.SetConstrainedCoords(True, False, True, False, False, False)   # tx, tz only
sphereconstr2 = chrono.ChVisualShapeSphere(0.02)               # joint marker
constr_bb.AddVisualShape(sphereconstr2)

# Crank Euler beam cross-section (circular)
section3 = fea.ChBeamSectionEulerAdvanced()
crankbeam_d = 0.06                                  # crank beam diameter
section3.SetDensity(2800)                          # rho = 2800
section3.SetYoungModulus(75.0e9)                   # E = 75 GPa
section3.SetShearModulusFromPoisson(0.25)          # G from nu = 0.25
section3.SetRayleighDamping(0.000)                 # no damping
section3.SetAsCircularSection(crankbeam_d)

# Build the crank Euler beam from pivot vG to vB
builderB = fea.ChBuilderBeamEuler()
builderB.BuildBeam(mesh, section3, 4, vG + vd, vB + vd, chrono.ChVector3d(0, 1, 0))
b_nodes = builderB.GetLastBeamNodes()              # keep strong ref
node_crnkG = b_nodes[0]                             # at the pivot end
node_crankB = b_nodes[len(b_nodes) - 1]            # at the vertical-beam end

# Prismatic constraint tying the crank-beam pivot end to the crank body
constr_cbd = chrono.ChLinkMatePrismatic()
constr_cbd.Initialize(node_crnkG, body_crank, False, node_crnkG.Frame(), node_crnkG.Frame())
sys.Add(constr_cbd)
constr_cbd.SetConstrainedCoords(True, True, True, True, True, True)      # fully fixed

# Generic constraint coupling the vertical beam bottom to the crank-beam end
constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(node_down, node_crankB, False, node_crankB.Frame(), node_crankB.Frame())
sys.Add(constr_bc)
constr_bc.SetConstrainedCoords(True, True, False, True, True, False)
sphereconstr3 = chrono.ChVisualShapeSphere(0.01)               # joint marker
constr_bc.AddVisualShape(sphereconstr3)

# Register the mesh with gravity enabled
mesh.SetAutomaticGravity(True)
sys.Add(mesh)

# FEA visualization shape 1 — bending-moment My color field
mvisualizebeamA = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MY)
mvisualizebeamA.SetColorscaleMinMax(-400, 400)
mvisualizebeamA.SetSmoothFaces(False)
mvisualizebeamA.SetWireframe(False)
mesh.AddVisualShapeFEA(mvisualizebeamA)

# FEA visualization shape 2 — node coordinate-system glyphs
mvisualizebeamC = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
mvisualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
mvisualizebeamC.SetSymbolsThickness(0.005)
mvisualizebeamC.SetSymbolsScale(0.01)
mvisualizebeamC.SetZbufferHide(True)
mesh.AddVisualShapeFEA(mvisualizebeamC)

# Irrlicht visualization (Initialize first, then scene elements; no grid)
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Buckling Beam Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.0, 1.0, -1.5))
vis.AddTypicalLights()

# Direct solver for the stiff FEA matrices
custom_solver = mklsolver.ChSolverPardisoMKL()
sys.SetSolver(custom_solver)
# HHT timestepper — accurate, low numerical damping
ts = chrono.ChTimestepperHHT(sys)
ts.SetAlpha(-0.2)                                  # numerical damping for the stiff loop
ts.SetMaxIters(8)                                  # Newton iterations per step
ts.SetAbsTolerances(1e-6)                          # convergence tolerance
ts.SetStepControl(False)
ts.SetModifiedNewton(False)                        # full Newton — stabilizes the closed loop
sys.SetTimestepper(ts)

time_step = 0.002                                  # stiff-beam step
sim_end = 5.0                                       # total simulated time
render_fps = 50.0                                   # review cadence
render_every = max(1, round(1.0 / (render_fps * time_step)))   # untagged cadence
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
