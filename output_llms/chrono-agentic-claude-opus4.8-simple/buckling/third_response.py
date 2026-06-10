import math as m                                                   # trig + constants for the motor ramp
import pychrono as chrono                                          # main PyChrono library
import pychrono.fea as fea                                         # finite element analysis module
import pychrono.pardisomkl as mklsolver                            # Pardiso MKL direct solver (stiff FEA)
import pychrono.irrlicht as chronicls                              # Irrlicht visualization module
import os                                                          # env-flag access for record mode

# Custom motor angle function: smooth cosine ramp to +/- pi, then hold
class ChFunctionMyFun(chrono.ChFunction):                          # subclass ChFunction (angle vs time)
    def __init__(self):
        chrono.ChFunction.__init__(self)                           # base ctor
    def GetVal(self, x):
        if x > 0.4:
            return -chrono.CH_PI                                   # hold at -pi after the ramp
        else:
            return -chrono.CH_PI * (1.0 - m.cos(chrono.CH_PI * x / 0.4)) / 2.0  # cosine ease-in ramp

# Create a Chrono physical system (SMC for stiff FEA beams)
sys = chrono.ChSystemSMC()                                         # fixed: ChSystemSMC (was ChSytemSMC)

# Key geometrical parameters of the buckling mechanism
L = 1                                                              # horizontal IGA beam length
H = 0.25                                                           # vertical drop
K = 0.05                                                           # crank offset
vA = chrono.ChVector3d(0, 0, 0)                                    # root of horizontal beam
vC = chrono.ChVector3d(L, 0, 0)                                    # tip of horizontal beam / top of vertical
vB = chrono.ChVector3d(L, -H, 0)                                   # bottom of vertical beam
vG = chrono.ChVector3d(L - K, -H, 0)                               # crank pivot
vd = chrono.ChVector3d(0, 0, 0.0001)                               # tiny z offset to break symmetry (buckling trigger)

# Truss body, fixed in space (anchors the horizontal beam and motor stator)
body_trss = chrono.ChBody()                                        # fixed reference body
body_trss.SetFixed(True)                                           # pin to world
sys.AddBody(body_trss)
boxtruss = chrono.ChVisualShapeBox(0.03, 0.25, 0.15)               # block decoration at the root
body_trss.AddVisualShape(boxtruss, chrono.ChFramed(chrono.ChVector3d(-0.01, 0, 0), chrono.QUNIT))

# Crank body, driven by the rotational motor at vG
body_crank = chrono.ChBody()                                       # rigid crank link
body_crank.SetPos((vC + vG) * 0.5)                                 # place between vC and vG
sys.AddBody(body_crank)
boxcrank = chrono.ChVisualShapeBox(K, 0.05, 0.03)                  # crank block decoration
body_crank.AddVisualShape(boxcrank)

# Rotational ANGLE motor between the fixed truss and the crank (drives the cyclic load)
motor = chrono.ChLinkMotorRotationAngle()                          # fixed: angle motor (was speed motor w/ torque fn)
motor.Initialize(body_trss, body_crank, chrono.ChFramed(vG))      # fixed: body_trss (was undefined body_truss)
myfun = ChFunctionMyFun()
motor.SetAngleFunction(myfun)                                      # fixed: SetAngleFunction (was SetTorqueFunction)
sys.Add(motor)

# FEM mesh container for all three beams
mesh = fea.ChMesh()

# Horizontal beam section (IGA Cosserat, thin rectangular -> buckles in bending)
beam_wy = 0.10                                                     # section width y
beam_wz = 0.01                                                     # section width z (thin -> low buckling load)
minertia = fea.ChInertiaCosseratSimple()                          # fixed: ChInertiaCosseratSimple (was ChIneritia...)
minertia.SetAsRectangularSection(beam_wy, beam_wz, 2700)          # rectangular + density (aluminium)
melasticity = fea.ChElasticityCosseratSimple()                    # elasticity model
melasticity.SetYoungModulus(73.0e9)                               # E for aluminium
melasticity.SetShearModulusFromPoisson(0.3)                       # G from Poisson nu
melasticity.SetAsRectangularSection(beam_wy, beam_wz)
msection1 = fea.ChBeamSectionCosserat(minertia, melasticity)      # fixed: ChBeamSectionCosserat (was ChMassSection...)
msection1.SetDrawThickness(beam_wy * 0.5, beam_wz)                # visual thickness

# Build the horizontal IGA beam (cubic, 32 spans) from vA to vC
builder_iga = fea.ChBuilderBeamIGA()
builder_iga.BuildBeam(mesh, msection1, 32, vA, vC, chrono.VECT_Y, 3)  # fixed: VECT_Y + 32 spans (canonical)
iga_nodes = builder_iga.GetLastBeamNodes()                        # keep strong ref (SWIG GC pitfall)
iga_nodes.front().SetFixed(True)                                  # fix the root node at vA
node_tip = iga_nodes.back()                                       # fixed: tip node (was hard-coded [65] out of range)
node_mid = iga_nodes[int(iga_nodes.size() / 2)]                   # mid node (safe index)

# Vertical beam section (Euler-Bernoulli, circular)
section2 = fea.ChBeamSectionEulerAdvanced()                       # fixed: ChBeamSectionEulerAdvanced (was AdvancedEuler)
hbeam_d = 0.024                                                   # vertical beam diameter
section2.SetDensity(2700)
section2.SetYoungModulus(73.0e9)
section2.SetShearModulusFromPoisson(0.3)
section2.SetRayleighDamping(0.000)
section2.SetAsCircularSection(hbeam_d)

# Build the vertical Euler beam (3 elements) from vC to vB
builderA = fea.ChBuilderBeamEuler()
builderA.BuildBeam(mesh, section2, 3, vC + vd, vB + vd, chrono.ChVector3d(1, 0, 0))
vert_nodes = builderA.GetLastBeamNodes()                          # keep strong ref
node_top = vert_nodes[0]                                          # fixed: top node is index 0 (at vC)
node_down = vert_nodes[-1]                                        # bottom node (at vB)

# Couple vertical-beam top to horizontal-beam tip (generic, selected DOFs)
constr_bb = chrono.ChLinkMateGeneric()
constr_bb.Initialize(node_top, node_tip, False, node_top.Frame(), node_top.Frame())
sys.Add(constr_bb)
constr_bb.SetConstrainedCoords(True, False, True, False, False, False)
sphereconstr2 = chrono.ChVisualShapeSphere(0.02)                  # joint marker
constr_bb.AddVisualShape(sphereconstr2)

# Crank beam section (Euler-Bernoulli, thicker circular)
section3 = fea.ChBeamSectionEulerAdvanced()                       # fixed: ChBeamSectionEulerAdvanced (was EulerAdvanced... ok, but unify)
crankbeam_d = 0.048                                              # crank beam diameter
section3.SetDensity(2700)
section3.SetYoungModulus(73.0e9)
section3.SetShearModulusFromPoisson(0.3)
section3.SetRayleighDamping(0.000)
section3.SetAsCircularSection(crankbeam_d)

# Build the crank Euler beam (3 elements) from vG to vB
builderB = fea.ChBuilderBeamEuler()                              # fixed: fea (was undefined module 'fe')
builderB.BuildBeam(mesh, section3, 3, vG + vd, vB + vd, chrono.ChVector3d(0, 1, 0))
crank_nodes = builderB.GetLastBeamNodes()                        # keep strong ref
node_crnkG = crank_nodes[0]                                      # crank node at vG
node_crankB = crank_nodes[-1]                                    # crank node at vB

# Constrain crank-beam G-node to the rigid crank body (prismatic, all DOF locked here)
constr_cbd = chrono.ChLinkMatePrismatic()
constr_cbd.Initialize(node_crnkG, body_crank, False, node_crnkG.Frame(), node_crnkG.Frame())
sys.Add(constr_cbd)
constr_cbd.SetConstrainedCoords(True, True, True, True, True, True)

# Constrain vertical-beam bottom to crank-beam bottom (generic, selected DOFs)
constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(node_down, node_crankB, False, node_crankB.Frame(), node_crankB.Frame())
sys.Add(constr_bc)
constr_bc.SetConstrainedCoords(True, True, False, True, True, False)
sphereconstr3 = chrono.ChVisualShapeSphere(0.01)                 # joint marker
constr_bc.AddVisualShape(sphereconstr3)

# Add the mesh to the system (FEA gravity off -> forced/buckling response only)
mesh.SetAutomaticGravity(False)                                  # canonical buckling: no self-weight gravity
sys.Add(mesh)

# FEA visualization shape 1: bending-moment Mx scalar field on the beams
mvisualizebeamA = chrono.ChVisualShapeFEA(mesh)                  # mesh is a ctor arg
mvisualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MX)
mvisualizebeamA.SetColorscaleMinMax(-500, 500)                  # moment color range
mvisualizebeamA.SetSmoothFaces(True)
mvisualizebeamA.SetWireframe(False)
mesh.AddVisualShapeFEA(mvisualizebeamA)

# FEA visualization shape 2: node coordinate-system glyphs
mvisualizebeamC = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
mvisualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
mvisualizebeamC.SetSymbolsThickness(0.006)
mvisualizebeamC.SetSymbolsScale(0.01)
mvisualizebeamC.SetZbufferHide(False)
mesh.AddVisualShapeFEA(mvisualizebeamC)

# Irrlicht visualization (Initialize FIRST, then scene elements; no grid)
vis = chronicls.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Beam buckling FEA')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.0, 0.6, -1.0))                 # canonical buckling camera eye
vis.AddTypicalLights()

# Direct sparse solver + HHT timestepper (stiff FEA beams need both)
custom_solver = mklsolver.ChSolverPardisoMKL()                  # fixed: Pardiso MKL direct solver
sys.SetSolver(custom_solver)
ts = chrono.ChTimestepperHHT(sys)                               # fixed: HHT (was EulerExplicit -> diverges on stiff FEA)
ts.SetStepControl(False)
sys.SetTimestepper(ts)

time_step = 0.001                                               # stiff-beam timestep
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))    # untagged render cadence

# Main simulation loop (real-time visual demo, render cadence batched)
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
