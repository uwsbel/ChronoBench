import math as m  # math for trig functions
import os  # os for directory ops
import pychrono as chrono  # main PyChrono library
import pychrono.fea as fea  # FEA module
import pychrono.pardisomkl as pardiso  # Pardiso direct solver
import pychrono.irrlicht as chronoirr  # Irrlicht visualization


# Custom motor angle function
class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self):
        chrono.ChFunction.__init__(self)  # call base constructor

    def GetVal(self, x):
        if x > 0.4:  # after 0.4 s: full PI rotation
            return chrono.CH_PI
        else:
            return -chrono.CH_PI * (1.0 - m.cos(chrono.CH_PI * x / 0.4)) / 2.0  # smooth ramp


# Output directory for beam buckling
out_dir = chrono.GetChronoOutputPath() + "BEAM_BUCKLING"

# Create the physical system
sys = chrono.ChSystemSMC()  # SMC for FEA stiff beams

# Geometry key points
L = 1    # beam length (m)
H = 0.25  # height drop (m)
K = 0.05  # crank arm length (m)
vA = chrono.ChVector3d(0, 0, 0)          # beam start
vC = chrono.ChVector3d(L, 0, 0)          # beam end / vertical top
vB = chrono.ChVector3d(L, -H, 0)         # vertical bottom / crank end
vG = chrono.ChVector3d(L - K, -H, 0)    # crank pivot on truss
vd = chrono.ChVector3d(0, 0, 0.0001)    # tiny offset to avoid node coincidence

# Fixed truss body
body_truss = chrono.ChBody()
body_truss.SetFixed(True)  # immobile ground reference
sys.AddBody(body_truss)

# Visualization box on the truss
boxtruss = chrono.ChVisualShapeBox(0.02, 0.2, 0.1)
body_truss.AddVisualShape(boxtruss, chrono.ChFramed(chrono.ChVector3d(-0.01, 0, 0), chrono.QUNIT))

# Crank body driven by the motor
body_crank = chrono.ChBody()
body_crank.SetPos((vB + vG) * 0.5)  # center between B and G
sys.AddBody(body_crank)

# Visualization box on the crank
boxcrank = chrono.ChVisualShapeBox(K, 0.02, 0.02)
body_crank.AddVisualShape(boxcrank)

# Rotational motor: truss drives crank via prescribed angle function
motor = chrono.ChLinkMotorRotationAngle()
motor.Initialize(body_truss, body_crank, chrono.ChFramed(vG))  # pivot at vG
myfun = ChFunctionMyFun()  # custom ramp function
motor.SetAngleFunction(myfun)  # prescribe motor angle
sys.Add(motor)

# FEA mesh container
mesh = fea.ChMesh()

# Horizontal IGA (Cosserat) beam: A -> C
beam_wy = 0.10   # width in Y (m)
beam_wz = 0.01   # width in Z (m)

minertia = fea.ChInertiaCosseratSimple()
minertia.SetAsRectangularSection(beam_wy, beam_wz, 2700)  # aluminium density

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(73.0e9)  # Al Young's modulus
melasticity.SetShearModulusFromPoisson(0.3)  # derive G from Poisson nu
melasticity.SetAsRectangularSection(beam_wy, beam_wz)  # section dims

msection1 = fea.ChBeamSectionCosserat(minertia, melasticity)  # combine inertia + elasticity
msection1.SetDrawThickness(beam_wy, beam_wz)  # rendering thickness

builder_iga = fea.ChBuilderBeamIGA()
builder_iga.BuildBeam(mesh, msection1, 32, vA, vC, chrono.VECT_Y, 3)  # 32 spans, cubic order

builder_iga.GetLastBeamNodes().front().SetFixed(True)  # clamp beam at A
iga_nodes = builder_iga.GetLastBeamNodes()  # keep strong ref (SWIG GC safety)
node_tip = iga_nodes[-1]   # tip node at C
node_mid = iga_nodes[17]   # mid-beam node

# Vertical Euler beam: C -> B
section2 = fea.ChBeamSectionEulerAdvanced()
hbeam_d = 0.024  # circular diameter (m)
section2.SetDensity(2700)  # aluminium density
section2.SetYoungModulus(73.0e9)  # Young's modulus
section2.SetShearModulusFromPoisson(0.3)  # shear from Poisson
section2.SetRayleighDamping(0.000)  # no Rayleigh damping
section2.SetAsCircularSection(hbeam_d)  # set circular cross-section

builderA = fea.ChBuilderBeamEuler()
builderA.BuildBeam(mesh, section2, 3, vC + vd, vB + vd, chrono.ChVector3d(1, 0, 0))  # 3 elements

euler_nodesA = builderA.GetLastBeamNodes()  # keep strong ref
node_top = euler_nodesA[0]    # node at top (C end)
node_down = euler_nodesA[-1]  # node at bottom (B end)

# Constraint: horizontal tip -> vertical top (translational only)
constr_bb = chrono.ChLinkMateGeneric()
constr_bb.Initialize(node_top, node_tip, False, node_top.Frame(), node_top.Frame())
sys.Add(constr_bb)
constr_bb.SetConstrainedCoords(True, True, True, False, False, False)  # xyz constrained, rot free

sphereconstr2 = chrono.ChVisualShapeSphere(0.01)
constr_bb.AddVisualShape(sphereconstr2)  # marker at constraint

# Crank Euler beam: G -> B
section3 = fea.ChBeamSectionEulerAdvanced()
crankbeam_d = 0.048  # larger diameter for crank stiffness
section3.SetDensity(2700)
section3.SetYoungModulus(73.0e9)
section3.SetShearModulusFromPoisson(0.3)
section3.SetRayleighDamping(0.000)
section3.SetAsCircularSection(crankbeam_d)

builderB = fea.ChBuilderBeamEuler()
builderB.BuildBeam(mesh, section3, 3, vG + vd, vB + vd, chrono.ChVector3d(0, 1, 0))  # up-vector Y

euler_nodesB = builderB.GetLastBeamNodes()  # strong ref
node_crankG = euler_nodesB[0]   # node at G (crank side)
node_crankB = euler_nodesB[-1]  # node at B (bottom)

# Constraint: crank beam G node -> rigid crank body (all 6 DOF)
constr_cbd = chrono.ChLinkMateGeneric()
constr_cbd.Initialize(node_crankG, body_crank, False, node_crankG.Frame(), node_crankG.Frame())
sys.Add(constr_cbd)
constr_cbd.SetConstrainedCoords(True, True, True, True, True, True)  # fully fixed

# Constraint: vertical bottom -> crank beam B (5 DOF, free twist)
constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(node_down, node_crankB, False, node_crankB.Frame(), node_crankB.Frame())
sys.Add(constr_bc)
constr_bc.SetConstrainedCoords(True, True, True, True, True, False)  # rot Z free

sphereconstr3 = chrono.ChVisualShapeSphere(0.01)
constr_bc.AddVisualShape(sphereconstr3)  # marker at B

# Disable automatic gravity on FEA mesh (forced buckling, not gravity-driven)
mesh.SetAutomaticGravity(False)

sys.Add(mesh)  # register mesh in system

# FEA visualization: beam moment Mx field
mvisualizebeamA = chrono.ChVisualShapeFEA(mesh)  # mesh as ctor arg (9.0.0)
mvisualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MX)  # axial moment
mvisualizebeamA.SetColorscaleMinMax(-500, 500)  # color range
mvisualizebeamA.SetSmoothFaces(True)
mvisualizebeamA.SetWireframe(False)
mesh.AddVisualShapeFEA(mvisualizebeamA)

# FEA visualization: node coordinate system glyphs
mvisualizebeamC = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)  # node triads
mvisualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
mvisualizebeamC.SetSymbolsThickness(0.006)
mvisualizebeamC.SetSymbolsScale(0.01)
mvisualizebeamC.SetZbufferHide(False)
mesh.AddVisualShapeFEA(mvisualizebeamC)

# Irrlicht visualization window (Initialize FIRST, then scene elements)
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Beams and constraints')
vis.Initialize()  # create Irrlicht device
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.0, 0.6, -1.0))  # wide view looking at beam
vis.AddTypicalLights()

# Pardiso MKL direct solver for stiff FEA matrices
pardiso_solver = pardiso.ChSolverPardisoMKL()
sys.SetSolver(pardiso_solver)

# HHT timestepper (canonical-minimal form for buckling beams)
ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)  # fixed step, no adaptive control
sys.SetTimestepper(ts)

# Simulation parameters
time_step = 0.001   # 1 ms step for stiff FEA
sim_end = 10.0      # 10 s simulation
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # physics steps per render frame


while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)  # advance one FEA step
        if sys.GetChTime() >= sim_end:
            break
