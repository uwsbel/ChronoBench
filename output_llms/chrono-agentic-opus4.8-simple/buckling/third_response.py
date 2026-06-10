import math as m                                   # trig functions and constants
import pychrono as chrono                            # main PyChrono library
import pychrono.fea as fea                           # finite element module
import pychrono.pardisomkl as pardiso                # Pardiso direct solver
import pychrono.irrlicht as chronoirr                # Irrlicht visualization


class ChFunctionMyFun(chrono.ChFunction):            # custom crank angle vs time
    def __init__(self):
        chrono.ChFunction.__init__(self)
    def GetVal(self, x):                             # x is the simulation time
        if x > 0.4:
            return chrono.CH_PI                       # hold at pi after 0.4 s
        else:
            return -chrono.CH_PI * (1.0 - m.cos(chrono.CH_PI * x / 0.4)) / 2.0


sys = chrono.ChSystemSMC()                           # SMC system for stiff FEA beams

L = 1.2                                              # horizontal beam length
H = 0.3                                              # vertical drop of crank pin
K = 0.07                                             # crank length
vA = chrono.ChVector3d(0, 0, 0)                      # root of horizontal beam
vC = chrono.ChVector3d(L, 0, 0)                      # tip of horizontal beam
vB = chrono.ChVector3d(L, -H, 0)                     # bottom of vertical beam
vG = chrono.ChVector3d(L - K, -H, 0)                 # crank pivot
vd = chrono.ChVector3d(0, 0, 0.0001)                 # tiny offset to avoid coincident nodes

body_truss = chrono.ChBody()                         # fixed truss
body_truss.SetFixed(True)
sys.AddBody(body_truss)

boxtruss = chrono.ChVisualShapeBox(0.03, 0.25, 0.12) # truss visual block
body_truss.AddVisualShape(boxtruss, chrono.ChFramed(chrono.ChVector3d(-0.01, 0, 0), chrono.QUNIT))

body_crank = chrono.ChBody()                         # rigid crank body
body_crank.SetPos((vB + vG) * 0.5)                   # centered between B and G
sys.AddBody(body_crank)

boxcrank = chrono.ChVisualShapeBox(K, 0.03, 0.03)    # crank visual block
body_crank.AddVisualShape(boxcrank)

motor = chrono.ChLinkMotorRotationAngle()            # angle-driven rotational motor
motor.Initialize(body_truss, body_crank, chrono.ChFramed(vG))   # pivot at crank G
myfun = ChFunctionMyFun()                            # the custom angle function
motor.SetAngleFunction(myfun)                        # drive the crank angle
sys.Add(motor)

mesh = fea.ChMesh()                                  # FEM mesh container

beam_wy = 0.12                                       # horizontal beam section width y
beam_wz = 0.012                                      # horizontal beam section width z

minertia = fea.ChInertiaCosseratSimple()             # IGA beam inertia
minertia.SetAsRectangularSection(beam_wy, beam_wz, 2700)       # rho = 2700 (aluminium)
melasticity = fea.ChElasticityCosseratSimple()       # IGA beam elasticity
melasticity.SetYoungModulus(73.0e9)                  # E = 73 GPa
melasticity.SetShearModulusFromPoisson(0.3)          # G from nu = 0.3
melasticity.SetAsRectangularSection(beam_wy, beam_wz)
msection1 = fea.ChBeamSectionCosserat(minertia, melasticity)   # combined Cosserat section
msection1.SetDrawThickness(beam_wy, beam_wz)         # draw cross-section size

builder_iga = fea.ChBuilderBeamIGA()                 # IGA (Cosserat) beam builder
builder_iga.BuildBeam(mesh, msection1, 32, vA, vC, chrono.VECT_Y, 3)   # 32 spans, cubic

builder_iga.GetLastBeamNodes().front().SetFixed(True)          # clamp the root node
iga_nodes = builder_iga.GetLastBeamNodes()           # keep a strong reference (SWIG GC)
node_tip = iga_nodes[iga_nodes.size() - 1]           # tip of the horizontal beam (at C)
node_mid = iga_nodes[17]                             # a node in the horizontal beam middle

section2 = fea.ChBeamSectionEulerAdvanced()          # vertical Euler beam section
hbeam_d = 0.03                                       # vertical beam diameter
section2.SetDensity(2700)                            # rho = 2700
section2.SetYoungModulus(73.0e9)                     # E = 73 GPa
section2.SetShearModulusFromPoisson(0.3)             # G from nu = 0.3
section2.SetRayleighDamping(0.000)                   # no damping
section2.SetAsCircularSection(hbeam_d)               # circular cross-section

builderA = fea.ChBuilderBeamEuler()                  # Euler beam builder (vertical)
builderA.BuildBeam(mesh, section2, 6, vC + vd, vB + vd, chrono.ChVector3d(1, 0, 0))   # C->B

a_nodes = builderA.GetLastBeamNodes()                # strong reference (SWIG GC)
node_top = a_nodes[0]                                # top of vertical beam (at C)
node_down = a_nodes[a_nodes.size() - 1]              # bottom of vertical beam (at B)

constr_bb = chrono.ChLinkMateGeneric()               # tie vertical top to horizontal tip
constr_bb.Initialize(node_top, node_tip, False, node_top.Frame(), node_top.Frame())
sys.Add(constr_bb)
constr_bb.SetConstrainedCoords(True, True, True, False, False, False)   # x, y, z

sphereconstr2 = chrono.ChVisualShapeSphere(0.012)    # constraint marker
constr_bb.AddVisualShape(sphereconstr2)

section3 = fea.ChBeamSectionEulerAdvanced()          # crank Euler beam section
crankbeam_d = 0.054                                  # crank beam diameter
section3.SetDensity(2700)                            # rho = 2700
section3.SetYoungModulus(73.0e9)                     # E = 73 GPa
section3.SetShearModulusFromPoisson(0.3)             # G from nu = 0.3
section3.SetRayleighDamping(0.000)                   # no damping
section3.SetAsCircularSection(crankbeam_d)           # circular cross-section

builderB = fea.ChBuilderBeamEuler()                  # Euler beam builder (crank)
builderB.BuildBeam(mesh, section3, 5, vG + vd, vB + vd, chrono.ChVector3d(0, 1, 0))   # G->B

b_nodes = builderB.GetLastBeamNodes()                # strong reference (SWIG GC)
node_crankG = b_nodes[0]                             # crank beam end at G
node_crankB = b_nodes[b_nodes.size() - 1]            # crank beam end at B

constr_cbd = chrono.ChLinkMateGeneric()              # weld crank beam end to crank body
constr_cbd.Initialize(node_crankG, body_crank, False, node_crankG.Frame(), node_crankG.Frame())
sys.Add(constr_cbd)
constr_cbd.SetConstrainedCoords(True, True, True, True, True, True)   # all 6 DOF

constr_bc = chrono.ChLinkMateGeneric()               # couple vertical bottom to crank end
constr_bc.Initialize(node_down, node_crankB, False, node_crankB.Frame(), node_crankB.Frame())
sys.Add(constr_bc)
constr_bc.SetConstrainedCoords(True, True, True, True, True, False)

sphereconstr3 = chrono.ChVisualShapeSphere(0.014)    # constraint marker
constr_bc.AddVisualShape(sphereconstr3)

mesh.SetAutomaticGravity(False)                      # disable FEA gravity (forced response)
sys.Add(mesh)                                        # register the mesh

mvisualizebeamA = chrono.ChVisualShapeFEA(mesh)      # bending moment Mx field
mvisualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MX)
mvisualizebeamA.SetColorscaleMinMax(-500, 500)       # color range for Mx
mvisualizebeamA.SetSmoothFaces(True)
mvisualizebeamA.SetWireframe(False)
mesh.AddVisualShapeFEA(mvisualizebeamA)

mvisualizebeamC = chrono.ChVisualShapeFEA(mesh)      # node coordinate-system glyphs
mvisualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
mvisualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
mvisualizebeamC.SetSymbolsThickness(0.006)
mvisualizebeamC.SetSymbolsScale(0.015)
mvisualizebeamC.SetZbufferHide(False)
mesh.AddVisualShapeFEA(mvisualizebeamC)

vis = chronoirr.ChVisualSystemIrrlicht()             # Irrlicht render window
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Beams and constraints')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.0, 0.7, -1.2))
vis.AddTypicalLights()

pardiso_solver = pardiso.ChSolverPardisoMKL()        # direct solver for stiff matrices
sys.SetSolver(pardiso_solver)
ts = chrono.ChTimestepperHHT(sys)                    # HHT for stiff implicit integration
ts.SetStepControl(False)
sys.SetTimestepper(ts)

time_step = 0.001                                    # integration step
sim_end = 5.0                                         # stop time
render_fps = 50.0                                    # frames per second for review
render_every = max(1, round(1.0 / (render_fps * time_step)))   # untagged cadence constant
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
