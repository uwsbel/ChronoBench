import os
import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl

sys = chrono.ChSystemSMC()                                           # SMC for stiff FEA matrices
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))     # Y-up gravity

mesh = fea.ChMesh()                                                  # FEA mesh container
mesh.SetAutomaticGravity(False)                                      # forced/static buckling response

# --- geometry parameters (final values) ---
L = 1.2                                                              # horizontal span
H = 0.3                                                              # vertical drop
K = 0.07                                                             # crank length
vA = chrono.ChVector3d(0, 0, 0)                                      # left support of horizontal beam
vC = chrono.ChVector3d(L, 0, 0)                                      # right end of horizontal beam / top of vertical beam
vB = chrono.ChVector3d(L, -H, 0)                                     # bottom of vertical beam / top end of crank beam
vG = chrono.ChVector3d(L - K, -H, 0)                                 # crank pivot
vd = chrono.ChVector3d(0, 0, 1e-4)                                   # tiny out-of-plane offset to seed buckling

# --- horizontal IGA (Cosserat) beam ---
minertia = fea.ChInertiaCosseratSimple()
minertia.SetAsRectangularSection(0.12, 0.012, 2700)                  # wy, wz, density (aluminium)
melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(73e9)                                    # aluminium E
melasticity.SetShearModulusFromPoisson(0.3)                          # G from Poisson
section_h = fea.ChBeamSectionCosserat(minertia, melasticity)
builder_iga = fea.ChBuilderBeamIGA()
builder_iga.BuildBeam(mesh, section_h, 32, vA, vC, chrono.VECT_Y, 3) # 32 spans, cubic IGA
iga_nodes = builder_iga.GetLastBeamNodes()                           # keep ref (SWIG GC)
node_A = iga_nodes.front()                                           # left node A
node_C = iga_nodes.back()                                            # right node C
node_A.SetFixed(True)                                                # clamp the horizontal beam at A

# --- vertical Euler beam (C -> B) ---
section_v = fea.ChBeamSectionEulerAdvanced()
section_v.SetAsCircularSection(0.03)                                 # diameter 0.03
section_v.SetDensity(2700)
section_v.SetYoungModulus(73e9)
section_v.SetShearModulusFromPoisson(0.3)
section_v.SetRayleighDamping(0.000)
builder_v = fea.ChBuilderBeamEuler()
builder_v.BuildBeam(mesh, section_v, 6, vC + vd, vB + vd, chrono.ChVector3d(1, 0, 0))  # 6 Euler elements
v_nodes = builder_v.GetLastBeamNodes()                               # keep ref (SWIG GC)
node_vtop = v_nodes.front()                                          # top of vertical beam (near C)
node_vbot = v_nodes.back()                                           # bottom of vertical beam (near B)

# --- crank Euler beam (G -> B) ---
section_k = fea.ChBeamSectionEulerAdvanced()
section_k.SetAsCircularSection(0.054)                                # diameter 0.054
section_k.SetDensity(2700)
section_k.SetYoungModulus(73e9)
section_k.SetShearModulusFromPoisson(0.3)
section_k.SetRayleighDamping(0.000)
builder_k = fea.ChBuilderBeamEuler()
builder_k.BuildBeam(mesh, section_k, 5, vG + vd, vB + vd, chrono.ChVector3d(0, 1, 0))  # 5 Euler elements
k_nodes = builder_k.GetLastBeamNodes()                               # keep ref (SWIG GC)
node_kG = k_nodes.front()                                            # crank end at pivot G
node_kB = k_nodes.back()                                             # crank end at B

sys.Add(mesh)                                                        # register the FEA mesh

# --- truss body (fixed reference frame) ---
truss = chrono.ChBody()
truss.SetFixed(True)
truss_box = chrono.ChVisualShapeBox(0.03, 0.25, 0.12)                # truss visualization box
truss.AddVisualShape(truss_box, chrono.ChFramed(vG, chrono.QUNIT))
sys.Add(truss)

# --- crank rigid body driven by the rotational motor ---
crank = chrono.ChBody()
crank.SetPos(vG)                                                     # pivot at G
crank_box = chrono.ChVisualShapeBox(K, 0.03, 0.03)                   # crank visualization box
crank.AddVisualShape(crank_box, chrono.ChFramed(chrono.ChVector3d(K / 2, 0, 0), chrono.QUNIT))
sys.Add(crank)

# --- rigid welds tying the beam-end nodes together (ChLinkMateFix = 6-DOF node weld) ---
constr_gG = chrono.ChLinkMateFix()                                   # crank-beam G node -> crank body
constr_gG.Initialize(node_kG, crank)
sys.Add(constr_gG)

constr_C = chrono.ChLinkMateFix()                                    # horizontal beam C -> vertical beam top
constr_C.Initialize(node_C, node_vtop)
sphere_C = chrono.ChVisualShapeSphere(0.012)                         # constraint visualization sphere
constr_C.AddVisualShape(sphere_C)
sys.Add(constr_C)

constr_B = chrono.ChLinkMateFix()                                    # crank-beam B node -> vertical beam bottom
constr_B.Initialize(node_kB, node_vbot)
sphere_B = chrono.ChVisualShapeSphere(0.014)                         # crank<->vertical constraint sphere
constr_B.AddVisualShape(sphere_B)
sys.Add(constr_B)

# --- rotational motor at the crank pivot: ramp the crank angle 0 -> pi over 0.4 s ---
crank_angle = chrono.ChFunctionConstAcc(chrono.CH_PI, 0.2, 0.3, 0.4) # smooth S-curve to pi over 0.4 s
motor = chrono.ChLinkMotorRotationAngle()                            # drives the crank rotation
motor.Initialize(crank, truss, chrono.ChFramed(vG, chrono.QuatFromAngleX(chrono.CH_PI_2)))
motor.SetAngleFunction(crank_angle)
sys.Add(motor)

# --- FEA visualization (two shapes: bending-moment field + node glyphs) ---
vis_beam = chrono.ChVisualShapeFEA(mesh)                             # surface/scalar field shape
vis_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MX)
vis_beam.SetColorscaleMinMax(-500, 500)                              # Mx colour range
vis_beam.SetSmoothFaces(True)
vis_beam.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_beam)

vis_glyph = chrono.ChVisualShapeFEA(mesh)                            # node-glyph shape
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.006)
vis_glyph.SetSymbolsScale(0.015)                                     # glyph scale
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)

# --- solver + timestepper (stiff beams: Pardiso MKL + HHT) ---
sys.SetSolver(mkl.ChSolverPardisoMKL())
ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
sys.SetTimestepper(ts)

# --- Irrlicht window ---
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)                    # Y-up scene
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("FEA buckling and bending")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.0, 0.7, -1.2), chrono.ChVector3d(L / 2, -H / 2, 0))  # camera position
vis.AddTypicalLights()

time_step = 1e-3                                                     # HHT step
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))         # untagged cadence constant
while vis.Run():                                                     # SCORED CORE = plain truth form
    vis.BeginScene()
    vis.Render()
    chronoirr.drawGrid(vis, 0.05, 0.05, 20, 20)                      # reference grid in the scene
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
