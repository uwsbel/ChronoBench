import os
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

# ---- geometry parameters (final desired values) ----
L = 1.2                                          # horizontal beam length
H = 0.3                                          # vertical beam height
K = 0.07                                         # crank length

sys = chrono.ChSystemSMC()                       # FEA scenes use SMC
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))   # Y-up gravity

# ---- truss (fixed reference body) ----
truss = chrono.ChBody()                          # fixed structural truss
truss.SetFixed(True)                             # ground reference
truss.AddVisualShape(chrono.ChVisualShapeBox(0.03, 0.25, 0.12),   # truss marker box
                     chrono.ChFramed(chrono.ChVector3d(-0.01, 0, 0), chrono.QUNIT))
sys.Add(truss)

# ---- crank body (rotates to push the vertical beam) ----
crank = chrono.ChBody()                          # rigid crank body
crank.SetPos(chrono.ChVector3d(-K, -H, 0))       # at base, offset by crank length
crank.AddVisualShape(chrono.ChVisualShapeBox(K, 0.03, 0.03),       # crank marker box
                     chrono.ChFramed(chrono.ChVector3d(K / 2, 0, 0), chrono.QUNIT))
sys.Add(crank)

# ---- FEA mesh ----
mesh = fea.ChMesh()                              # container for all beams
mesh.SetAutomaticGravity(False)                  # static/forced buckling response
sys.Add(mesh)

# ---- horizontal beam (Euler-Bernoulli, rectangular section) ----
section_h = fea.ChBeamSectionEulerAdvanced()     # Euler rectangular section
section_h.SetAsRectangularSection(0.12, 0.012)   # width Y x width Z
section_h.SetYoungModulus(0.02e10)               # stiffness
section_h.SetShearModulusFromPoisson(0.3)        # derive G from Poisson nu
section_h.SetRayleighDamping(0.0001)             # light damping
section_h.SetDensity(1000)                       # kg/m^3

builder_h = fea.ChBuilderBeamEuler()             # Euler beam builder
builder_h.BuildBeam(mesh, section_h, 16,         # number of Euler elements
                    chrono.ChVector3d(0, 0, 0),  # A: left end at origin
                    chrono.ChVector3d(L, 0, 0),  # B: right end
                    chrono.ChVector3d(0, 1, 0))  # section Y up
nodes_h = builder_h.GetLastBeamNodes()           # keep container ref (SWIG GC)
node_hA = nodes_h.front()                         # left tip of horizontal beam
node_hB = nodes_h.back()                          # right tip of horizontal beam

# ---- vertical beam (Euler circular section) ----
section_v = fea.ChBeamSectionEulerAdvanced()     # vertical circular section
section_v.SetAsCircularSection(0.03)             # circular diameter
section_v.SetYoungModulus(0.02e10)               # stiffness
section_v.SetShearModulusFromPoisson(0.3)        # G from Poisson
section_v.SetRayleighDamping(0.0001)             # light damping
section_v.SetDensity(1000)                        # kg/m^3

builder_v = fea.ChBuilderBeamEuler()             # Euler beam builder
builder_v.BuildBeam(mesh, section_v, 6,          # 6 Euler elements for vertical beam
                    chrono.ChVector3d(0, 0, 0),  # A: top (joins horizontal at origin)
                    chrono.ChVector3d(0, -H, 0), # B: bottom of vertical beam
                    chrono.ChVector3d(1, 0, 0))  # section reference X
nodes_v = builder_v.GetLastBeamNodes()           # keep container ref (SWIG GC)
node_vA = nodes_v.front()                         # top node of vertical beam
node_vB = nodes_v.back()                          # bottom node of vertical beam

# ---- crank beam (Euler circular section) ----
section_c = fea.ChBeamSectionEulerAdvanced()     # crank circular section
section_c.SetAsCircularSection(0.054)            # circular diameter
section_c.SetYoungModulus(0.02e10)               # stiffness
section_c.SetShearModulusFromPoisson(0.3)        # G from Poisson
section_c.SetRayleighDamping(0.0001)             # light damping
section_c.SetDensity(1000)                        # kg/m^3

builder_c = fea.ChBuilderBeamEuler()             # Euler beam builder
builder_c.BuildBeam(mesh, section_c, 5,          # 5 Euler elements for crank beam
                    chrono.ChVector3d(-K, -H, 0),# A: crank pivot at base
                    chrono.ChVector3d(0, -H, 0), # B: joins vertical beam bottom
                    chrono.ChVector3d(0, 1, 0))  # section Y up
nodes_c = builder_c.GetLastBeamNodes()           # keep container ref (SWIG GC)
node_cA = nodes_c.front()                         # crank-side node
node_cB = nodes_c.back()                          # node shared with vertical bottom

# ---- constraints joining the structure ----
# horizontal beam left end fixed to truss (all DOF)
constr_truss = chrono.ChLinkMateGeneric()        # truss anchor
constr_truss.Initialize(node_hA, truss, chrono.ChFramed(node_hA.GetPos()))
constr_truss.SetConstrainedCoords(True, True, True, True, True, True)   # all 6 DOF
sys.Add(constr_truss)

# horizontal-beam right end joined to vertical-beam top
constr_hv = chrono.ChLinkMateGeneric()           # horizontal->vertical joint
constr_hv.Initialize(node_hB, node_vA, chrono.ChFramed(node_hB.GetPos()))
constr_hv.SetConstrainedCoords(True, True, True, True, True, True)      # all 6 DOF
sys.Add(constr_hv)

# crank beam end fixed to the rigid crank body
constr_crank = chrono.ChLinkMateFix()            # crank beam -> crank body weld
constr_crank.Initialize(node_cA, crank)
sys.Add(constr_crank)

# crank-beam end joined to vertical-beam bottom (spherical-style coupling)
constr_cv = chrono.ChLinkMateGeneric()           # crank->vertical bottom joint
constr_cv.Initialize(node_cB, node_vB, chrono.ChFramed(node_cB.GetPos()))
constr_cv.SetConstrainedCoords(True, True, True, False, False, False)   # translation only
sys.Add(constr_cv)


# ---- motor function: drive the crank to load the column ----
crank_angle = chrono.ChFunctionSine(0.25 * chrono.CH_PI, 0.2)   # amplitude, frequency
motor = chrono.ChLinkMotorRotationAngle()        # angle motor on the crank pivot
motor.Initialize(crank, truss, chrono.ChFramed(chrono.ChVector3d(-K, -H, 0)))
motor.SetAngleFunction(crank_angle)              # prescribed angle profile
sys.Add(motor)

# ---- constraint visualization spheres ----
sph_truss = chrono.ChVisualShapeSphere(0.012)    # constraint marker sphere
truss.AddVisualShape(sph_truss, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
sph_cv = chrono.ChVisualShapeSphere(0.014)       # crank<->vertical constraint marker
crank.AddVisualShape(sph_cv, chrono.ChFramed(chrono.ChVector3d(K, 0, 0), chrono.QUNIT))

# ---- FEA visualization: surface scalar field + node glyphs ----
vis_beam = chrono.ChVisualShapeFEA(mesh)         # bending-moment coloured field
vis_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_beam.SetColorscaleMinMax(-1.4, 1.4)          # moment colour range (lo, hi)
vis_beam.SetSmoothFaces(True)                    # smooth shading
vis_beam.SetWireframe(False)                     # solid surface
mesh.AddVisualShapeFEA(vis_beam)

vis_glyph = chrono.ChVisualShapeFEA(mesh)        # node coordinate-system glyphs
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.006)             # glyph line thickness
vis_glyph.SetSymbolsScale(0.015)                 # glyph scale
vis_glyph.SetZbufferHide(False)                  # always draw glyphs
mesh.AddVisualShapeFEA(vis_glyph)

# ---- solver + timestepper for stiff Euler beams ----
sys.SetSolver(mkl.ChSolverPardisoMKL())          # direct solver for stiff beams
ts = chrono.ChTimestepperHHT(sys)                # implicit HHT timestepper
ts.SetStepControl(False)                         # canonical-minimal form
sys.SetTimestepper(ts)

# ---- Irrlicht visualization ----
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Beam buckling and constraints")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.0, 0.7, -1.2), chrono.ChVector3d(0, 0, 0))   # camera eye/target
vis.AddTypicalLights()

# ---- time-stepping loop ----
time_step = 0.001                                # stiff-beam timestep
sim_end = 5.0                                    # simulation duration
render_fps = 50.0                                # review frame rate
render_every = max(1, round(1.0 / (render_fps * time_step)))         # untagged cadence constant
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
