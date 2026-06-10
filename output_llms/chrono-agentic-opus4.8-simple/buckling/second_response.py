import os
import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl

sys = chrono.ChSystemSMC()                                           # FEA stiff-beam system
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))     # Y-up gravity

L = 1.2                                                              # overall span length
H = 0.3                                                              # vertical beam height
K = 0.07                                                             # crank length

# truss (fixed reference body)
truss = chrono.ChBody()                                              # ground truss body
truss.SetFixed(True)                                                # truss is immovable
truss_box = chrono.ChVisualShapeBox(0.03, 0.25, 0.12)               # truss visualization box
truss.AddVisualShape(truss_box, chrono.ChFramed(chrono.ChVector3d(-0.01, 0, 0)))  # offset behind origin
sys.Add(truss)                                                      # register truss

# crank body (driven rigid arm pivoting at the base of the crank beam)
crank = chrono.ChBody()                                             # rigid crank body
crank.SetPos(chrono.ChVector3d(0, 0, 0))                           # crank pivots about the origin
crank_box = chrono.ChVisualShapeBox(K, 0.03, 0.03)                # crank visualization box
crank.AddVisualShape(crank_box, chrono.ChFramed(chrono.ChVector3d(K / 2, 0, 0)))  # box centered on arm
sys.Add(crank)                                                     # register crank

# rotational motor that slowly oscillates the crank about the truss (Z axis)
motor = chrono.ChLinkMotorRotationSpeed()                           # constant-speed motor
motor.Initialize(crank, truss, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))  # motor frame at crank pivot
motor.SetMotorFunction(chrono.ChFunctionConst(chrono.CH_PI / 30.0))  # slow 6 deg/s rotation
sys.Add(motor)                                                     # register motor

mesh = fea.ChMesh()                                                # FEA mesh container
mesh.SetAutomaticGravity(False)                                   # forced response, gravity off on mesh
refs = {}                                                         # keep strong refs (SWIG GC)

# horizontal beam — rectangular Euler section, cantilevered from the truss
sec_h = fea.ChBeamSectionEulerAdvanced()                           # horizontal beam section
sec_h.SetAsRectangularSection(0.12, 0.012)                        # width Y = 0.12, width Z = 0.012
sec_h.SetDensity(2700)                                             # aluminium density
sec_h.SetYoungModulus(73e9)                                      # aluminium Young's modulus
sec_h.SetShearModulusFromPoisson(0.3)                            # G from Poisson ratio
sec_h.SetRayleighDamping(0.000)                                 # no Rayleigh damping
refs["sec_h"] = sec_h                                            # retain section

builder_h = fea.ChBuilderBeamEuler()                              # Euler beam builder (horizontal)
builder_h.BuildBeam(mesh, sec_h, 6,                              # 6 elements across the span
                    chrono.ChVector3d(0, H, 0),                  # A: at top, fixed to truss
                    chrono.ChVector3d(L, H, 0),                  # B: free tip of span
                    chrono.ChVector3d(0, 1, 0))                  # section Y reference up
h_nodes = builder_h.GetLastBeamNodes()                           # retain horizontal node container
refs["h_nodes"] = h_nodes                                        # keep strong ref
node_h_start = h_nodes.front()                                   # horizontal beam root node
node_h_end = h_nodes.back()                                      # horizontal beam tip node

# vertical beam — circular Euler section, hangs from the horizontal-beam tip
sec_v = fea.ChBeamSectionEulerAdvanced()                          # vertical beam section
sec_v.SetAsCircularSection(0.03)                                # circular diameter 0.03
sec_v.SetDensity(2700)                                           # aluminium density
sec_v.SetYoungModulus(73e9)                                     # aluminium Young's modulus
sec_v.SetShearModulusFromPoisson(0.3)                          # G from Poisson ratio
sec_v.SetRayleighDamping(0.000)                                # no Rayleigh damping
refs["sec_v"] = sec_v                                          # retain section

builder_v = fea.ChBuilderBeamEuler()                             # Euler beam builder (vertical)
builder_v.BuildBeam(mesh, sec_v, 6,                             # 6 Euler elements (updated)
                    chrono.ChVector3d(L, H, 0),                 # A: at horizontal-beam tip
                    chrono.ChVector3d(L, 0, 0),                 # B: down to base level
                    chrono.ChVector3d(1, 0, 0))                 # section Y reference
v_nodes = builder_v.GetLastBeamNodes()                          # retain vertical node container
refs["v_nodes"] = v_nodes                                       # keep strong ref
node_v_top = v_nodes.front()                                   # vertical beam upper node (at tip)
node_v_bottom = v_nodes.back()                                 # vertical beam lower node

# crank beam — circular Euler section, links the rotating crank to the vertical base
sec_k = fea.ChBeamSectionEulerAdvanced()                         # crank beam section
sec_k.SetAsCircularSection(0.054)                              # circular diameter 0.054 (updated)
sec_k.SetDensity(2700)                                          # aluminium density
sec_k.SetYoungModulus(73e9)                                   # aluminium Young's modulus
sec_k.SetShearModulusFromPoisson(0.3)                        # G from Poisson ratio
sec_k.SetRayleighDamping(0.000)                              # no Rayleigh damping
refs["sec_k"] = sec_k                                        # retain section

builder_k = fea.ChBuilderBeamEuler()                           # Euler beam builder (crank)
builder_k.BuildBeam(mesh, sec_k, 5,                           # 5 Euler elements (updated)
                    chrono.ChVector3d(K, 0, 0),               # A: at the crank arm tip
                    chrono.ChVector3d(L, 0, 0),               # B: at the vertical-beam base
                    chrono.ChVector3d(0, 1, 0))               # section Y reference
k_nodes = builder_k.GetLastBeamNodes()                        # retain crank-beam node container
refs["k_nodes"] = k_nodes                                     # keep strong ref
node_k_crank = k_nodes.front()                               # crank-beam node at the crank arm
node_k_base = k_nodes.back()                                # crank-beam node at the vertical base

sys.Add(mesh)                                                # register FEA mesh

# constraint: clamp horizontal-beam root to the fixed truss (cantilever support)
constr_root = chrono.ChLinkMateGeneric()                     # generic 6-DOF constraint
constr_root.Initialize(node_h_start, truss, False, node_h_start.Frame(), node_h_start.Frame())  # root <-> truss
constr_root.SetConstrainedCoords(True, True, True, True, True, True)  # all 6 DOF clamped
sys.Add(constr_root)                                        # register constraint

# constraint: tie crank-beam crank end to the rotating crank body (driving load)
constr_k_crank = chrono.ChLinkMateGeneric()                # crank-beam drive constraint
constr_k_crank.Initialize(node_k_crank, crank, False, node_k_crank.Frame(), node_k_crank.Frame())  # crank end <-> crank body
constr_k_crank.SetConstrainedCoords(True, True, True, True, True, True)  # all 6 DOF clamped
sys.Add(constr_k_crank)                                    # register constraint

# constraint: join crank-beam base to vertical-beam bottom
constr_kv = chrono.ChLinkMateGeneric()                     # crank-to-vertical coupling
constr_kv.Initialize(node_k_base, node_v_bottom, False, node_k_base.Frame(), node_k_base.Frame())  # crank base <-> vertical bottom
constr_kv.SetConstrainedCoords(True, True, True, True, True, True)  # all 6 DOF clamped
sys.Add(constr_kv)                                        # register constraint

# visualization marker spheres at the constraint locations
mark_root = chrono.ChVisualShapeSphere(0.012)              # root constraint marker (updated size)
truss.AddVisualShape(mark_root, chrono.ChFramed(chrono.ChVector3d(0, H, 0)))  # at the cantilever root
mark_kv = chrono.ChVisualShapeSphere(0.014)               # crank-vertical marker (updated size)
truss.AddVisualShape(mark_kv, chrono.ChFramed(chrono.ChVector3d(L, 0, 0)))   # at crank-beam/vertical junction

# FEA surface (bending-moment) visualization shape
vis_surface = chrono.ChVisualShapeFEA(mesh)               # scalar-field visual shape
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)  # bending moment Mz field
vis_surface.SetColorscaleMinMax(-0.4, 0.4)              # color range (lo, hi)
vis_surface.SetSmoothFaces(True)                       # smooth shaded faces
vis_surface.SetWireframe(False)                        # solid (not wireframe)
mesh.AddVisualShapeFEA(vis_surface)                    # register surface shape

# FEA glyph visualization shape (node coordinate systems)
vis_glyph = chrono.ChVisualShapeFEA(mesh)              # node glyph visual shape
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)  # coordinate-system triads
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)  # no scalar field on glyphs
vis_glyph.SetSymbolsThickness(0.006)                  # glyph line thickness
vis_glyph.SetSymbolsScale(0.015)                      # glyph scale (updated)
vis_glyph.SetZbufferHide(False)                       # always show glyphs
mesh.AddVisualShapeFEA(vis_glyph)                     # register glyph shape

# direct solver + HHT timestepper for stiff Euler beams
sys.SetSolver(mkl.ChSolverPardisoMKL())              # Pardiso MKL direct solver
ts = chrono.ChTimestepperHHT(sys)                    # HHT implicit timestepper
ts.SetStepControl(False)                             # canonical-minimal HHT form
sys.SetTimestepper(ts)                               # install timestepper

vis = chronoirr.ChVisualSystemIrrlicht()             # Irrlicht visualization system
vis.AttachSystem(sys)                                # bind the physical system
vis.SetWindowSize(1280, 720)                         # window resolution
vis.SetWindowTitle("Buckling beam mechanism")        # window title
vis.Initialize()                                     # create device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # PyChrono logo
vis.AddSkyBox()                                       # sky box
vis.AddCamera(chrono.ChVector3d(0.0, 0.7, -1.2), chrono.ChVector3d(0.6, 0.15, 0.0))  # camera eye + target
vis.AddTypicalLights()                               # standard two-light setup

time_step = 1e-3                                      # stiff-beam time step
sim_end = 8.0                                         # simulation end time
render_fps = 50.0                                     # review frame rate
render_every = max(1, round(1.0 / (render_fps * time_step)))         # untagged cadence constant
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
