import os
import math
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemSMC()                                            # FEA uses SMC
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -3.71, 0))      # lower-gravity environment (Y-up)

mesh = fea.ChMesh()                                                    # FEA mesh container
mesh.SetAutomaticGravity(True)                                         # let FEA add self-weight

beam_L = 10                                                            # rotor shaft length
beam_ro = 0.060                                                        # outer radius of the hollow shaft
beam_ri = 0.055                                                        # inner radius of the hollow shaft

# Geometric properties of the hollow circular cross-section
area = math.pi * (beam_ro ** 2 - beam_ri ** 2)                        # cross-section area
Iyy = (math.pi / 4.0) * (beam_ro ** 4 - beam_ri ** 4)                 # second moment about y
Izz = (math.pi / 4.0) * (beam_ro ** 4 - beam_ri ** 4)                 # second moment about z
J = Iyy + Izz                                                         # polar moment (torsion)

# Inertia part of the Cosserat section
minertia = fea.ChInertiaCosseratSimple()                             # lumped inertia model
minertia.SetDensity(7800)                                            # steel density
minertia.SetArea(area)                                               # section area
minertia.SetIyy(Iyy)                                                 # bending inertia y
minertia.SetIzz(Izz)                                                 # bending inertia z

# Elasticity part of the Cosserat section
melasticity = fea.ChElasticityCosseratSimple()                      # simple elasticity model
melasticity.SetYoungModulus(210e9)                                  # steel Young's modulus
melasticity.SetShearModulusFromPoisson(0.3)                         # derive G from Poisson ratio
melasticity.SetIyy(Iyy)                                              # bending inertia y
melasticity.SetIzz(Izz)                                              # bending inertia z
melasticity.SetJ(J)                                                 # torsion constant

msection = fea.ChBeamSectionCosserat(minertia, melasticity)         # combined Cosserat section
msection.SetCircular(True)                                          # circular profile
msection.SetDrawCircularRadius(beam_ro)                            # render radius (does not overwrite inertia)

# Build the rotor shaft as an IGA Cosserat beam from A to B
builder = fea.ChBuilderBeamIGA()                                    # IGA beam builder
builder.BuildBeam(mesh, msection,
                  20,                                              # number of spans
                  chrono.ChVector3d(0, 0, 0),                      # node A (shaft start)
                  chrono.ChVector3d(beam_L, 0, 0),                 # node B (shaft end)
                  chrono.VECT_Y,                                   # suggested section Y direction
                  3)                                              # cubic order

beam_nodes = builder.GetLastBeamNodes()                            # keep a strong ref (SWIG GC)
node_first = beam_nodes.front()                                    # driven end node
node_last = beam_nodes.back()                                      # supported end node
node_mid = beam_nodes[int(beam_nodes.size() / 2)]                 # mid-span node for the flywheel

sys.Add(mesh)                                                      # register the mesh

# Flywheel rigid body welded to the mid-span node
mbodyflywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.30, 0.1, 7800)   # radius, height, density
mbodyflywheel.SetCoordsys(chrono.ChCoordsysd(node_mid.GetPos(),
                          chrono.QuatFromAngleAxis(chrono.CH_PI / 2.0, chrono.ChVector3d(0, 0, 1))))  # align disc axis with shaft
sys.Add(mbodyflywheel)

# Rigidly weld the flywheel to the mid-span beam node
constr_flywheel = chrono.ChLinkMateFix()                          # 6-DOF weld
constr_flywheel.Initialize(node_mid, mbodyflywheel)              # node <-> flywheel body
sys.Add(constr_flywheel)

# Fixed truss bodies that act as the two bearings
truss = chrono.ChBody()                                          # static support truss
truss.SetFixed(True)                                            # truss does not move
sys.Add(truss)

# Motor that spins the driven end of the shaft about the X axis
rotmotor = chrono.ChLinkMotorRotationSpeed()                    # speed-controlled rotary motor
rotmotor.Initialize(node_first,                                # rotor node
                    truss,                                     # stator (truss)
                    chrono.ChFramed(node_first.GetPos(),
                                    chrono.QuatFromAngleAxis(chrono.CH_PI / 2.0, chrono.ChVector3d(0, 1, 0))))  # motor frame, Z->X
f_ramp = chrono.ChFunctionSine(60, 0.1)                        # angular-speed function: amplitude 60, freq 0.1
rotmotor.SetSpeedFunction(f_ramp)                             # drive speed = sine
sys.Add(rotmotor)

# Bearing at the far end: constrain translations + selected rotations, free spin about X
constr_bearing = chrono.ChLinkMateGeneric(True, True, True, False, True, True)   # tx,ty,tz, -,ry,rz constrained
constr_bearing.Initialize(node_last,                          # supported node
                          truss,                             # truss bearing
                          False,
                          node_last.Frame(),                 # frame on node
                          node_last.Frame())                 # frame on truss
sys.Add(constr_bearing)

# FEA visualization: surface (deformed shaft) + node coordinate glyphs
vis_surface = chrono.ChVisualShapeFEA(mesh)                  # mesh is a ctor arg in 9.0.0
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)   # plain deformed surface
vis_surface.SetColorscaleMinMax(-0.4, 0.4)                  # two scalars (lo, hi)
vis_surface.SetSmoothFaces(True)                           # smooth shading
vis_surface.SetWireframe(False)                            # solid surface
mesh.AddVisualShapeFEA(vis_surface)

vis_glyph = chrono.ChVisualShapeFEA(mesh)                  # node-glyph shape
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)   # coordinate triads
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)         # no scalar field on glyphs
vis_glyph.SetSymbolsThickness(0.006)                      # glyph line thickness
vis_glyph.SetSymbolsScale(0.01)                           # glyph size
vis_glyph.SetZbufferHide(False)                           # always show glyphs
mesh.AddVisualShapeFEA(vis_glyph)

# Direct sparse solver (Pardiso MKL) for the stiff beam stiffness matrix
sys.SetSolver(mkl.ChSolverPardisoMKL())                   # required for stiff FEA

# HHT timestepper (canonical-minimal) for the implicit dynamics
ts = chrono.ChTimestepperHHT(sys)                        # implicit HHT
ts.SetStepControl(False)                                # fixed-step (truth idiom)
sys.SetTimestepper(ts)

# Irrlicht visualization window (Initialize first, scene elements after)
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Jeffcott rotor")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2, 8), chrono.ChVector3d(beam_L / 2.0, 0, 0))   # raised, pulled-back view
vis.AddTypicalLights()

# Pre-solve static linear step to settle the rotor under gravity + motor load
sys.DoStaticLinear()

time_step = 0.002                                        # IGA rotor timestep
sim_end = 10.0                                           # simulation duration
render_fps = 50.0                                        # frames per second for review
render_every = max(1, round(1.0 / (render_fps * time_step)))   # untagged cadence constant
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
