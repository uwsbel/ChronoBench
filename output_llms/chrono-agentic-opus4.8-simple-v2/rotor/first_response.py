import os
import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl

sys = chrono.ChSystemSMC()                                            # FEA truths use SMC
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))      # Y-up world, g down

mesh = fea.ChMesh()                                                   # FEA container for the rotor shaft
mesh.SetAutomaticGravity(False)                                       # forced response — no FEA self-gravity

# --- IGA Cosserat shaft section (hollow circular rotor) ---
beam_L = 6.0                                                          # rotor shaft length (m)
beam_ro = 0.050                                                       # outer radius (m)
beam_ri = 0.045                                                       # inner radius (m)
density = 7800.0                                                      # steel density kg/m^3
area = chrono.CH_PI * (beam_ro**2 - beam_ri**2)                       # annular cross-section area
Iyy = (chrono.CH_PI / 4.0) * (beam_ro**4 - beam_ri**4)               # second moment of area
Izz = Iyy                                                             # symmetric circular section
J = Iyy + Izz                                                         # polar moment (torsion)

minertia = fea.ChInertiaCosseratSimple()                             # mass distribution of the section
minertia.SetDensity(density)                                         # rho
minertia.SetArea(area)                                               # A
minertia.SetIyy(Iyy)                                                 # bending inertia y
minertia.SetIzz(Izz)                                                 # bending inertia z

melasticity = fea.ChElasticityCosseratSimple()                       # stiffness of the section
melasticity.SetYoungModulus(210e9)                                   # E = 210 GPa (steel)
melasticity.SetShearModulusFromPoisson(0.3)                          # G from nu = 0.3
melasticity.SetArea(area)                                            # A
melasticity.SetIyy(Iyy)                                              # bending stiffness y
melasticity.SetIzz(Izz)                                              # bending stiffness z
melasticity.SetJ(J)                                                  # torsional constant

msection = fea.ChBeamSectionCosserat(minertia, melasticity)          # combine inertia + elasticity
msection.SetCircular(True)                                           # circular cross section
msection.SetDrawCircularRadius(beam_ro)                              # draw radius (does NOT overwrite I/J)

# --- Build the IGA beam from A(0,0,0) to B(beam_L,0,0) ---
builder = fea.ChBuilderBeamIGA()                                     # isogeometric beam builder
builder.BuildBeam(mesh, msection,
                  20,                                                # number of spans
                  chrono.ChVector3d(0, 0, 0),                        # A — motor-driven end
                  chrono.ChVector3d(beam_L, 0, 0),                   # B — free/bearing end
                  chrono.VECT_Y,                                     # suggested section Y direction
                  3)                                                 # order 3 = cubic IGA

beam_nodes = builder.GetLastBeamNodes()                              # keep strong ref (SWIG GC)
n_nodes = beam_nodes.size()                                          # node count along the shaft
node_A = beam_nodes.front()                                          # node at the driven end (x=0)
node_B = beam_nodes.back()                                           # node at the far end (x=L)
node_mid = beam_nodes[n_nodes // 2]                                  # node at the rotor center (x=L/2)

sys.Add(mesh)                                                        # register the FEA mesh

# --- Flywheel rigidly welded to the mid-span node (Jeffcott disk) ---
fly_r = 0.30                                                         # flywheel radius (m)
fly_h = 0.05                                                         # flywheel thickness (m)
fly_density = 7800.0                                                 # steel disk
flywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_X,               # cylinder axis along the shaft
                                     fly_r, fly_h, fly_density,
                                     True, False)                    # visualize, no collision
flywheel.SetPos(node_mid.GetPos())                                  # place at the mid-span node
flywheel.SetRot(node_mid.GetRot())                                  # align with the node frame
sys.Add(flywheel)                                                   # add the disk body

weld = chrono.ChLinkMateFix()                                       # rigid 6-DOF weld
weld.Initialize(node_mid, flywheel)                                # tie disk to the mid node
sys.Add(weld)

# --- Mass eccentricity (the Jeffcott signature: offset mass drives whirl) ---
ecc = 0.12                                                          # radial offset of the unbalance mass (m)
unbalance = chrono.ChBodyEasySphere(0.06, 9000.0, True, False)     # small dense unbalance ball
unbalance.SetPos(node_mid.GetPos() + chrono.ChVector3d(0, ecc, 0)) # offset radially on the disk
sys.Add(unbalance)
weld_ub = chrono.ChLinkMateFix()                                   # weld the ball to the disk
weld_ub.Initialize(unbalance, flywheel)                           # rotates with the flywheel
sys.Add(weld_ub)

# --- Fixed truss for the bearings ---
truss = chrono.ChBody()                                             # ground reference body
truss.SetFixed(True)                                                # immovable
sys.Add(truss)

# --- Bearing at the far end B: constrain translation + radial rotation, free spin about X ---
bearing = chrono.ChLinkMateGeneric(True, True, True,               # tx, ty, tz constrained
                                   False, True, True)              # rx free (spin), ry, rz constrained
bearing.Initialize(node_B, truss, False, node_B.Frame(), node_B.Frame())
sys.Add(bearing)

# --- Rotational motor driving the near end A about the shaft (X) axis ---
motor = chrono.ChLinkMotorRotationSpeed()                           # speed-controlled rotary motor
rot_x_to_z = chrono.QuatFromAngleY(-chrono.CH_PI_2)                 # motor spins about local Z -> align to X
motor.Initialize(node_A, truss,
                 chrono.ChFramed(node_A.GetPos(), rot_x_to_z))     # motor frame at node A, axis along X
motor.SetSpeedFunction(chrono.ChFunctionConst(4.0 * chrono.CH_PI)) # spin at 4*pi rad/s (~2 Hz)
sys.Add(motor)

# --- FEA visualization: surface field + node glyphs ---
vis_surface = chrono.ChVisualShapeFEA(mesh)                        # deformed surface shape
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
vis_surface.SetSmoothFaces(True)                                  # smooth shading
vis_surface.SetWireframe(False)                                   # solid surface
mesh.AddVisualShapeFEA(vis_surface)

vis_glyph = chrono.ChVisualShapeFEA(mesh)                         # node coordinate-system glyphs
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.006)                             # glyph line thickness
vis_glyph.SetSymbolsScale(0.01)                                  # glyph size
vis_glyph.SetZbufferHide(False)                                  # always draw glyphs
mesh.AddVisualShapeFEA(vis_glyph)

# --- Solver + timestepper for the stiff IGA beam ---
sys.SetSolver(mkl.ChSolverPardisoMKL())                          # direct solver for stiff stiffness matrix
ts = chrono.ChTimestepperHHT(sys)                                # implicit HHT integrator
ts.SetStepControl(False)                                         # canonical-minimal HHT
sys.SetTimestepper(ts)

sys.DoStaticLinear()                                             # settle the structure before dynamics

# --- Irrlicht visualization window ---
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)               # Y-up scene
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Jeffcott Rotor (IGA beam)")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(3, 1, -4), chrono.ChVector3d(3, 0, 0))
vis.AddTypicalLights()

time_step = 0.002                                              # IGA rotor timestep
sim_end = 10.0                                                 # simulation duration (s)
render_fps = 50.0                                             # review frames per second
render_every = max(1, round(1.0 / (render_fps * time_step)))  # untagged cadence constant
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
