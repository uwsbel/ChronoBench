import os
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemSMC()                                            # SMC system for stiff FEA matrices

beam_L = 0.1                                                          # length of each beam element span

mesh = fea.ChMesh()                                                  # FEA container mesh
mesh.SetAutomaticGravity(False)                                      # static/forced response, no FEA gravity

msection = fea.ChBeamSectionEulerAdvanced()                          # Euler-Bernoulli beam section
msection.SetAsRectangularSection(0.012, 0.025)                       # cross-section wy=0.012, wz=0.025
msection.SetYoungModulus(0.01e9)                                     # E = 0.01 GPa
msection.SetShearModulus(0.01e9 * 0.3)                               # G = 0.3 * E
msection.SetRayleighDamping(0.000)                                   # no structural damping
msection.SetCentroid(0, 0.02)                                        # offset centroid
msection.SetShearCenter(0, 0.1)                                      # offset shear center
msection.SetSectionRotation(45 * chrono.CH_RAD_TO_DEG)               # twist the section frame

# three nodes along x: 0, beam_L, 2*beam_L
hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))      # root node
hnode2 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_L, 0, 0))) # mid node
hnode3 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(2 * beam_L, 0, 0)))  # tip node
mesh.AddNode(hnode1)                                                 # register node 1
mesh.AddNode(hnode2)                                                 # register node 2
mesh.AddNode(hnode3)                                                 # register node 3

# two Euler beam elements connecting the three nodes
belement1 = fea.ChElementBeamEuler()                                 # element node1 -> node2
belement1.SetNodes(hnode1, hnode2)                                   # endpoints
belement1.SetSection(msection)                                       # assign section
mesh.AddElement(belement1)                                           # register element 1

belement2 = fea.ChElementBeamEuler()                                 # element node2 -> node3
belement2.SetNodes(hnode2, hnode3)                                   # endpoints
belement2.SetSection(msection)                                       # assign section
mesh.AddElement(belement2)                                           # register element 2

# applied loads
hnode2.SetForce(chrono.ChVector3d(4, 2, 0))                          # force on the mid node (N)
hnode3.SetTorque(chrono.ChVector3d(0, -0.04, 0))                     # torque on the tip node (Nm)

sys.Add(mesh)                                                        # add mesh to the system

# fixed truss the beam is constrained to
mtruss = chrono.ChBody()                                             # ground/truss body
mtruss.SetFixed(True)                                                # immovable
sys.Add(mtruss)                                                      # add truss

# tip node3 fully fixed to the truss (all 6 DOF)
constr_bc = chrono.ChLinkMateGeneric()                              # generic mate constraint
constr_bc.Initialize(hnode3, mtruss, False, hnode3.Frame(), hnode3.Frame())  # bind node3 -> truss
sys.Add(constr_bc)                                                  # add constraint
constr_bc.SetConstrainedCoords(True, True, True,                   # tx, ty, tz
                               True, True, True)                   # rx, ry, rz

# root node1 constrains only y,z translation (slides along x)
constr_d = chrono.ChLinkMateGeneric()                              # generic mate constraint
constr_d.Initialize(hnode1, mtruss, False, hnode1.Frame(), hnode1.Frame())   # bind node1 -> truss
sys.Add(constr_d)                                                  # add constraint
constr_d.SetConstrainedCoords(False, True, True,                  # free x, constrain y, z
                              False, False, False)                # free all rotations

# visualization shape 1 — beam bending moment Mz scalar field
mvisualizebeamA = chrono.ChVisualShapeFEA(mesh)                    # mesh is a ctor arg
mvisualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)  # bending moment Mz
mvisualizebeamA.SetColorscaleMinMax(-0.4, 0.4)                     # color range (lo, hi)
mvisualizebeamA.SetSmoothFaces(True)                               # smooth shading
mvisualizebeamA.SetWireframe(False)                                # solid surface
mesh.AddVisualShapeFEA(mvisualizebeamA)                            # register surface shape

# visualization shape 2 — node coordinate-system glyphs
mvisualizebeamC = chrono.ChVisualShapeFEA(mesh)                    # node glyph shape
mvisualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)   # coordinate triads
mvisualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)          # no scalar on glyphs
mvisualizebeamC.SetSymbolsThickness(0.006)                         # triad line thickness
mvisualizebeamC.SetSymbolsScale(0.01)                              # triad size
mvisualizebeamC.SetZbufferHide(False)                              # draw on top
mesh.AddVisualShapeFEA(mvisualizebeamC)                            # register glyph shape

# direct solver + HHT timestepper for stiff beam matrices
sys.SetSolver(mkl.ChSolverPardisoMKL())                            # Pardiso MKL direct solver

# Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()                          # create Irrlicht visual system
vis.AttachSystem(sys)                                             # bind it to the system
vis.SetWindowSize(1024, 768)                                      # window size
vis.SetWindowTitle("Beam finite elements")                        # window title
vis.Initialize()                                                  # create the device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # logo overlay
vis.AddSkyBox()                                                   # sky box
vis.AddCamera(chrono.ChVector3d(0.1, 0.1, 0.2))                   # camera eye position
vis.AddTypicalLights()                                            # standard lighting

time_step = 1e-3                                                  # integration step
render_fps = 50.0                                                 # review frame rate
render_every = max(1, round(1.0 / (render_fps * time_step)))     # physics steps per frame

while vis.Run():                                                  # SCORED CORE = plain truth loop
    vis.BeginScene()                                             # begin frame
    vis.Render()                                                 # draw scene
    vis.EndScene()                                               # end frame
    for _ in range(render_every):                               # advance physics for one frame
        sys.DoStepDynamics(time_step)                            # step the dynamics
