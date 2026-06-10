import os
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemSMC()                                            # SMC for FEA
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))     # Y-up gravity

beam_L = 0.1                                                          # beam length per segment m
beam_d = 0.01                                                         # circular section diameter m

sec = fea.ChBeamSectionEulerAdvanced()
sec.SetAsCircularSection(beam_d)                                      # circular cross-section
sec.SetDensity(2700)                                                   # aluminium kg/m^3
sec.SetYoungModulus(73e9)                                              # Young's modulus Pa
sec.SetShearModulusFromPoisson(0.3)                                    # shear modulus from Poisson
sec.SetRayleighDamping(0.000)                                          # no Rayleigh damping

mesh = fea.ChMesh()                                                    # FEA mesh container

builder1 = fea.ChBuilderBeamEuler()
builder1.BuildBeam(
    mesh, sec, 6,
    chrono.ChVector3d(0, 0, 0),                                        # root at origin
    chrono.ChVector3d(beam_L, 0, 0),                                   # tip along +X
    chrono.ChVector3d(0, 1, 0),                                        # section Y direction
)
beam_nodes1 = builder1.GetLastBeamNodes()                              # strong ref (SWIG GC)
beam_nodes1.front().SetFixed(True)                                     # fix root node

builder2 = fea.ChBuilderBeamEuler()
builder2.BuildBeam(
    mesh, sec, 6,
    chrono.ChVector3d(beam_L, 0, 0),                                   # start at tip of beam1
    chrono.ChVector3d(beam_L, beam_L, 0),                              # end upward along +Y
    chrono.ChVector3d(0, 1, 0),                                        # section Y direction
)
beam_nodes2 = builder2.GetLastBeamNodes()                              # strong ref (SWIG GC)
nodes_b1 = [beam_nodes1[i] for i in range(beam_nodes1.size())]        # keep all node refs b1
nodes_b2 = [beam_nodes2[i] for i in range(beam_nodes2.size())]        # keep all node refs b2

nodes_b2[-1].SetForce(chrono.ChVector3d(4, 2, 0))                     # tip force N
nodes_b2[-1].SetTorque(chrono.ChVector3d(0, -0.04, 0))                # tip torque Nm

sys.Add(mesh)                                                          # register mesh with system

sys.SetSolver(mkl.ChSolverPardisoMKL())                               # direct solver for stiff beams

ts = chrono.ChTimestepperHHT(sys)                                      # HHT timestepper
ts.SetStepControl(False)                                               # fixed step
sys.SetTimestepper(ts)

vis_surface = chrono.ChVisualShapeFEA(mesh)                            # surface/scalar FEA shape
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)  # bending moment Mz
vis_surface.SetColorscaleMinMax(-0.4, 0.4)                             # colorscale bounds
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)                                    # attach to mesh

vis_glyph = chrono.ChVisualShapeFEA(mesh)                              # glyph FEA shape
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)  # node coord systems
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.006)
vis_glyph.SetSymbolsScale(0.01)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)                                      # attach to mesh

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Beam FEA Demo")
vis.Initialize()                                                        # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.1, 0.2, 0.2), chrono.ChVector3d(0.05, 0.05, 0))
vis.AddTypicalLights()

time_step = 0.001                                                       # timestep for stiff beams s
sim_end = 10.0                                                          # simulation end time s
render_fps = 50.0                                                       # review render frame rate
render_every = max(1, round(1.0 / (render_fps * time_step)))           # cadence: physics steps per frame
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
