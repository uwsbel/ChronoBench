import os
import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr


class Model1:
    def __init__(self, system, mesh, n_chains=6):                      # n_chains default 6
        self.beam_nodes = []                                           # keep node refs alive (SWIG GC)
        self.end_bodies = []                                           # end box body of each chain

        for i in range(n_chains):                                      # one chain per iteration
            # ANCF cable section: 15 mm diameter, soft modulus, no Rayleigh damping
            msection_cable = fea.ChBeamSectionCable()                  # cable cross-section
            msection_cable.SetDiameter(0.015)                          # 15 mm diameter
            msection_cable.SetYoungModulus(0.01e9)                     # E = 0.01 GPa
            msection_cable.SetRayleighDamping(0.000)                   # no structural damping

            # fixed truss body acting as the reference frame for this chain
            mtruss = chrono.ChBody()                                   # per-chain truss
            mtruss.SetFixed(True)                                      # fixed reference frame
            system.Add(mtruss)

            # number of cable elements grows with chain index to avoid identical chains
            n_elements = 1 + i                                         # increasing element count
            # offset successive chains along Z so they do not overlap
            starting_point = chrono.ChVector3d(0, 0, -0.1 * (i + 1))   # chain start
            ending_point = chrono.ChVector3d(0.5, 0, -0.1 * (i + 1))   # chain end

            builder = fea.ChBuilderCableANCF()                         # ANCF cable builder
            builder.BuildBeam(mesh, msection_cable,                    # add nodes + elements to mesh
                              n_elements,
                              starting_point,
                              ending_point)

            # keep a strong reference to the node container before indexing (SWIG GC pitfall)
            chain_nodes = builder.GetLastBeamNodes()                   # this chain's nodes
            self.beam_nodes.append(chain_nodes)

            # apply a small downward pull on the front node of the cable
            chain_nodes.front().SetForce(chrono.ChVector3d(0, -0.2, 0))  # tip load

            # hinge the back node of the cable to the fixed truss (3 translational DOF)
            constraint_hinge = fea.ChLinkNodeFrame()                   # node->truss hinge
            constraint_hinge.Initialize(chain_nodes.back(), mtruss)    # pin back node
            system.Add(constraint_hinge)

            # connect the cable front node to a rigid box body
            mbox = chrono.ChBodyEasyBox(0.04, 0.04, 0.04, 1000)        # end box (4 cm, 1000 kg/m^3)
            mbox.SetPos(chain_nodes.front().GetPos())                  # spawn at the cable tip
            system.Add(mbox)
            self.end_bodies.append(mbox)

            # constrain the cable front node to the box (3 translational DOF)
            constraint_pos = fea.ChLinkNodeFrame()                     # node->box link
            constraint_pos.Initialize(chain_nodes.front(), mbox)       # tie node to box
            system.Add(constraint_pos)

            # rigidly fix the box orientation to the cable tip direction (all DOF)
            constraint_dir = fea.ChLinkNodeSlopeFrame()                # node-direction->box link
            constraint_dir.Initialize(chain_nodes.front(), mbox)       # tie tip slope to box
            constraint_dir.SetDirectionInBodyCoords(chrono.ChVector3d(1, 0, 0))  # along +X
            system.Add(constraint_dir)

    def PrintBodyPositions(self):                                      # log end-body positions
        for i, body in enumerate(self.end_bodies):                     # each chain's end box
            pos = body.GetPos()                                        # current world position
            print("Chain ", i, " end body pos: ", pos.x, "  ", pos.y, "  ", pos.z)


sys = chrono.ChSystemSMC()                                             # SMC system for FEA

mesh = fea.ChMesh()                                                    # FEA mesh container

model = Model1(sys, mesh, n_chains=6)                                  # build 6 cable chains

sys.Add(mesh)                                                          # register the mesh

# visual shape 1: scalar bending-moment field over the cable surface
visualizebeamA = chrono.ChVisualShapeFEA(mesh)                         # surface/scalar shape
visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)  # bending moment Mz
visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)                          # color range (lo, hi)
visualizebeamA.SetSmoothFaces(True)                                    # smooth shading
visualizebeamA.SetWireframe(False)                                     # solid surface
mesh.AddVisualShapeFEA(visualizebeamA)

# visual shape 2: node coordinate-system glyphs
visualizebeamC = chrono.ChVisualShapeFEA(mesh)                         # node glyph shape
visualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)  # node triads
visualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)   # no field on glyphs
visualizebeamC.SetSymbolsThickness(0.006)                              # glyph line thickness
visualizebeamC.SetSymbolsScale(0.01)                                   # glyph size
visualizebeamC.SetZbufferHide(False)                                   # draw over geometry
mesh.AddVisualShapeFEA(visualizebeamC)

# ANCF cable: sparse-QR direct solver
solver = chrono.ChSolverSparseQR()                                     # direct sparse solver
sys.SetSolver(solver)
solver.UseSparsityPatternLearner(True)                                 # learn sparsity pattern
solver.LockSparsityPattern(True)                                       # lock once learned
solver.SetVerbose(False)                                               # quiet solver

ts = chrono.ChTimestepperEulerImplicitLinearized(sys)                  # linearized implicit stepper
sys.SetTimestepper(ts)

# Irrlicht window (Initialize FIRST, then scene elements, NO grid)
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)                                           # window size
vis.SetWindowTitle("ANCF cable chains")                                # window title
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0.6, -1))                           # eye behind the chains
vis.AddTypicalLights()

time_step = 0.01                                                       # ANCF cable timestep
render_fps = 50.0                                                      # review fps
render_every = max(1, round(1.0 / (render_fps * time_step)))           # untagged cadence constant
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)                                  # advance one cable step
        model.PrintBodyPositions()                                     # log end-body positions each step
