import os
import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr


# Model1 builds several chains of ANCF cable beams, each pinned to its own
# truss and ending on a free box body, exactly like the FEAcables benchmark.
class Model1:
    def __init__(self, sys, mesh, n_chains=6):
        self.bodies = []                                              # end box body of every chain
        self.nodes = []                                               # end cable node of every chain
        self.refs = []                                                # keep SWIG containers alive
        for i in range(n_chains):                                     # build n_chains independent chains
            # ANCF cable cross-section: thin flexible cable
            msection_cable = fea.ChBeamSectionCable()                 # cable section properties
            msection_cable.SetDiameter(0.015)                         # 15 mm diameter
            msection_cable.SetYoungModulus(0.01e9)                    # very soft, 0.01 GPa
            msection_cable.SetRayleighDamping(0.000)                  # no structural damping

            # one fixed truss per chain, acting as the anchoring reference frame
            mtruss = chrono.ChBody()                                  # rigid reference body
            mtruss.SetFixed(True)                                     # truss is world-fixed
            sys.Add(mtruss)                                           # register the truss

            # build a horizontal cable; element count and start offset grow per chain
            builder = fea.ChBuilderCableANCF()                        # ANCF cable builder
            builder.BuildBeam(
                mesh,                                                 # target FEA mesh
                msection_cable,                                       # the cable section
                1 + i,                                               # n_elements increases each chain
                chrono.ChVector3d(0, 0, -0.1 * i),                   # A: start, staggered in Z to avoid overlap
                chrono.ChVector3d(0.1 + 0.1 * i, 0, -0.1 * i),       # B: end, longer each chain
            )

            beam_nodes = builder.GetLastBeamNodes()                  # keep the SWIG container alive
            self.refs.append(beam_nodes)                             # store strong reference

            # apply a small constant force on the cable's free front node
            beam_nodes.front().SetForce(chrono.ChVector3d(0, -0.7, 0))  # downward tug, N

            # hinge the cable's back node to the truss (3 translational DOF)
            constraint_hinge = fea.ChLinkNodeFrame()                 # node-to-body hinge
            constraint_hinge.Initialize(beam_nodes.back(), mtruss)   # pin back node to truss
            sys.Add(constraint_hinge)                                # register the hinge

            # a small box body that the cable's free end will drag around
            mbox = chrono.ChBodyEasyBox(0.2, 0.04, 0.04, 1000)       # size x,y,z and density
            mbox.SetPos(beam_nodes.front().GetPos() + chrono.ChVector3d(0.1, 0, 0))  # just past the cable tip
            sys.Add(mbox)                                            # register the box

            # constrain the cable end node to the box: position (hinge)
            constraint_pos = fea.ChLinkNodeFrame()                  # node-to-box position link
            constraint_pos.Initialize(beam_nodes.front(), mbox)     # tie tip node to box
            sys.Add(constraint_pos)                                  # register the position link

            # constrain the cable end direction to the box: slope (ANCF gradient)
            constraint_dir = fea.ChLinkNodeSlopeFrame()             # node-direction-to-box link
            constraint_dir.Initialize(beam_nodes.front(), mbox)     # tie tip slope to box
            constraint_dir.SetDirectionInBodyCoords(chrono.ChVector3d(1, 0, 0))  # reference direction
            sys.Add(constraint_dir)                                 # register the slope link

            self.bodies.append(mbox)                                # remember end body for printing
            self.nodes.append(beam_nodes.front())                   # remember end node

    # PrintBodyPositions prints the end-body position of each chain, called each step
    def PrintBodyPositions(self):
        print("Time: ", round(sys.GetChTime(), 4))                  # current simulation time
        for i, body in enumerate(self.bodies):                      # iterate over all chains
            pos = body.GetPos()                                     # end box position
            print("  chain ", i, " end body pos: ", pos.x, pos.y, pos.z)  # x,y,z coordinates


# FEA scenes use ChSystemSMC; the cable benchmark is Y-up under gravity
sys = chrono.ChSystemSMC()                                            # SMC system for FEA
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))      # gravity, Y-up

mesh = fea.ChMesh()                                                   # the FEA mesh holds all cables
mesh.SetAutomaticGravity(True)                                        # apply gravity to cable nodes

model = Model1(sys, mesh, 6)                                          # build six staggered chains

sys.Add(mesh)                                                         # register the mesh with the system

# surface colour field: ANCF axial stretch along every cable
mvisualizebeamA = chrono.ChVisualShapeFEA(mesh)                       # mesh is a ctor argument
mvisualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ANCF_BEAM_AX)  # axial stretch field
mvisualizebeamA.SetColorscaleMinMax(-0.005, 0.005)                   # field colour range
mvisualizebeamA.SetSmoothFaces(True)                                 # smooth shaded faces
mvisualizebeamA.SetWireframe(False)                                  # solid, not wireframe
mesh.AddVisualShapeFEA(mvisualizebeamA)                              # register surface shape

# glyph shape: draw a dot at every cable node
mvisualizebeamB = chrono.ChVisualShapeFEA(mesh)                       # second visual shape
mvisualizebeamB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)  # node dots
mvisualizebeamB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)  # no field on glyphs
mvisualizebeamB.SetSymbolsThickness(0.006)                          # dot thickness
mvisualizebeamB.SetSymbolsScale(0.01)                               # dot scale
mvisualizebeamB.SetZbufferHide(False)                               # always draw glyphs
mesh.AddVisualShapeFEA(mvisualizebeamB)                             # register glyph shape

# ANCF cable uses the sparse-QR solver with the linearized implicit timestepper
solver = chrono.ChSolverSparseQR()                                   # direct sparse QR solver
sys.SetSolver(solver)                                                # attach the solver
solver.UseSparsityPatternLearner(True)                               # learn the sparsity pattern
solver.LockSparsityPattern(True)                                     # then lock it for speed
solver.SetVerbose(False)                                             # quiet solver

ts = chrono.ChTimestepperEulerImplicitLinearized(sys)                # linearized implicit stepper
sys.SetTimestepper(ts)                                               # attach the timestepper

# Irrlicht visualization window (Initialize first, then scene elements, NO grid)
vis = chronoirr.ChVisualSystemIrrlicht()                             # create the renderer
vis.AttachSystem(sys)                                                # bind it to the system
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)                   # Y-up world
vis.SetWindowSize(1024, 768)                                         # window resolution
vis.SetWindowTitle("Cables FEM - ANCF chains")                      # window title
vis.Initialize()                                                     # build the device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))    # corner logo
vis.AddSkyBox()                                                      # sky box
vis.AddCamera(chrono.ChVector3d(0, 0.6, -1), chrono.ChVector3d(0, 0, 0))  # eye, target
vis.AddTypicalLights()                                               # standard lights

time_step = 0.01                                                     # ANCF cable timestep
sim_end = 10.0                                                       # simulation length, s
render_fps = 50.0                                                    # frames per second for review
render_every = max(1, round(1.0 / (render_fps * time_step)))        # untagged cadence constant
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()                                                # begin frame
    vis.Render()                                                    # draw the scene
    vis.EndScene()                                                  # end frame
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)                               # advance one physics step
        model.PrintBodyPositions()                                  # print end-body positions each step
        if sys.GetChTime() >= sim_end:
            break
