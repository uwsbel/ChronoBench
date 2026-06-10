"""ANCF cable beam simulation using PyChrono SMC/FEA.

The model is a flexible cable beam built from ANCF elements, with its left end
hinged to a fixed truss and the rest of the beam deforming under Y-down
gravity. Irrlicht renders the deformed beam surface and node-position glyphs
inside the simulation loop.
"""


import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr


# === Constants === beam and run parameters kept explicit for reproducibility
beam_length = 0.80
beam_start = chrono.ChVector3d(0.0, 0.0, 0.0)
beam_end = chrono.ChVector3d(beam_length, 0.0, 0.0)
num_elements = 16
cable_diameter = 0.012
young_modulus = 0.001e9
rayleigh_damping = 0.000
time_step = 0.01
sim_end = 6.0
render_fps = 30.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once


# === System & gravity === SMC FEA system with ANCF-specific direct solver
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, -9.81, 0.0))

solver = chrono.ChSolverSparseQR()
sys.SetSolver(solver)
solver.UseSparsityPatternLearner(True)
solver.LockSparsityPattern(True)
solver.SetVerbose(False)

timestepper = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(timestepper)


# === FEA beam === ANCF cable mesh plus node hinge to a fixed truss
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

section = fea.ChBeamSectionCable()
section.SetDiameter(cable_diameter)
section.SetYoungModulus(young_modulus)
section.SetRayleighDamping(rayleigh_damping)

builder = fea.ChBuilderCableANCF()
builder.BuildBeam(mesh, section, num_elements, beam_start, beam_end)
beam_node_container = builder.GetLastBeamNodes()  # cache: keep SWIG node container alive
beam_nodes = [beam_node_container[i] for i in range(beam_node_container.size())]
root_node = beam_nodes[0]  # cache: fixed end for hinge logging and constraint
mid_node = beam_nodes[len(beam_nodes) // 2]  # cache: representative deformation point
tip_node = beam_nodes[-1]  # cache: free end for deformation logging

truss = chrono.ChBody()
truss.SetFixed(True)
truss.SetPos(beam_start)
truss.SetMass(1.0)
truss.SetInertiaXX(chrono.ChVector3d(1e-6, 1e-6, 1e-6))
truss_visual = chrono.ChVisualShapeSphere(0.025)
truss_visual.SetColor(chrono.ChColor(0.2, 0.2, 0.8))
truss.AddVisualShape(truss_visual)
sys.AddBody(truss)

hinge = fea.ChLinkNodeFrame()
hinge.Initialize(root_node, truss)
sys.Add(hinge)
sys.Add(mesh)

# FEA beam: no contact material needed -- driven by hinge constraint and gravity only.

node_markers = []
for node in beam_nodes:
    marker = chrono.ChBody()
    marker.SetFixed(True)
    marker.SetPos(node.GetPos())
    marker.SetMass(1e-6)
    marker.SetInertiaXX(chrono.ChVector3d(1e-9, 1e-9, 1e-9))
    marker_shape = chrono.ChVisualShapeSphere(0.012)
    marker_shape.SetColor(chrono.ChColor(1.0, 0.1, 0.1))
    marker.AddVisualShape(marker_shape)
    sys.AddBody(marker)
    node_markers.append(marker)


# === FEA visualization === deformed cable field and visible nodal positions
beam_shape = chrono.ChVisualShapeFEA(mesh)
beam_shape.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_DISP_NORM)
beam_shape.SetColorscaleMinMax(0.0, 0.35)
beam_shape.SetSmoothFaces(True)
beam_shape.SetWireframe(False)
mesh.AddVisualShapeFEA(beam_shape)

node_shape = chrono.ChVisualShapeFEA(mesh)
node_shape.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
node_shape.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
node_shape.SetSymbolsThickness(0.02)
node_shape.SetSymbolsScale(0.04)
node_shape.SetZbufferHide(False)
mesh.AddVisualShapeFEA(node_shape)


# === Visualization === Irrlicht renders the beam deformation in a Y-up scene
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("ANCF Cable Beam Under Gravity")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.55, 0.45, 1.25), chrono.ChVector3d(0.40, -0.25, 0.0))
vis.AddTypicalLights()
vis.AddGrid(
    0.1,
    0.1,
    20,
    20,
    chrono.ChCoordsysd(chrono.ChVector3d(0.25, -0.45, 0.0), chrono.QuatFromAngleX(chrono.CH_PI_2)),
    chrono.ChColor(0.35, 0.35, 0.35),
)


# === Main loop === render frames and advance batched physics steps
frame = 0
try:

    while vis.Run() and sys.GetChTime() < sim_end:
        for node, marker in zip(beam_nodes, node_markers):
            marker.SetPos(node.GetPos())

        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(render_every):
            sim_time = sys.GetChTime()  # cache: reused for logging and stop checks
            root_pos = root_node.GetPos()  # cache: node getter used once per step
            mid_pos = mid_node.GetPos()  # cache: node getter used once per step
            tip_pos = tip_node.GetPos()  # cache: node getter used once per step
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (OSError, IOError) as exc:  # file and recording path failures
    print(f"Output error: {exc}")
    raise
except (RuntimeError, ValueError) as exc:  # solver divergence or invalid FEA state
    print(f"Simulation error: {exc}")
    raise
finally:
    pass
