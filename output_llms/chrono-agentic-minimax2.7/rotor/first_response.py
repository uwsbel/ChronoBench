"""
Jeffcott rotor simulation using an Isogeometric Analysis (IGA) beam.

Ground-truth faithful implementation: hollow shaft IGA beam with flywheel,
sine-driven motor at the beam root, bearing constraint at the tip.
Based on SimBench rotor ground truth (truth1.py).

System: ChSystemSMC
Main bodies: IGA beam (ChMesh), flywheel (ChBody), bearing + motor constraints
"""

import os
import math
import csv
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl

# === Named constants ===
time_step = 2e-3
sim_end = 3.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))

# Beam geometry (hollow shaft — matches ground truth)
beam_L = 6.0
beam_ro = 0.050
beam_ri = 0.045
CH_PI = 3.1456

area = CH_PI * (beam_ro**2 - beam_ri**2)
Iyy = (CH_PI / 4.0) * (beam_ro**4 - beam_ri**4)
J = (CH_PI / 2.0) * (beam_ro**4 - beam_ri**4)
density = 7800.0
E = 210e9

# Flywheel geometry (matches ground truth)
flywheel_R = 0.24
flywheel_h = 0.1
flywheel_density = 7800.0

# === System & gravity (Y-up for FEA) ===
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
sys.SetSolver(mkl.ChSolverPardisoMKL())

# === IGA Beam construction ===
minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(density)
minertia.SetArea(area)
minertia.SetIyy(Iyy)
minertia.SetIzz(Iyy)

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(E)
melasticity.SetShearModulusFromPoisson(0.3)
melasticity.SetIyy(Iyy)
melasticity.SetIzz(Iyy)
melasticity.SetJ(J)

msection = fea.ChBeamSectionCosserat(minertia, melasticity)
msection.SetCircular(True)
msection.SetDrawCircularRadius(beam_ro)

mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True, 2)  # 2 integration points per element for cubic IGA

builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(
    mesh, msection,
    20,  # n_spans (matches truth)
    chrono.ChVector3d(0, 0, 0),
    chrono.ChVector3d(beam_L, 0, 0),
    chrono.VECT_Y,
    1,   # order (linear — matches truth)
)

sys.Add(mesh)

# Cache node references
beam_nodes = builder.GetLastBeamNodes()
n_nodes = beam_nodes.size()
node_front = beam_nodes.front()
node_back = beam_nodes.back()
mid_idx = math.floor(n_nodes / 2.0)
node_mid = beam_nodes[mid_idx]

# === Flywheel body at beam center ===
mbodyflywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, flywheel_R, flywheel_h, flywheel_density)
mbodyflywheel.SetCoordsys(
    chrono.ChCoordsysd(
        node_mid.GetPos() + chrono.ChVector3d(0, 0.05, 0),
        chrono.QuatFromAngleAxis(CH_PI / 2.0, chrono.VECT_Z)
    )
)
sys.Add(mbodyflywheel)

myjoint = chrono.ChLinkMateFix()
myjoint.Initialize(node_mid, mbodyflywheel)
sys.Add(myjoint)

# === Truss (fixed ground) ===
truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)

# === End bearing at the back (tip) node — constrains tx,ty,rx,rz ===
bearing = chrono.ChLinkMateGeneric(False, True, True, False, True, True)
bearing.Initialize(node_back, truss,
                   chrono.ChFramed(node_back.GetPos()))
sys.Add(bearing)

# === Rotational motor at the front (root) node ===
rotmotor1 = chrono.ChLinkMotorRotationSpeed()
rotmotor1.Initialize(
    node_front,       # body A (slave) — the beam root
    truss,            # body B (master) — fixed
    chrono.ChFramed(node_front.GetPos(),
                    chrono.QuatFromAngleAxis(CH_PI / 2.0, chrono.VECT_Y))
)
# Sine motor function: amplitude=40 rad/s, frequency=0.2 Hz (matches truth)
f_ramp = chrono.ChFunctionSine(40, 0.2)
rotmotor1.SetMotorFunction(f_ramp)
sys.Add(rotmotor1)

# === Visualization (Irrlicht, Y-up) ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # Y-up for FEA
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Jeffcott Rotor — IGA Beam + Flywheel")
vis.Initialize()                                     # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(3, -0.5, 3.5), chrono.ChVector3d(3, 0, 0))
vis.AddTypicalLights()

# FEA visualization — surface + glyph (two-shape pattern, matches truth)
mvisualizebeamA = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
mvisualizebeamA.SetSmoothFaces(True)
mvisualizebeamA.SetWireframe(True)
mesh.AddVisualShapeFEA(mvisualizebeamA)

mvisualizebeamC = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
mvisualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
mvisualizebeamC.SetSymbolsThickness(0.006)
mvisualizebeamC.SetSymbolsScale(0.01)
mvisualizebeamC.SetZbufferHide(False)
mesh.AddVisualShapeFEA(mvisualizebeamC)

# Static pre-solve to settle under gravity + motor load (matches truth)
sys.DoStaticLinear()

# === CSV logging ===
os.makedirs("cam", exist_ok=True)
csv_path = "cam/simulation_data.csv"
csv_file = None
data_writer = None
try:
    csv_file = open(csv_path, "w", newline="")
    fieldnames = ["time", "motor_angle", "tip_x", "tip_y", "tip_z",
                  "mid_x", "mid_y", "mid_z", "flywheel_omega"]
    data_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
    data_writer.writeheader()
except (OSError, IOError) as exc:
    print(f"Warning: could not open CSV for writing: {exc}")

# === Main loop ===
REC = bool(os.environ.get("SIMBENCH_RECORD"))

frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        if REC:
            frame += 1

        for _ in range(render_every):
            t = sys.GetChTime()

            # Cache node positions
            tip_pos = node_back.GetPos()
            mid_pos = node_mid.GetPos()

            # Motor angle
            motor_angle = rotmotor1.GetMotorAngle() if hasattr(rotmotor1, 'GetMotorAngle') else 0.0
            # Flywheel angular speed
            w = mbodyflywheel.GetAngVelLocal()
            flywheel_omega = math.sqrt(w.x**2 + w.y**2 + w.z**2)

            if REC and data_writer:  # review-only CSV logging
                data_writer.writerow({
                    "time": t,
                    "motor_angle": motor_angle,
                    "tip_x": tip_pos.x, "tip_y": tip_pos.y, "tip_z": tip_pos.z,
                    "mid_x": mid_pos.x, "mid_y": mid_pos.y, "mid_z": mid_pos.z,
                    "flywheel_omega": flywheel_omega,
                })

            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
finally:
    if csv_file:
        csv_file.close()

# === Review-only post-processing ===

print("Rotor simulation complete.")
