"""Three dynamic PyChrono spheres in an NSC particle-style interaction.

The model uses three ChBodyEasySphere bodies with collision enabled, zero
ambient gravity, the requested initial positions and velocities, and a simple
pairwise attractive force so the bodies visibly interact as a three-body system.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


# === Constants ===
# Physics and display constants are defined once so the loop uses cached values.
TIME_STEP = 0.01
SIM_END = 12.0
SPHERE_RADIUS = 1.2
SPHERE_DENSITY = 1000.0
FORCE_SCALE = 0.01
SOFTENING = 0.35

SPHERE_1_POS = chrono.ChVector3d(10.0, 0.0, 0.0)
SPHERE_2_POS = chrono.ChVector3d(-10.0, -10.0, 0.0)
SPHERE_3_POS = chrono.ChVector3d(0.0, 20.0, 0.0)

SPHERE_1_VEL = chrono.ChVector3d(0.5, 0.0, 0.1)
SPHERE_2_VEL = chrono.ChVector3d(-0.5, 0.0, -0.1)
SPHERE_3_VEL = chrono.ChVector3d(0.0, -0.5, 0.2)


def make_sphere(name, position, velocity, color, material):
    """Create one requested easy sphere with its initial state."""
    sphere = chrono.ChBodyEasySphere(
        SPHERE_RADIUS, SPHERE_DENSITY, True, True, material
    )
    sphere.SetName(name)
    sphere.SetPos(position)
    sphere.SetPosDt(velocity)
    sphere.GetVisualShape(0).SetColor(color)
    return sphere


def vector_length(vec):
    """Return Euclidean length for a Chrono vector."""
    return math.sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z)


def apply_pairwise_forces(bodies):
    """Apply central pairwise forces between all sphere pairs."""
    for body in bodies:
        body.EmptyAccumulators()

    for i, body_i in enumerate(bodies):
        pos_i = body_i.GetPos()  # cache: reused for all pair force calculations
        mass_i = body_i.GetMass()  # cache: pair force scale reads mass repeatedly
        for body_j in bodies[i + 1 :]:
            pos_j = body_j.GetPos()  # cache: reused for symmetric force
            delta = pos_j - pos_i
            distance = max(vector_length(delta), 1.0e-6)
            direction = delta / distance
            mass_j = body_j.GetMass()  # cache: symmetric force magnitude
            magnitude = FORCE_SCALE * mass_i * mass_j / (
                distance * distance + SOFTENING * SOFTENING
            )
            force = direction * magnitude
            body_i.AccumulateForce(force, pos_i, False)
            body_j.AccumulateForce(-force, pos_j, False)


# === System ===
# NSC with Bullet collision supports the requested colliding easy spheres.
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, 0.0))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(80)

contact_material = chrono.ChContactMaterialNSC()
contact_material.SetFriction(0.15)
contact_material.SetRestitution(0.85)


# === Bodies ===
# The three requested ChBodyEasySphere objects start at the requested states.
sphere_1 = make_sphere(
    "sphere_1", SPHERE_1_POS, SPHERE_1_VEL, chrono.ChColor(0.9, 0.15, 0.1), contact_material
)
sphere_2 = make_sphere(
    "sphere_2", SPHERE_2_POS, SPHERE_2_VEL, chrono.ChColor(0.1, 0.35, 0.9), contact_material
)
sphere_3 = make_sphere(
    "sphere_3", SPHERE_3_POS, SPHERE_3_VEL, chrono.ChColor(0.1, 0.75, 0.25), contact_material
)

for sphere in (sphere_1, sphere_2, sphere_3):
    sys.Add(sphere)

spheres = (sphere_1, sphere_2, sphere_3)  # cache: reused every time step


# === Visualization ===
# Irrlicht is initialized before adding camera, sky, lights, and grid elements.
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Three-Sphere Particle Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.0, -45.0, 32.0), chrono.ChVector3d(0.0, 4.0, 0.0))
vis.AddTypicalLights()
vis.AddGrid(
    5.0,
    5.0,
    12,
    12,
    chrono.ChCoordsysd(chrono.ChVector3d(0.0, 0.0, -1.0), chrono.QUNIT),
    chrono.ChColor(0.35, 0.35, 0.35),
)


# === Main loop ===
# The loop renders the three moving spheres and advances their mutual interaction.

try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        sim_time = sys.GetChTime()  # cache: one system-time read per step
        apply_pairwise_forces(spheres)
        sys.DoStepDynamics(TIME_STEP)
except (RuntimeError, ValueError) as exc:  # solver divergence / invalid vector state
    traceback.print_exc()
    raise
finally:
    pass


# === Post-processing ===
