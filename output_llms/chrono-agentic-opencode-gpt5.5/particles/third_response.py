"""Three dynamic PyChrono spheres in a zero-gravity NSC particle scene.

The script creates three ChBodyEasySphere bodies at the requested positions,
assigns the requested initial velocities, and applies pairwise gravitational
attraction so the bodies exhibit a three-body interaction while remaining free
to collide if their trajectories meet.
"""


import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


# === Constants ===
time_step = 0.005
sim_end = 12.0
render_fps = 30.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

sphere_radius = 1.5
sphere_density = 1000.0
sphere_mass = 1.0
sphere_inertia = (2.0 / 5.0) * sphere_mass * sphere_radius * sphere_radius
gravity_constant = 150.0
softening = 0.5


# === Helpers ===
def make_sphere(name, pos, vel, color, material):
    sphere = chrono.ChBodyEasySphere(sphere_radius, sphere_density, True, True, material)
    sphere.SetName(name)
    sphere.SetMass(sphere_mass)
    sphere.SetInertiaXX(chrono.ChVector3d(sphere_inertia, sphere_inertia, sphere_inertia))
    sphere.SetPos(pos)
    sphere.SetPosDt(vel)
    sphere.GetVisualShape(0).SetColor(color)
    return sphere


def apply_pairwise_attraction(bodies):
    for body in bodies:
        body.EmptyAccumulators()
    for i, body_a in enumerate(bodies):
        pos_a = body_a.GetPos()
        for body_b in bodies[i + 1:]:
            pos_b = body_b.GetPos()
            delta = pos_b - pos_a
            distance_sq = delta.Length2() + softening * softening
            distance = distance_sq ** 0.5
            direction = delta / distance
            force_mag = gravity_constant * sphere_mass * sphere_mass / distance_sq
            force = direction * force_mag
            body_a.AccumulateForce(force, pos_a, False)
            body_b.AccumulateForce(-force, pos_b, False)


# === System & Bodies ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(80)

contact_mat = chrono.ChContactMaterialNSC()
contact_mat.SetFriction(0.2)
contact_mat.SetRestitution(0.8)

sphere1 = make_sphere(
    "sphere_1",
    chrono.ChVector3d(10, 0, 0),
    chrono.ChVector3d(0.5, 0, 0.1),
    chrono.ChColor(0.8, 0.1, 0.1),
    contact_mat,
)
sphere2 = make_sphere(
    "sphere_2",
    chrono.ChVector3d(-10, -10, 0),
    chrono.ChVector3d(-0.5, 0, -0.1),
    chrono.ChColor(0.1, 0.4, 0.9),
    contact_mat,
)
sphere3 = make_sphere(
    "sphere_3",
    chrono.ChVector3d(0, 20, 0),
    chrono.ChVector3d(0, -0.5, 0.2),
    chrono.ChColor(0.1, 0.8, 0.2),
    contact_mat,
)

sys.Add(sphere1)
sys.Add(sphere2)
sys.Add(sphere3)
spheres = [sphere1, sphere2, sphere3]  # cache: reused every physics step


# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Three-Body Particle Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, -45, 25), chrono.ChVector3d(0, 3, 0))
vis.AddTypicalLights()
vis.AddGrid(5.0, 5.0, 12, 12, chrono.ChCoordsysd(), chrono.ChColor(0.35, 0.35, 0.35))


# === Main Loop ===
frame = 0

try:

    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(render_every):
            t = sys.GetChTime()  # cache: current simulation time for this step
            p1 = sphere1.GetPos()  # cache: logged and used for compact output
            p2 = sphere2.GetPos()  # cache: logged and used for compact output
            p3 = sphere3.GetPos()  # cache: logged and used for compact output
            v1 = sphere1.GetPosDt()  # cache: logged velocity for sphere 1
            v2 = sphere2.GetPosDt()  # cache: logged velocity for sphere 2
            v3 = sphere3.GetPosDt()  # cache: logged velocity for sphere 3
            apply_pairwise_attraction(spheres)
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence / invalid numeric state
    print(f"Simulation failed: {exc}")
    raise
except (OSError, IOError) as exc:  # recording file path or disk failure
    print(f"Recording output failed: {exc}")
    raise
finally:
    pass
