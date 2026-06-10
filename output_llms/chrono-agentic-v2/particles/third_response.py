"""
Three-body gravitational particle simulation using PyChrono (NSC system).

Models a three-body problem: three spherical particles interacting via
mutual gravitational attraction. Each sphere has an initial position and
velocity; the gravitational forces cause complex orbital/chaotic dynamics.

System type:  ChSystemNSC (no contact — pure gravitational MBS)
Bodies:       Sphere 1 at (10, 10, 0), Sphere 2 at (-10, -10, 0), Sphere 3 at (0, 20, 0)
Initial velocities: Sphere 1 (0.5, 0, 0.1), Sphere 2 (-0.5, 0, -0.1), Sphere 3 (0, -0.5, 0.2)
Expected behavior: Three bodies orbit/interact under mutual gravity, exhibiting
                   a classic chaotic three-body trajectory.
"""

import os
import csv
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Constants & simulation parameters ===
SPHERE_RADIUS   = 0.5       # visual radius [m]
SPHERE_MASS     = 1e12      # mass [kg] — large for measurable gravity at ~20 m separation
GRAV_CONST      = 6.674e-11 # gravitational constant [m³ kg⁻¹ s⁻²]
TIME_STEP       = 0.05      # physics time step [s]
SIM_END         = 30.0      # simulation duration [s]
RENDER_FPS      = 50.0
render_every    = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Sphere initial positions
POS1 = chrono.ChVector3d(10,   10,  0)
POS2 = chrono.ChVector3d(-10, -10,  0)
POS3 = chrono.ChVector3d(0,    20,  0)

# Sphere initial velocities
VEL1 = chrono.ChVector3d( 0.5,  0.0,  0.1)
VEL2 = chrono.ChVector3d(-0.5,  0.0, -0.1)
VEL3 = chrono.ChVector3d( 0.0, -0.5,  0.2)

# === System & gravity ===
# Pure three-body gravitational simulation — no surface contacts, so gravity
# is disabled at the system level and applied as custom forces each step.
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))  # disable system gravity; using custom N-body gravity

# === Bodies ===
mat = chrono.ChContactMaterialNSC()
mat.SetFriction(0.0)

sphere1 = chrono.ChBodyEasySphere(SPHERE_RADIUS, 1.0, True, False, mat)
sphere1.SetPos(POS1)
sphere1.SetPosDt(VEL1)
sphere1.SetMass(SPHERE_MASS)
sphere1.SetInertiaXX(chrono.ChVector3d(
    0.4 * SPHERE_MASS * SPHERE_RADIUS**2,
    0.4 * SPHERE_MASS * SPHERE_RADIUS**2,
    0.4 * SPHERE_MASS * SPHERE_RADIUS**2))
sphere1.GetVisualShape(0).SetColor(chrono.ChColor(0.9, 0.3, 0.2))
sys.Add(sphere1)

sphere2 = chrono.ChBodyEasySphere(SPHERE_RADIUS, 1.0, True, False, mat)
sphere2.SetPos(POS2)
sphere2.SetPosDt(VEL2)
sphere2.SetMass(SPHERE_MASS)
sphere2.SetInertiaXX(chrono.ChVector3d(
    0.4 * SPHERE_MASS * SPHERE_RADIUS**2,
    0.4 * SPHERE_MASS * SPHERE_RADIUS**2,
    0.4 * SPHERE_MASS * SPHERE_RADIUS**2))
sphere2.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.6, 0.9))
sys.Add(sphere2)

sphere3 = chrono.ChBodyEasySphere(SPHERE_RADIUS, 1.0, True, False, mat)
sphere3.SetPos(POS3)
sphere3.SetPosDt(VEL3)
sphere3.SetMass(SPHERE_MASS)
sphere3.SetInertiaXX(chrono.ChVector3d(
    0.4 * SPHERE_MASS * SPHERE_RADIUS**2,
    0.4 * SPHERE_MASS * SPHERE_RADIUS**2,
    0.4 * SPHERE_MASS * SPHERE_RADIUS**2))
sphere3.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.9, 0.3))
sys.Add(sphere3)

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Three-Body Gravitational Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(40, -40, 30), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(5.0, 5.0, 20, 20,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Review-only setup ===


# === Main loop ===
frame = 0

try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            # Apply mutual gravitational forces between the three spheres
            p1 = sphere1.GetPos()  # cache: fetched once per step
            p2 = sphere2.GetPos()  # cache: fetched once per step
            p3 = sphere3.GetPos()  # cache: fetched once per step
            m1 = sphere1.GetMass()  # cache: constant
            m2 = sphere2.GetMass()  # cache: constant
            m3 = sphere3.GetMass()  # cache: constant

            # Force on sphere1 from sphere2
            r12 = p2 - p1
            dist12 = r12.Length()
            if dist12 > 1e-3:
                f12 = GRAV_CONST * m1 * m2 / (dist12 * dist12) * (r12 / dist12)
                sphere1.AccumulateForce(f12, p1, False)
                sphere2.AccumulateForce(-f12, p2, False)

            # Force on sphere1 from sphere3
            r13 = p3 - p1
            dist13 = r13.Length()
            if dist13 > 1e-3:
                f13 = GRAV_CONST * m1 * m3 / (dist13 * dist13) * (r13 / dist13)
                sphere1.AccumulateForce(f13, p1, False)
                sphere3.AccumulateForce(-f13, p3, False)

            # Force on sphere2 from sphere3
            r23 = p3 - p2
            dist23 = r23.Length()
            if dist23 > 1e-3:
                f23 = GRAV_CONST * m2 * m3 / (dist23 * dist23) * (r23 / dist23)
                sphere2.AccumulateForce(f23, p2, False)
                sphere3.AccumulateForce(-f23, p3, False)


            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break

except (RuntimeError, ValueError) as exc:  # solver divergence / bad body state
    import traceback
    traceback.print_exc()
    raise

finally:
    pass
