"""
Particle emitter demo — PyChrono NSC rigid-body simulation.

System type: ChSystemNSC (non-smooth contact, no gravity — gravitational
attraction between bodies is handled via user-applied forces).
Bodies: one large fixed sphere attractor + dynamically emitted small spheres.
Particle creator: ChRandomShapeCreatorSpheres with ChZhangDistribution for
diameter (average=0.6, min=0.23) and ChConstantDistribution for density (1600).
Energy: kinetic energy (sum of 0.5*m*v^2) and gravitational potential energy
(sum of -G*mA*mB/r for all body pairs) printed every simulation step.
G_constant is defined inside the loop for organizational clarity.
Expected behaviour: particles stream from the emitter, attract toward the big
central sphere and each other; KE + PE reported each step.
"""

import os
import csv
import math
from itertools import combinations

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Constants ===
PARTICLES_PER_SEC = 2000
MAX_PARTICLES     = 200
TIME_STEP         = 1e-2
SIM_END           = 10.0
RENDER_FPS        = 50.0
RENDER_EVERY      = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once


# === System & gravity ===
# ChSystemNSC with Bullet collision; gravity zeroed — N-body gravitation applied manually
sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))
sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(40)
coll = sys.GetCollisionSystem()  # cache: fetched once for callback binding

# === Bodies ===
sphere_mat = chrono.ChContactMaterialNSC()
sphere_mat.SetFriction(0.2)
sphere_mat.SetRestitution(0.0)

big_sphere = chrono.ChBodyEasySphere(2.1, 1800, True, True, sphere_mat)
big_sphere.SetPos(chrono.ChVector3d(1, 1, 0))
big_sphere.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(big_sphere)

# === Particle emitter ===

class MyCreatorForAll(chrono.ChRandomShapeCreator_AddBodyCallback):
    """Callback: set texture + bind each emitted body to Irrlicht and collision."""

    def __init__(self, vis, collision_sys):
        chrono.ChRandomShapeCreator_AddBodyCallback.__init__(self)
        self._vis = vis
        self._coll = collision_sys

    def OnAddBody(self, body, coords, creator):
        if body.GetVisualModel() and body.GetVisualModel().GetNumShapes() > 0:
            body.GetVisualShape(0).SetTexture(
                chrono.GetChronoDataFile("textures/bluewhite.png")
            )
        self._vis.BindItem(body)
        self._coll.BindItem(body)
        body.SetUseGyroTorque(False)


emitter = chrono.ChParticleEmitter()
emitter.SetParticlesPerSecond(PARTICLES_PER_SEC)
emitter.SetUseParticleReservoir(True)
emitter.SetParticleReservoirAmount(MAX_PARTICLES)

pos_random = chrono.ChRandomParticlePositionOnGeometry()
pos_random.SetGeometry(chrono.ChBox(50, 50, 50), chrono.ChFramed())
emitter.SetParticlePositioner(pos_random)

align_random = chrono.ChRandomParticleAlignmentUniform()
emitter.SetParticleAligner(align_random)

vel_random = chrono.ChRandomParticleVelocityAnyDirection()
vel_random.SetModulusDistribution(chrono.ChUniformDistribution(0.0, 0.5))
emitter.SetParticleVelocity(vel_random)

avel_random = chrono.ChRandomParticleVelocityAnyDirection()
avel_random.SetModulusDistribution(chrono.ChUniformDistribution(0.0, 0.2))
emitter.SetParticleAngularVelocity(avel_random)

# Replace ConvexHulls creator with Spheres creator using Zhang diameter distribution
mcreator_spheres = chrono.ChRandomShapeCreatorSpheres()
mcreator_spheres.SetDiameterDistribution(chrono.ChZhangDistribution(0.6, 0.23))
mcreator_spheres.SetDensityDistribution(chrono.ChConstantDistribution(1600))
emitter.SetParticleCreator(mcreator_spheres)

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Particle Emitter — Sphere Creator + Energy")
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 14, -20), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()

creator_callback = MyCreatorForAll(vis, coll)  # keep reference to prevent GC
emitter.RegisterAddBodyCallback(creator_callback)

# === Review-only setup ===

# === Main loop ===
frame = 0

try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(RENDER_EVERY):
            emitter.EmitParticles(sys, TIME_STEP)

            # Reset all accumulated forces before applying gravitational attraction
            for body in sys.GetBodies():
                body.EmptyAccumulators()

            # G_constant defined here for organizational clarity (prompt requirement)
            G_constant = 6.674e-3  # scaled gravitational constant for N-body demo

            # Energy calculations
            kinetic_energy   = 0.0
            potential_energy = 0.0
            bodies = list(sys.GetBodies())  # cache: body list fetched once per step

            for b in bodies:
                v = b.GetPosDt()  # linear velocity vector
                kinetic_energy += 0.5 * b.GetMass() * v.Length2()

            for bodyA, bodyB in combinations(bodies, 2):
                d_vec    = bodyB.GetPos() - bodyA.GetPos()
                r_attract = d_vec.Length()
                if r_attract < 1e-10:
                    continue
                f_mag = G_constant * bodyA.GetMass() * bodyB.GetMass() / (r_attract * r_attract)
                F     = (d_vec / r_attract) * f_mag
                bodyA.AccumulateForce( F, bodyA.GetPos(), False)
                bodyB.AccumulateForce(-F, bodyB.GetPos(), False)
                potential_energy += -G_constant * bodyA.GetMass() * bodyB.GetMass() / r_attract

            total_energy = kinetic_energy + potential_energy
            print(
                f"t={sys.GetChTime():.3f} | KE={kinetic_energy:10.4f} | "
                f"PE={potential_energy:10.4f} | Total={total_energy:10.4f}"
            )


            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break

except (RuntimeError, ValueError) as exc:  # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
