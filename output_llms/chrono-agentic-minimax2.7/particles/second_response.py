"""
Particle emitter simulation with gravitational attraction.
Uses ChRandomShapeCreatorSpheres with energy calculations (kinetic + potential + total).
"""

import os
import math
import csv
from itertools import combinations

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


class MyCreatorForAll(chrono.ChRandomShapeCreator_AddBodyCallback):
    """Callback to bind visual shapes and collision for emitted particles."""

    def __init__(self, vis, coll):
        super().__init__()
        self.vis = vis
        self.coll = coll

    def OnAddBody(self, body, coords, creator):
        body.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
        self.vis.BindItem(body)
        self.coll.BindItem(body)
        body.SetUseGyroTorque(False)


# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
coll = sys.GetCollisionSystem()

sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(40)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))


# === Bodies: central sphere ===
sphere_mat = chrono.ChContactMaterialNSC()
sphere_mat.SetFriction(0.2)
msphereBody = chrono.ChBodyEasySphere(2.1, 1800, True, True, sphere_mat)
msphereBody.SetPos(chrono.ChVector3d(1, 1, 0))
msphereBody.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(msphereBody)


# === Particle emitter ===
emitter = chrono.ChParticleEmitter()
emitter.SetParticlesPerSecond(2000)
emitter.SetUseParticleReservoir(True)
emitter.SetParticleReservoirAmount(200)

# Position on a box geometry
emitter_positions = chrono.ChRandomParticlePositionOnGeometry()
emitter_positions.SetGeometry(chrono.ChBox(50, 50, 50), chrono.ChFramed())
emitter.SetParticlePositioner(emitter_positions)

# Uniform random alignment
emitter_rotations = chrono.ChRandomParticleAlignmentUniform()
emitter.SetParticleAligner(emitter_rotations)

# Velocity distribution
mvelo = chrono.ChRandomParticleVelocityAnyDirection()
mvelo.SetModulusDistribution(chrono.ChUniformDistribution(0.0, 0.5))
emitter.SetParticleVelocity(mvelo)

# Angular velocity distribution
mangvelo = chrono.ChRandomParticleVelocityAnyDirection()
mangvelo.SetModulusDistribution(chrono.ChUniformDistribution(0.0, 0.2))
emitter.SetParticleAngularVelocity(mangvelo)

# Sphere particle creator with diameter 0.6 (std 0.23) and density 1600
mcreator_spheres = chrono.ChRandomShapeCreatorSpheres()
mcreator_spheres.SetDiameterDistribution(chrono.ChZhangDistribution(0.6, 0.23))
mcreator_spheres.SetDensityDistribution(chrono.ChConstantDistribution(1600))
emitter.SetParticleCreator(mcreator_spheres)


# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Particle emitter demo")
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 14, -20), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()

# Register callback after vis is available
mcreation_callback = MyCreatorForAll(vis, coll)
emitter.RegisterAddBodyCallback(mcreation_callback)


# === Simulation parameters ===
time_step = 1e-2
sim_end = 10.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))


# === Main loop ===
frame = 0
csv_f = open("simulation_data.csv", "w", newline="")

while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    for _ in range(render_every):
        emitter.EmitParticles(sys, time_step)

        for body in sys.GetBodies():
            body.EmptyAccumulators()

        # G_constant moved inside the loop per input2 requirements
        G_constant = 6.674e-3

        # Calculate kinetic energy
        kinetic_energy = 0.0
        for body in sys.GetBodies():
            mass = body.GetMass()
            velocity = body.GetPosDt()
            kinetic_energy += 0.5 * mass * velocity.Length2()

        # Calculate gravitational potential energy
        potential_energy = 0.0
        for abodyA, abodyB in combinations(sys.GetBodies(), 2):
            D_attract = abodyB.GetPos() - abodyA.GetPos()
            r_attract = D_attract.Length()
            if r_attract > 1e-9:
                potential_energy += -G_constant * (abodyA.GetMass() * abodyB.GetMass()) / r_attract

        total_energy = kinetic_energy + potential_energy

        # Apply gravitational attraction forces
        for abodyA, abodyB in combinations(sys.GetBodies(), 2):
            D_attract = abodyB.GetPos() - abodyA.GetPos()
            r_attract = D_attract.Length()
            if r_attract > 1e-9:
                f_attract = G_constant * (abodyA.GetMass() * abodyB.GetMass()) / (r_attract ** 2)
                F_attract = (D_attract / r_attract) * f_attract
                abodyA.AccumulateForce(F_attract, abodyA.GetPos(), False)
                abodyB.AccumulateForce(-F_attract, abodyB.GetPos(), False)

        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
