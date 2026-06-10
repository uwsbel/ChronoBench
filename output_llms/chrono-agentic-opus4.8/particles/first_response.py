"""Gravitational N-body particle flow with a random-shape emitter (PyChrono, NSC).

Models the mutual gravitational attraction between a cloud of rigid particles.
A ChParticleEmitter continuously spawns particles with random shapes (spheres,
boxes, cylinders), random positions on a rectangular outlet, random orientations,
and random velocities. The global gravity field is disabled; instead a custom
pairwise (Newtonian) gravitational force is accumulated on every particle each
step, so the cloud attracts and clumps together dynamically.

System type: ChSystemNSC (non-smooth rigid contact, PSOR solver).
Main bodies : a fixed floor box + the emitted particle bodies.
Expected behavior: particles stream from the outlet and pull toward one another
under the custom inverse-square attraction, forming drifting clusters.
"""

import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# === Named constants === geometry / physics / flow parameters
time_step = 0.01                 # integrator step [s]
sim_end = 12.0                   # simulation duration [s]
render_fps = 50.0                # review video frame rate
grav_constant = 40.0             # scaled gravitational constant for the custom N-body force
soften = 0.6                     # softening length [m] to bound the 1/r^2 force at close range
emit_speed = 0.2                 # small emission speed [m/s] so attraction dominates
particles_per_second = 22.0      # emitter flow rate [particles/s]
reservoir = 160                  # cap on total emitted particles
outlet_pos = chrono.ChVector3d(0, 2.0, 0)   # rectangular outlet center
outlet_w = 4.0                   # outlet width  [m]
outlet_h = 4.0                   # outlet height [m]
render_every = max(1, round(1.0 / (render_fps * time_step)))   # precomputed once


# === System & gravity === NSC rigid system; global gravity OFF (custom force drives motion)
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))   # custom pairwise attraction instead
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # required: particles collide
sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(40)

# === Bodies === fixed floor so settled particles have a support plane
floor_mat = chrono.ChContactMaterialNSC()
floor_mat.SetFriction(0.3)
floor = chrono.ChBodyEasyBox(20, 1, 20, 1000, True, True, floor_mat)
floor.SetPos(chrono.ChVector3d(0, -6, 0))
floor.SetFixed(True)
floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(floor)

# === Emitter === four randomizers: shape creator, positioner, aligner, velocity
emitter = chrono.ChParticleEmitter()
emitter.SetParticlesPerSecond(particles_per_second)
emitter.SetUseParticleReservoir(True)
emitter.SetParticleReservoirAmount(reservoir)

# Positioner: random positions over a rectangular outlet patch (X-Z plane at outlet height)
class RectangleOutletPositioner(chrono.ChRandomParticlePosition):
    def RandomPosition(self):
        x = outlet_pos.x + (chrono.ChRandom.Get() - 0.5) * outlet_w
        z = outlet_pos.z + (chrono.ChRandom.Get() - 0.5) * outlet_h
        return chrono.ChVector3d(x, outlet_pos.y, z)


emitter_positions = RectangleOutletPositioner()
emitter.SetParticlePositioner(emitter_positions)

# Aligner: uniformly-random orientations
emitter.SetParticleAligner(chrono.ChRandomParticleAlignmentUniform())

# Velocity: random direction, modest speed so particles spread before attracting
emitter_velocity = chrono.ChRandomParticleVelocityAnyDirection()
emitter_velocity.SetModulusDistribution(chrono.ChUniformDistribution(0.0, emit_speed))
emitter.SetParticleVelocity(emitter_velocity)

# Shape creator: a family mix of spheres, boxes and cylinders (random shapes)
creator_spheres = chrono.ChRandomShapeCreatorSpheres()
creator_spheres.SetDiameterDistribution(chrono.ChUniformDistribution(0.3, 0.6))
creator_spheres.SetDensityDistribution(chrono.ChConstantDistribution(1000))

creator_boxes = chrono.ChRandomShapeCreatorBoxes()
creator_boxes.SetXsizeDistribution(chrono.ChUniformDistribution(0.3, 0.6))
creator_boxes.SetSizeRatioZDistribution(chrono.ChUniformDistribution(0.4, 1.0))
creator_boxes.SetSizeRatioYZDistribution(chrono.ChUniformDistribution(0.4, 1.0))
creator_boxes.SetDensityDistribution(chrono.ChConstantDistribution(1000))

creator_cylinders = chrono.ChRandomShapeCreatorCylinders()
creator_cylinders.SetDiameterDistribution(chrono.ChUniformDistribution(0.3, 0.5))
creator_cylinders.SetLengthFactorDistribution(chrono.ChUniformDistribution(0.8, 1.6))
creator_cylinders.SetDensityDistribution(chrono.ChConstantDistribution(1000))

creator = chrono.ChRandomShapeCreatorFromFamilies()
creator.AddFamily(creator_spheres, 1.0)
creator.AddFamily(creator_boxes, 1.0)
creator.AddFamily(creator_cylinders, 1.0)
creator.SetAddCollisionShape(True)
creator.SetAddVisualizationAsset(True)
creator.Setup()


# Per-particle creation callback: give each new particle a random color.
class ColorCreatorCallback(chrono.ChRandomShapeCreator_AddBodyCallback):
    def OnAddBody(self, body, coords, shape_creator):
        body.GetVisualShape(0).SetColor(
            chrono.ChColor(float(chrono.ChRandom.Get()), 0.4, float(chrono.ChRandom.Get())))


color_cb = ColorCreatorCallback()
creator.RegisterAddBodyCallback(color_cb)
emitter.SetParticleCreator(creator)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)   # Y-up scene (outlet along +Y)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Gravitational particle emitter")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(5, 7, -12), chrono.ChVector3d(0, 1, 0))
vis.AddTypicalLights()
vis.AddGrid(1.0, 1.0, 30, 30,
            chrono.ChCoordsysd(chrono.ChVector3d(0, -5.5, 0), chrono.QuatFromAngleX(chrono.CH_PI_2)),
            chrono.ChColor(0.4, 0.4, 0.4))


# === Emitter binding === bind every newly emitted particle to the Irrlicht visual system
class BindToVisualCallback(chrono.ChRandomShapeCreator_AddBodyCallback):
    def OnAddBody(self, body, coords, shape_creator):
        vis.BindItem(body)                 # make the new particle render in the window
        body.SetUseGyroTorque(False)       # drop gyroscopic torque for integrator stability


bind_cb = BindToVisualCallback()
emitter.RegisterAddBodyCallback(bind_cb)


# === Custom gravitational attraction === pairwise inverse-square force, applied each step
def apply_gravitational_attraction():
    bodies = [b for b in sys.GetBodies() if not b.IsFixed()]   # cache: dynamic particles only
    n = len(bodies)
    for b in bodies:
        b.EmptyAccumulators()
    for i in range(n):
        bi = bodies[i]
        pi = bi.GetPos()
        mi = bi.GetMass()
        for j in range(i + 1, n):
            bj = bodies[j]
            d = bj.GetPos() - pi
            r2 = d.Length2() + soften * soften       # softened squared distance
            inv_r = 1.0 / math.sqrt(r2)
            dir_ij = d * inv_r                        # unit vector i -> j
            fmag = grav_constant * mi * bj.GetMass() / r2
            force = dir_ij * fmag
            bi.AccumulateForce(force, pi, False)       # pull i toward j
            bj.AccumulateForce(-force, bj.GetPos(), False)  # equal & opposite on j


# === Main loop === emit particles, apply attraction, step physics
frame = 0

try:

    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            emitter.EmitParticles(sys, time_step)
            apply_gravitational_attraction()
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state during stepping
    import traceback
    traceback.print_exc()
    raise

# === Post-processing === close writer, assemble review video + timeseries plot, prune frames
