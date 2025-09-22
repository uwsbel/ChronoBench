# -------------------------------------------------------------------------
# PyChrono Particle-emitter demo  (corrected & updated)
# -------------------------------------------------------------------------
# 1.  convex-hull creator  --->  random-sphere creator
# 2.  on–line energy bookkeeping (kinetic + gravitational)
# 3.  G_constant moved inside the time loop
# -------------------------------------------------------------------------

import pychrono as chrono
import pychrono.irrlicht as chronoirr
from itertools import combinations


# -------------------------------------------------------------------------
# Utility callback: give a texture to each newly-created body and register it
# -------------------------------------------------------------------------
class MyCreatorForAll(chrono.ChRandomShapeCreator_AddBodyCallback):
    def __init__(self, vis, coll):
        super().__init__()
        self.vis = vis
        self.coll = coll

    def OnAddBody(self, body, coords, creator):
        # Make sure the visual asset exists before accessing it
        if body.GetVisualModel():
            body.GetVisualShape(0).SetTexture(
                chrono.GetChronoDataFile("textures/bluewhite.png")
            )

        # Tell Irrlicht and the collision system about the new body
        self.vis.BindItem(body)
        self.coll.BindItem(body)

        # Disable gyroscopic torques for a slightly cheaper simulation
        body.SetUseGyroTorque(False)


# -------------------------------------------------------------------------
#  Create the Chrono physical system
# -------------------------------------------------------------------------
sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
coll = sys.GetCollisionSystem()

# -------------------------------------------------------------------------
#  A massive sphere that will attract the particles
# -------------------------------------------------------------------------
sphere_mat = chrono.ChContactMaterialNSC()
sphere_mat.SetFriction(0.2)

big_sphere = chrono.ChBodyEasySphere(2.1,        # radius
                                     1800,       # density
                                     True, True, # collide?  visual?
                                     sphere_mat)
big_sphere.SetPos(chrono.ChVector3d(1, 1, 0))
big_sphere.GetVisualShape(0).SetTexture(
    chrono.GetChronoDataFile("textures/concrete.jpg")
)
sys.Add(big_sphere)

# -------------------------------------------------------------------------
#  Particle emitter
# -------------------------------------------------------------------------
emitter = chrono.ChParticleEmitter()
emitter.SetParticlesPerSecond(2000)
emitter.SetUseParticleReservoir(True)
emitter.SetParticleReservoirAmount(200)

# Where the particles are born ------------------------------------------------
pos_random = chrono.ChRandomParticlePositionOnGeometry()
pos_random.SetGeometry(chrono.ChBox(50, 50, 50), chrono.ChFramed())
emitter.SetParticlePositioner(pos_random)

# Random initial orientation --------------------------------------------------
align_random = chrono.ChRandomParticleAlignmentUniform()
emitter.SetParticleAligner(align_random)

# Random linear velocity ------------------------------------------------------
vel_random = chrono.ChRandomParticleVelocityAnyDirection()
vel_random.SetModulusDistribution(chrono.ChUniformDistribution(0.0, 0.5))
emitter.SetParticleVelocity(vel_random)

# Random angular velocity -----------------------------------------------------
avel_random = chrono.ChRandomParticleVelocityAnyDirection()
avel_random.SetModulusDistribution(chrono.ChUniformDistribution(0.0, 0.2))
emitter.SetParticleAngularVelocity(avel_random)

# -------------------------------------------------------------------------
# 1.  NEW – use spheres, not convex hulls
# -------------------------------------------------------------------------
mcreator_spheres = chrono.ChRandomShapeCreatorSpheres()
mcreator_spheres.SetDiameterDistribution(
    chrono.ChZhangDistribution(0.6, 0.23)      # mean & std-dev of diameter
)
mcreator_spheres.SetDensityDistribution(
    chrono.ChConstantDistribution(1600)        # kg/m^3
)
emitter.SetParticleCreator(mcreator_spheres)

# -------------------------------------------------------------------------
#  Irrlicht visualisation
# -------------------------------------------------------------------------
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Particle emitter demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 14, -20))
vis.AddTypicalLights()

# Register callback so that newly emitted particles obtain a texture, etc.
emitter.RegisterAddBodyCallback(MyCreatorForAll(vis, coll))

# -------------------------------------------------------------------------
#  Solver & gravity
# -------------------------------------------------------------------------
sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(40)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))   # we apply our own gravity

# -------------------------------------------------------------------------
#  Main time loop
# -------------------------------------------------------------------------
stepsize = 1e-2

while vis.Run():
    # ---- Irrlicht draw step -------------------------------------------------
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # ---- Emit any new particles -------------------------------------------
    emitter.EmitParticles(sys, stepsize)

    # ---- Clear forces from previous step -----------------------------------
    for body in sys.GetBodies():
        body.EmptyAccumulators()

    # ---- 3. Gravitational constant (now inside the loop) -------------------
    G_constant = 6.674e-3     # [arbitrary units]

    # ---- 2.  Energy bookkeeping & gravitational forces ---------------------
    kinetic_energy   = 0.0
    potential_energy = 0.0

    bodies = list(sys.GetBodies())

    # Kinetic part first
    for b in bodies:
        v = b.GetPos_dt()
        kinetic_energy += 0.5 * b.GetMass() * v.Length2()

    # Pair-wise forces and potential energy
    for bodyA, bodyB in combinations(bodies, 2):
        d_vec = bodyB.GetPos() - bodyA.GetPos()
        r     = d_vec.Length()
        if r == 0:
            continue

        # Gravitational force
        f_mag = G_constant * bodyA.GetMass() * bodyB.GetMass() / (r * r)
        F     = (d_vec / r) * f_mag

        bodyA.AccumulateForce( F, bodyA.GetPos(), False)
        bodyB.AccumulateForce(-F, bodyB.GetPos(), False)

        # Potential energy
        potential_energy += -G_constant * bodyA.GetMass() * bodyB.GetMass() / r

    total_energy = kinetic_energy + potential_energy
    print(f"KE = {kinetic_energy:10.4f} | PE = {potential_energy:10.4f} | "
          f"Total = {total_energy:10.4f}")

    # ---- Advance the system -------------------------------------------------
    sys.DoStepDynamics(stepsize)