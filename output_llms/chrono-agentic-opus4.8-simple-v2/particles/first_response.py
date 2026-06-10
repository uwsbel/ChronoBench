import pychrono as chrono                                             # core PyChrono
import pychrono.irrlicht as chronoirr                                 # Irrlicht renderer
from itertools import combinations                                    # pairwise body combinations for the force field


# A callback fired at every particle creation: bind the new body to the
# visualization + collision systems and tweak per-body settings.
class MyCreatorForAll(chrono.ChRandomShapeCreator_AddBodyCallback):
    def __init__(self, vis, coll):
        chrono.ChRandomShapeCreator_AddBodyCallback.__init__(self)
        self.vis = vis                                               # Irrlicht visual system
        self.coll = coll                                            # collision system

    def OnAddBody(self, body, coords, creator):
        body.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))  # texture each particle
        self.vis.BindItem(body)                                     # register body with Irrlicht
        self.coll.BindItem(body)                                    # register body with collision system
        body.SetUseGyroTorque(False)                               # disable gyroscopic torque for integrator stability


sys = chrono.ChSystemNSC()                                           # non-smooth contact system
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)     # Bullet collision (particles collide)
coll = sys.GetCollisionSystem()                                      # handle for binding emitted bodies

sphere_mat = chrono.ChContactMaterialNSC()                           # contact material for the central sphere
sphere_mat.SetFriction(0.2)                                          # friction coefficient

msphereBody = chrono.ChBodyEasySphere(2.1,                           # radius
                                      1800,                          # density
                                      True,                          # visualize
                                      True,                          # collide
                                      sphere_mat)                    # contact material
msphereBody.SetPos(chrono.ChVector3d(1, 1, 0))                       # offset slightly from origin
msphereBody.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))  # texture the sphere
sys.Add(msphereBody)                                                 # add the seed body to the system

emitter = chrono.ChParticleEmitter()                                 # the particle-flow generator
emitter.SetParticlesPerSecond(2000)                                  # emission flow rate
emitter.SetUseParticleReservoir(True)                                # cap the total emitted count
emitter.SetParticleReservoirAmount(200)                              # reservoir = 200 particles

# ---Randomizer for POSITIONS: random points inside a large cube
emitter_positions = chrono.ChRandomParticlePositionOnGeometry()      # positions sampled on a geometry
sampled_cube = chrono.ChBox(50, 50, 50)                              # 50x50x50 sampling box
emitter_positions.SetGeometry(sampled_cube, chrono.ChFramed())       # attach the sampling geometry
emitter.SetParticlePositioner(emitter_positions)                     # register the positioner

# ---Randomizer for ALIGNMENTS: uniformly random orientations
emitter_rotations = chrono.ChRandomParticleAlignmentUniform()        # uniform random rotation
emitter.SetParticleAligner(emitter_rotations)                        # register the aligner

# ---Randomizer for VELOCITIES: any direction, magnitude in [0, 0.5]
mvelo = chrono.ChRandomParticleVelocityAnyDirection()                # isotropic linear velocity
mvelo.SetModulusDistribution(chrono.ChUniformDistribution(0.0, 0.5)) # speed distribution
emitter.SetParticleVelocity(mvelo)                                   # register the velocity randomizer

# ---Randomizer for ANGULAR VELOCITIES: any direction, magnitude in [0, 0.2]
mangvelo = chrono.ChRandomParticleVelocityAnyDirection()             # isotropic angular velocity
mangvelo.SetModulusDistribution(chrono.ChUniformDistribution(0.0, 0.2))  # angular-speed distribution
emitter.SetParticleAngularVelocity(mangvelo)                         # register the angular-velocity randomizer

# ---Randomizer for CREATED SHAPES: faceted convex-hull fragments
mcreator_hulls = chrono.ChRandomShapeCreatorConvexHulls()            # random convex-hull shapes
mcreator_hulls.SetNpoints(15)                                        # points per hull
mcreator_hulls.SetChordDistribution(chrono.ChZhangDistribution(1.3, 0.4))  # size distribution
mcreator_hulls.SetDensityDistribution(chrono.ChConstantDistribution(1600)) # constant density
emitter.SetParticleCreator(mcreator_hulls)                           # register the shape creator

vis = chronoirr.ChVisualSystemIrrlicht()                             # Irrlicht visual system
vis.AttachSystem(sys)                                                # bind the physical system
vis.SetWindowSize(1024, 768)                                         # window dimensions
vis.SetWindowTitle('Particle emitter demo')                         # window title
vis.Initialize()                                                     # create the device first
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))     # branding logo
vis.AddSkyBox()                                                      # sky box
vis.AddCamera(chrono.ChVector3d(0, 14, -20))                         # camera eye position
vis.AddTypicalLights()                                               # standard two-light rig

# Attach the per-particle creation callback so every emitted body is bound to viz + collision.
mcreation_callback = MyCreatorForAll(vis, coll)                      # the callback object
emitter.RegisterAddBodyCallback(mcreation_callback)                 # register it on the emitter

sys.SetSolverType(chrono.ChSolver.Type_PSOR)                         # iterative PSOR solver
sys.GetSolver().AsIterative().SetMaxIterations(40)                  # solver iteration cap

sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))         # disable default gravity; custom force field instead

stepsize = 1e-2                                                      # integration step
sim_end = 20.0                                                       # stop time [s]
render_fps = 50.0                                                    # review frame rate
render_every = max(1, round(1.0 / (render_fps * stepsize)))         # untagged cadence constant
G_constant = 6.674e-3                                                # gravitational constant (scaled up to speed the demo)

while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        emitter.EmitParticles(sys, stepsize)                        # create the particle flow this step

        # Apply custom gravitational attraction between all body pairs (brute force).
        for body in sys.GetBodies():                                # A) clear user-force accumulators
            body.EmptyAccumulators()

        mlist = list(combinations(sys.GetBodies(), 2))              # all unordered body pairs
        for bodycomb in mlist:                                      # B) accumulate pairwise attraction
            abodyA = bodycomb[0]
            abodyB = bodycomb[1]
            D_attract = abodyB.GetPos() - abodyA.GetPos()           # separation vector A->B
            r_attract = D_attract.Length()                         # distance between bodies
            f_attract = G_constant * (abodyA.GetMass() * abodyB.GetMass()) / (pow(r_attract, 2))  # Newton's law
            F_attract = (D_attract / r_attract) * f_attract        # force vector toward each other
            abodyA.AccumulateForce(F_attract, abodyA.GetPos(), False)   # equal/opposite on the pair
            abodyB.AccumulateForce(-F_attract, abodyB.GetPos(), False)

        sys.DoStepDynamics(stepsize)                                # advance the dynamics one step
        if sys.GetChTime() >= sim_end:
            break
