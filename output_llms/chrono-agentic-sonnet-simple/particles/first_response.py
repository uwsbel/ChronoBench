import os
import pychrono as chrono
import pychrono.irrlicht as chronoirr
from itertools import combinations


class MyCreatorForAll(chrono.ChRandomShapeCreator_AddBodyCallback):
    def __init__(self, vis, coll):
        super().__init__()
        self.vis = vis
        self.coll = coll

    def OnAddBody(self, body, coords, creator):
        body.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))  # texture
        self.vis.BindItem(body)                                           # bind to renderer
        self.coll.BindItem(body)                                          # bind to collision
        body.SetUseGyroTorque(False)                                      # disable gyro


# Create a Chrono physical system
sys = chrono.ChSystemNSC()                                               # NSC for rigid contacts
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)          # Bullet collision required
coll = sys.GetCollisionSystem()                                           # collision system handle

# Contact material for the sphere
sphere_mat = chrono.ChContactMaterialNSC()                               # NSC material
sphere_mat.SetFriction(0.2)                                              # friction coefficient

# Central sphere body
msphereBody = chrono.ChBodyEasySphere(2.1, 1800, True, True, sphere_mat)  # r=2.1, density=1800
msphereBody.SetPos(chrono.ChVector3d(1, 1, 0))                           # spawn position
msphereBody.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))  # concrete texture
sys.Add(msphereBody)                                                      # add to system

# Create a particle emitter
emitter = chrono.ChParticleEmitter()                                      # particle emitter
emitter.SetParticlesPerSecond(2000)                                       # emission rate
emitter.SetUseParticleReservoir(True)                                     # cap total particles
emitter.SetParticleReservoirAmount(200)                                   # max 200 particles

# Random position on a bounding box
emitter_positions = chrono.ChRandomParticlePositionOnGeometry()           # positional randomizer
emitter_positions.SetGeometry(chrono.ChBox(50, 50, 50), chrono.ChFramed())  # 50x50x50 box domain
emitter.SetParticlePositioner(emitter_positions)                          # assign positioner

# Random orientation
emitter_rotations = chrono.ChRandomParticleAlignmentUniform()             # uniform random rotation
emitter.SetParticleAligner(emitter_rotations)                             # assign aligner

# Random linear velocity
mvelo = chrono.ChRandomParticleVelocityAnyDirection()                     # any-direction velocity
mvelo.SetModulusDistribution(chrono.ChUniformDistribution(0.0, 0.5))      # 0..0.5 m/s
emitter.SetParticleVelocity(mvelo)                                        # assign velocity

# Random angular velocity
mangvelo = chrono.ChRandomParticleVelocityAnyDirection()                  # any-direction angvel
mangvelo.SetModulusDistribution(chrono.ChUniformDistribution(0.0, 0.2))   # 0..0.2 rad/s
emitter.SetParticleAngularVelocity(mangvelo)                              # assign angular velocity

# Convex hull shape creator with random shape
mcreator_hulls = chrono.ChRandomShapeCreatorConvexHulls()                 # convex hull shapes
mcreator_hulls.SetNpoints(15)                                             # 15 hull vertices
mcreator_hulls.SetChordDistribution(chrono.ChZhangDistribution(1.3, 0.4))  # size distribution
mcreator_hulls.SetDensityDistribution(chrono.ChConstantDistribution(1600))  # density 1600 kg/m3
emitter.SetParticleCreator(mcreator_hulls)                                # assign creator

# Irrlicht visualization — Initialize first, then add scene elements
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Particle emitter demo')
vis.Initialize()                                                          # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))          # logo after Initialize
vis.AddSkyBox()                                                           # sky box
vis.AddCamera(chrono.ChVector3d(0, 14, -20))                              # camera position
vis.AddTypicalLights()                                                    # standard lights

# Attach creation callback (after vis is initialized so BindItem works)
mcreation_callback = MyCreatorForAll(vis, coll)                           # bind callback
emitter.RegisterAddBodyCallback(mcreation_callback)                       # register callback

# Solver settings
sys.SetSolverType(chrono.ChSolver.Type_PSOR)                              # PSOR solver
sys.GetSolver().AsIterative().SetMaxIterations(40)                        # 40 iterations
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))             # no external gravity

# Simulation settings
stepsize = 1e-2                                                           # physics time step [s]
sim_end = 20.0                                                            # simulation end [s]
render_fps = 50.0                                                         # target render fps
G_constant = 6.674e-3                                                     # modified gravitational constant
render_every = max(1, round(1.0 / (render_fps * stepsize)))               # render cadence (untagged)


while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    emitter.EmitParticles(sys, stepsize)                                  # emit particles each step

    for body in sys.GetBodies():                                          # clear accumulators
        body.EmptyAccumulators()

    mlist = list(combinations(sys.GetBodies(), 2))                        # all body pairs
    for abodyA, abodyB in mlist:
        D_attract = abodyB.GetPos() - abodyA.GetPos()                     # displacement vector
        r_attract = D_attract.Length()                                    # distance
        if r_attract < 1e-6:                                              # avoid division by zero
            continue
        f_attract = G_constant * (abodyA.GetMass() * abodyB.GetMass()) / (r_attract ** 2)  # gravity magnitude
        F_attract = (D_attract / r_attract) * f_attract                   # gravity force vector
        abodyA.AccumulateForce(F_attract, abodyA.GetPos(), False)         # apply to A
        abodyB.AccumulateForce(-F_attract, abodyB.GetPos(), False)        # apply to B (reaction)

    sys.DoStepDynamics(stepsize)                                          # advance simulation
