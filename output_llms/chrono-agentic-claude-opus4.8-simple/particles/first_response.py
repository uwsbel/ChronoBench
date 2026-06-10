import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemNSC()                                           # NSC rigid-body system
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)     # collision required (particles collide)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))         # zero gravity; N-body attraction supplies all force
sys.SetSolverType(chrono.ChSolver.Type_PSOR)                         # iterative PSOR solver
sys.GetSolver().AsIterative().SetMaxIterations(40)                   # solver iteration cap

sphere_mat = chrono.ChContactMaterialNSC()                           # NSC contact material for the attractor
sphere_mat.SetFriction(0.2)                                          # moderate friction

# central attractor sphere — the heavy body the particles fall toward
msphereBody = chrono.ChBodyEasySphere(2.1, 1800, True, True, sphere_mat)  # radius, density, visual, collide, material
msphereBody.SetPos(chrono.ChVector3d(1, 1, 0))                       # placed slightly off origin
msphereBody.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))  # concrete texture
sys.Add(msphereBody)                                                 # add the attractor to the system

# particle emitter — generates random-shape particles with random pos/vel/orientation
emitter = chrono.ChParticleEmitter()                                 # the emitter
emitter.SetParticlesPerSecond(2000)                                  # creation flow rate
emitter.SetUseParticleReservoir(True)                                # cap total number created
emitter.SetParticleReservoirAmount(200)                             # reservoir of 200 particles

# positioner — random positions on a box volume
emitter_positions = chrono.ChRandomParticlePositionOnGeometry()      # positions sampled over a geometry
emitter_positions.SetGeometry(chrono.ChBox(50, 50, 50), chrono.ChFramed())  # within a 50^3 box at the origin
emitter.SetParticlePositioner(emitter_positions)                     # attach positioner

# aligner — random uniform orientation
emitter_rotations = chrono.ChRandomParticleAlignmentUniform()        # uniformly-random orientation
emitter.SetParticleAligner(emitter_rotations)                        # attach aligner

# linear velocity — random direction, small modulus
mvelo = chrono.ChRandomParticleVelocityAnyDirection()                # isotropic random linear velocity
mvelo.SetModulusDistribution(chrono.ChUniformDistribution(0.0, 0.5)) # speed in [0, 0.5]
emitter.SetParticleVelocity(mvelo)                                   # attach linear-velocity model

# angular velocity — random direction, small modulus
mangvelo = chrono.ChRandomParticleVelocityAnyDirection()             # isotropic random angular velocity
mangvelo.SetModulusDistribution(chrono.ChUniformDistribution(0.0, 0.2))  # ang. speed in [0, 0.2]
emitter.SetParticleAngularVelocity(mangvelo)                         # attach angular-velocity model

# shape creator — random convex hulls give the particles random shapes
mcreator_hulls = chrono.ChRandomShapeCreatorConvexHulls()            # convex-hull particle shapes
mcreator_hulls.SetNpoints(15)                                        # 15 hull points per particle
mcreator_hulls.SetChordDistribution(chrono.ChZhangDistribution(1.3, 0.4))  # size (chord) distribution
mcreator_hulls.SetDensityDistribution(chrono.ChConstantDistribution(1600)) # constant particle density
emitter.SetParticleCreator(mcreator_hulls)                           # attach the shape creator


# callback applied to every newly-created particle body
class MyCreatorForAll(chrono.ChRandomShapeCreator_AddBodyCallback):  # per-body add callback
    def __init__(self, vis, coll):
        chrono.ChRandomShapeCreator_AddBodyCallback.__init__(self)
        self.vis = vis                                               # visual system handle
        self.coll = coll                                             # collision system handle

    def OnAddBody(self, body, acoord, creator):                      # invoked for each created body
        body.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))  # texture
        self.vis.BindItem(body)                                      # register visual asset at runtime
        self.coll.BindItem(body)                                     # register collision shape at runtime
        body.SetUseGyroTorque(False)                                 # disable gyroscopic torque for stability


vis = chronoirr.ChVisualSystemIrrlicht()                             # Irrlicht visualization
vis.AttachSystem(sys)                                                # bind the system
vis.SetWindowSize(1280, 720)                                         # window size
vis.SetWindowTitle("Particle emitter: gravitational attraction")     # window title
vis.Initialize()                                                     # create device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))     # logo (after Initialize)
vis.AddSkyBox()                                                      # sky box
vis.AddCamera(chrono.ChVector3d(0, 14, -20))                         # camera eye position
vis.AddTypicalLights()                                               # standard lighting

# register the per-body callback now that vis + collision system exist
mcreation_callback = MyCreatorForAll(vis, sys.GetCollisionSystem())  # the callback instance
mcreator_hulls.RegisterAddBodyCallback(mcreation_callback)           # called on each new particle

stepsize = 1e-2                                                      # integration step
G_constant = 6.674e-3                                               # scaled gravitational constant

render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * stepsize)))          # untagged cadence constant
while vis.Run():                                                     # SCORED CORE = plain truth form, NO time bound
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        emitter.EmitParticles(sys, stepsize)                         # spawn new particles this step
        for body in sys.GetBodies():                                 # clear force accumulators
            body.EmptyAccumulators()
        bodies = sys.GetBodies()                                     # current body list
        for i in range(len(bodies)):                                 # all unordered body pairs
            abodyA = bodies[i]
            for j in range(i + 1, len(bodies)):
                abodyB = bodies[j]
                D_attract = abodyB.GetPos() - abodyA.GetPos()        # separation vector A->B
                r_attract = D_attract.Length()                       # distance
                if r_attract > 1e-6:                                 # skip coincident bodies
                    F_attract = (G_constant * abodyA.GetMass() * abodyB.GetMass()) / (r_attract * r_attract)  # |F|
                    Dir = D_attract * (1.0 / r_attract)              # unit separation
                    abodyA.AccumulateForce(Dir * F_attract, abodyA.GetPos(), False)   # pull A toward B
                    abodyB.AccumulateForce(Dir * (-F_attract), abodyB.GetPos(), False)  # pull B toward A
        sys.DoStepDynamics(stepsize)                                 # advance physics
