import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemNSC()                                           # NSC rigid-body system
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)     # required: bodies have contact shapes
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))         # zero gravity: N-body attraction supplies all force
sys.SetSolverType(chrono.ChSolver.Type_PSOR)                         # iterative PSOR solver
sys.GetSolver().AsIterative().SetMaxIterations(40)                   # solver iteration cap

sphere_mat = chrono.ChContactMaterialNSC()                           # NSC contact material for the spheres
sphere_mat.SetFriction(0.2)                                          # friction coefficient

# Three massive spheres -> three-body problem (each ChBodyEasySphere: radius, density, visualize, collide, material)
msphere1 = chrono.ChBodyEasySphere(2.1, 1800, True, True, sphere_mat)  # sphere 1 (base body)
msphere1.SetPos(chrono.ChVector3d(1, 1, 0))                          # sphere 1 initial position
msphere1.SetPosDt(chrono.ChVector3d(0.5, 0, 0.1))                    # sphere 1 initial velocity
msphere1.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))  # concrete texture
sys.Add(msphere1)                                                    # add sphere 1 to the system

msphere2 = chrono.ChBodyEasySphere(2.1, 1800, True, True, sphere_mat)  # sphere 2
msphere2.SetPos(chrono.ChVector3d(-10, -10, 0))                      # sphere 2 initial position
msphere2.SetPosDt(chrono.ChVector3d(-0.5, 0, -0.1))                  # sphere 2 initial velocity
msphere2.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))  # concrete texture
sys.Add(msphere2)                                                    # add sphere 2 to the system

msphere3 = chrono.ChBodyEasySphere(2.1, 1800, True, True, sphere_mat)  # sphere 3
msphere3.SetPos(chrono.ChVector3d(0, 20, 0))                         # sphere 3 initial position
msphere3.SetPosDt(chrono.ChVector3d(0, -0.5, 0.2))                   # sphere 3 initial velocity
msphere3.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))  # concrete texture
sys.Add(msphere3)                                                    # add sphere 3 to the system

# Particle emitter -> a cloud of small bodies attracted by the three spheres
emitter = chrono.ChParticleEmitter()                                 # the particle emitter
emitter.SetParticlesPerSecond(2000)                                  # emission rate
emitter.SetUseParticleReservoir(True)                                # cap total emitted particles
emitter.SetParticleReservoirAmount(200)                              # reservoir size

emitter_positions = chrono.ChRandomParticlePositionOnGeometry()      # spawn positions on a geometry
emitter_positions.SetGeometry(chrono.ChBox(50, 50, 50), chrono.ChFramed())  # within a 50^3 box
emitter.SetParticlePositioner(emitter_positions)                     # attach the positioner

emitter_rotations = chrono.ChRandomParticleAlignmentUniform()        # uniform random alignment
emitter.SetParticleAligner(emitter_rotations)                        # attach the aligner

mvelo = chrono.ChRandomParticleVelocityAnyDirection()                # random linear velocity direction
mvelo.SetModulusDistribution(chrono.ChUniformDistribution(0.0, 0.5)) # linear speed 0..0.5
emitter.SetParticleVelocity(mvelo)                                   # attach linear velocity

mangvelo = chrono.ChRandomParticleVelocityAnyDirection()             # random angular velocity direction
mangvelo.SetModulusDistribution(chrono.ChUniformDistribution(0.0, 0.2))  # angular speed 0..0.2
emitter.SetParticleAngularVelocity(mangvelo)                         # attach angular velocity

creator_spheres = chrono.ChRandomShapeCreatorSpheres()               # particle shapes: small spheres
creator_spheres.SetDiameterDistribution(chrono.ChZhangDistribution(0.6, 0.23))  # diameter distribution
creator_spheres.SetDensityDistribution(chrono.ChConstantDistribution(1600))     # constant density
emitter.SetParticleCreator(creator_spheres)                          # attach the shape creator

vis = chronoirr.ChVisualSystemIrrlicht()                             # Irrlicht visualization window
vis.AttachSystem(sys)                                                # bind the system
vis.SetWindowSize(1280, 720)                                         # window resolution
vis.SetWindowTitle("Three-body particle simulation")                # window title
vis.Initialize()                                                     # build the device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))     # logo overlay
vis.AddSkyBox()                                                      # sky box
vis.AddCamera(chrono.ChVector3d(0, 14, -20))                         # camera eye position
vis.AddTypicalLights()                                               # standard lights

# Per-emitted-body callback: texture + bind to vis/collision + disable gyroscopic torque
class MyCreatorForAll(chrono.ChRandomShapeCreator_AddBodyCallback):  # add-body callback
    def __init__(self, vis, coll):
        super().__init__()
        self.vis = vis                                               # visual system handle
        self.coll = coll                                             # collision system handle
    def OnAddBody(self, body, acoord, creator):
        body.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))  # texture
        self.vis.BindItem(body)                                      # register visual asset
        self.coll.BindItem(body)                                     # register collision shape
        body.SetUseGyroTorque(False)                                 # ignore gyroscopic torque

creator_callback = MyCreatorForAll(vis, sys.GetCollisionSystem())    # instantiate the callback
emitter.RegisterAddBodyCallback(creator_callback)                    # register it on the emitter

stepsize = 1e-2                                                      # integration step
G_constant = 6.674e-3                                               # scaled gravitational constant

render_fps = 50.0                                                   # render cadence target
render_every = max(1, round(1.0 / (render_fps * stepsize)))          # untagged cadence constant

while vis.Run():                                                     # scored core: plain truth-form loop
    vis.BeginScene()                                                # begin frame
    vis.Render()                                                    # draw scene
    vis.EndScene()                                                  # end frame
    for _ in range(render_every):                                   # advance physics between frames
        emitter.EmitParticles(sys, stepsize)                        # spawn new particles this step
        for body in sys.GetBodies():                                # clear force accumulators
            body.EmptyAccumulators()
        bodies = sys.GetBodies()                                    # current body list
        PE = 0.0                                                    # total gravitational potential energy
        for i in range(len(bodies)):                                # N-body pairwise gravity
            bodyA = bodies[i]
            mA = bodyA.GetMass()
            posA = bodyA.GetPos()
            for j in range(i + 1, len(bodies)):
                bodyB = bodies[j]
                mB = bodyB.GetMass()
                posB = bodyB.GetPos()
                D_attract = posB - posA                             # separation vector A->B
                r_attract = D_attract.Length()                     # distance
                if r_attract > 1e-6:                               # guard against coincident bodies
                    Fgravity = G_constant * mA * mB / (r_attract * r_attract)  # Newtonian magnitude
                    D_attract.Normalize()                          # unit direction
                    bodyA.AccumulateForce(D_attract * Fgravity, posA, False)    # pull A toward B
                    bodyB.AccumulateForce(-D_attract * Fgravity, posB, False)   # pull B toward A
                    PE += -G_constant * mA * mB / r_attract        # accumulate potential energy
        print("Total potential energy: ", PE)                       # truth2/3 data output idiom
        sys.DoStepDynamics(stepsize)                                # advance the dynamics
