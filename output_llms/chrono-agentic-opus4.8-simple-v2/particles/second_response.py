import os
import pychrono as chrono
import pychrono.irrlicht as chronoirr
from itertools import combinations


class MyCreatorForAll(chrono.ChRandomShapeCreator_AddBodyCallback):     # per-particle creation callback
    def __init__(self, vis, coll):
        chrono.ChRandomShapeCreator_AddBodyCallback.__init__(self)
        self.vis = vis                                                 # Irrlicht visual system
        self.coll = coll                                               # collision system

    def OnAddBody(self, body, coords, creator):
        body.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))  # tint new particle
        self.vis.BindItem(body)                                        # bind to Irrlicht so it renders
        self.coll.BindItem(body)                                       # bind to collision system
        body.SetUseGyroTorque(False)                                   # disable gyro torque for integrator stability


sys = chrono.ChSystemNSC()                                             # non-smooth contact rigid-body system
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)       # Bullet narrowphase (contact present)
coll = sys.GetCollisionSystem()                                        # handle for binding new particles

sphere_mat = chrono.ChContactMaterialNSC()                             # contact material for the seed body
sphere_mat.SetFriction(0.2)                                            # friction coefficient

msphereBody = chrono.ChBodyEasySphere(2.1, 1800, True, True, sphere_mat)  # radius, density, vis, collide, mat
msphereBody.SetPos(chrono.ChVector3d(1, 1, 0))                         # seed sphere offset from origin
msphereBody.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))  # texture
sys.Add(msphereBody)                                                   # add the seed attractor body

emitter = chrono.ChParticleEmitter()                                   # cluster particle emitter
emitter.SetParticlesPerSecond(2000)                                    # emission flow rate
emitter.SetUseParticleReservoir(True)                                  # cap total emitted particles
emitter.SetParticleReservoirAmount(200)                                # reservoir size

emitter_positions = chrono.ChRandomParticlePositionOnGeometry()        # random positions on a geometry
sampled_cube = chrono.ChBox(50, 50, 50)                                # large sampling cube
emitter_positions.SetGeometry(sampled_cube, chrono.ChFramed())         # sample within the cube at origin
emitter.SetParticlePositioner(emitter_positions)                       # attach positioner

emitter_rotations = chrono.ChRandomParticleAlignmentUniform()          # uniform random alignment
emitter.SetParticleAligner(emitter_rotations)                          # attach aligner

mvelo = chrono.ChRandomParticleVelocityAnyDirection()                  # random linear velocity, any direction
mvelo.SetModulusDistribution(chrono.ChUniformDistribution(0.0, 0.5))   # speed magnitude distribution
emitter.SetParticleVelocity(mvelo)                                     # attach velocity randomizer

mangvelo = chrono.ChRandomParticleVelocityAnyDirection()               # random angular velocity, any direction
mangvelo.SetModulusDistribution(chrono.ChUniformDistribution(0.0, 0.2))  # angular speed distribution
emitter.SetParticleAngularVelocity(mangvelo)                           # attach angular-velocity randomizer

mcreator_spheres = chrono.ChRandomShapeCreatorSpheres()                # spherical-particle shape creator
mcreator_spheres.SetDiameterDistribution(chrono.ChZhangDistribution(0.6, 0.23))  # Zhang diameter distribution
mcreator_spheres.SetDensityDistribution(chrono.ChConstantDistribution(1600))     # constant density 1600 kg/m^3
emitter.SetParticleCreator(mcreator_spheres)                           # use sphere creator for the flow

vis = chronoirr.ChVisualSystemIrrlicht()                               # Irrlicht visualization
vis.AttachSystem(sys)                                                  # bind the physical system
vis.SetWindowSize(1024, 768)                                           # window dimensions
vis.SetWindowTitle('Particle emitter demo')                           # window title
vis.Initialize()                                                       # create the device FIRST
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))       # logo overlay
vis.AddSkyBox()                                                        # sky box
vis.AddCamera(chrono.ChVector3d(0, 14, -20))                           # eye position
vis.AddTypicalLights()                                                 # standard two-light setup

mcreation_callback = MyCreatorForAll(vis, coll)                        # bind callback to vis + collision
emitter.RegisterAddBodyCallback(mcreation_callback)                    # fire on every created particle

sys.SetSolverType(chrono.ChSolver.Type_PSOR)                           # projected SOR solver
sys.GetSolver().AsIterative().SetMaxIterations(40)                     # solver iteration cap

sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))           # turn off downward gravity (N-body field)

stepsize = 1e-2                                                        # integration step size
sim_end = 8.0                                                          # total simulated time
render_fps = 50.0                                                      # target frames per second
render_every = max(1, round(1.0 / (render_fps * stepsize)))           # physics steps per rendered frame
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()                                                  # open the frame
    vis.Render()                                                      # draw the scene
    vis.EndScene()                                                    # finish the frame
    for _ in range(render_every):
        emitter.EmitParticles(sys, stepsize)                          # spawn the particle flow

        for body in sys.GetBodies():                                  # A) clear user force accumulators
            body.EmptyAccumulators()

        G_constant = 6.674e-3                                         # gravitational constant (scaled for speed)
        mlist = list(combinations(sys.GetBodies(), 2))               # all unique body pairs

        kinetic = 0.0                                                # total kinetic energy
        for body in sys.GetBodies():                                 # KE = sum 0.5 m |v|^2
            kinetic += 0.5 * body.GetMass() * body.GetPosDt().Length2()

        potential = 0.0                                              # total gravitational potential energy
        for bodycomb in mlist:                                       # B) pairwise attraction + PE
            abodyA = bodycomb[0]
            abodyB = bodycomb[1]
            D_attract = abodyB.GetPos() - abodyA.GetPos()            # vector A -> B
            r_attract = D_attract.Length()                          # separation distance
            f_attract = G_constant * (abodyA.GetMass() * abodyB.GetMass()) / (pow(r_attract, 2))  # force magnitude
            F_attract = (D_attract / r_attract) * f_attract         # force vector along the line
            abodyA.AccumulateForce(F_attract, abodyA.GetPos(), False)   # pull A toward B
            abodyB.AccumulateForce(-F_attract, abodyB.GetPos(), False)  # pull B toward A
            potential += -G_constant * (abodyA.GetMass() * abodyB.GetMass()) / r_attract  # PE of the pair

        total = kinetic + potential                                 # total mechanical energy
        print("Kinetic: ", kinetic, " Potential: ", potential, " Total: ", total)  # per-step energy log

        sys.DoStepDynamics(stepsize)                                 # advance one step
        if sys.GetChTime() >= sim_end:
            break
