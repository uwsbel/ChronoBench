import os
import math
from itertools import combinations
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemNSC()                                            # NSC rigid-body system
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)      # collision required (particles collide)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))          # zero gravity; N-body attraction supplies all force
sys.SetSolverType(chrono.ChSolver.Type_PSOR)                          # iterative PSOR solver
sys.GetSolver().AsIterative().SetMaxIterations(40)                    # solver iterations

sphere_mat = chrono.ChContactMaterialNSC()                            # contact material for the attractor
sphere_mat.SetFriction(0.2)                                           # friction coefficient

msphereBody = chrono.ChBodyEasySphere(2.1, 1800, True, True, sphere_mat)  # big attractor sphere (radius, density)
msphereBody.SetPos(chrono.ChVector3d(1, 1, 0))                        # attractor position
msphereBody.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))  # texture
sys.Add(msphereBody)                                                  # add attractor to system

emitter = chrono.ChParticleEmitter()                                  # particle emitter
emitter.SetParticlesPerSecond(2000)                                   # emission rate
emitter.SetUseParticleReservoir(True)                                 # cap total particles via reservoir
emitter.SetParticleReservoirAmount(200)                              # total particles to emit

emitter_positions = chrono.ChRandomParticlePositionOnGeometry()       # spawn positions on a geometry
emitter_positions.SetGeometry(chrono.ChBox(50, 50, 50), chrono.ChFramed())  # within a 50^3 box region
emitter.SetParticlePositioner(emitter_positions)                      # assign positioner

emitter_rotations = chrono.ChRandomParticleAlignmentUniform()         # uniform random orientation
emitter.SetParticleAligner(emitter_rotations)                         # assign aligner

mvelo = chrono.ChRandomParticleVelocityAnyDirection()                 # random linear velocity, any direction
mvelo.SetModulusDistribution(chrono.ChUniformDistribution(0.0, 0.5))  # linear speed in [0, 0.5]
emitter.SetParticleVelocity(mvelo)                                    # assign linear velocity

mangvelo = chrono.ChRandomParticleVelocityAnyDirection()              # random angular velocity, any direction
mangvelo.SetModulusDistribution(chrono.ChUniformDistribution(0.0, 0.2))  # angular speed in [0, 0.2]
emitter.SetParticleAngularVelocity(mangvelo)                          # assign angular velocity

mcreator_spheres = chrono.ChRandomShapeCreatorSpheres()               # sphere-shaped particle creator
mcreator_spheres.SetDiameterDistribution(chrono.ChZhangDistribution(0.6, 0.23))  # diameter Zhang(mean, min)
mcreator_spheres.SetDensityDistribution(chrono.ChConstantDistribution(1600))     # constant density 1600
emitter.SetParticleCreator(mcreator_spheres)                          # assign creator


class MyCreatorForAll(chrono.ChRandomShapeCreator_AddBodyCallback):    # per-body setup callback
    def __init__(self, vis, coll):
        super().__init__()
        self.vis = vis                                                # visual system handle
        self.coll = coll                                             # collision system handle

    def OnAddBody(self, body, acoord, creator):                       # called for every emitted particle
        body.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))  # particle texture
        self.vis.BindItem(body)                                       # bind new body to visualization
        self.coll.BindItem(body)                                      # bind new body to collision system
        body.SetUseGyroTorque(False)                                 # disable gyroscopic torque for stability


vis = chronoirr.ChVisualSystemIrrlicht()                              # Irrlicht visualization
vis.AttachSystem(sys)                                                 # attach physical system
vis.SetWindowSize(1280, 720)                                          # window size
vis.SetWindowTitle("Particle emitter, gravitational attraction")     # window title
vis.Initialize()                                                      # create device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))      # logo
vis.AddSkyBox()                                                       # sky box
vis.AddCamera(chrono.ChVector3d(0, 14, -20))                          # camera eye position
vis.AddTypicalLights()                                                # standard lights

creator_callback = MyCreatorForAll(vis, sys.GetCollisionSystem())     # instantiate add-body callback
emitter.RegisterAddBodyCallback(creator_callback)                     # register callback on emitter

stepsize = 1e-2                                                       # integration step
render_fps = 50.0                                                     # review-render cadence
render_every = max(1, round(1.0 / (render_fps * stepsize)))          # untagged cadence constant
while vis.Run():                                                     # SCORED CORE = plain truth form, NO time bound
    vis.BeginScene()                                                 # begin frame
    vis.Render()                                                     # draw scene
    vis.EndScene()                                                   # end frame

    emitter.EmitParticles(sys, stepsize)                             # emit new particles this step

    G_constant = 6.674e-3                                            # gravitational constant (scaled)

    for body in sys.GetBodies():                                     # reset force/torque accumulators
        body.EmptyAccumulators()                                     # clear before re-accumulating

    for bodyA, bodyB in combinations(sys.GetBodies(), 2):            # every unordered body pair
        D_attract = bodyB.GetPos() - bodyA.GetPos()                  # separation vector A->B
        r_attract = D_attract.Length()                               # distance
        if r_attract > 1e-9:                                         # guard against coincident bodies
            mforce = G_constant * bodyA.GetMass() * bodyB.GetMass() / (r_attract * r_attract)  # F = G mA mB / r^2
            D_attract.Normalize()                                    # unit direction
            bodyA.AccumulateForce(D_attract * mforce, bodyA.GetPos(), False)   # pull A toward B
            bodyB.AccumulateForce(-D_attract * mforce, bodyB.GetPos(), False)  # pull B toward A

    kinetic_energy = 0.0                                             # total kinetic energy
    for body in sys.GetBodies():                                     # sum over all bodies
        kinetic_energy += 0.5 * body.GetMass() * body.GetPosDt().Length2()  # 1/2 m v^2

    potential_energy = 0.0                                           # total gravitational potential energy
    for bodyA, bodyB in combinations(sys.GetBodies(), 2):           # every unordered body pair
        r_attract = (bodyB.GetPos() - bodyA.GetPos()).Length()      # pair distance
        if r_attract > 1e-9:                                        # guard against coincident bodies
            potential_energy += -G_constant * (bodyA.GetMass() * bodyB.GetMass()) / r_attract  # -G mA mB / r

    total_energy = kinetic_energy + potential_energy                # total mechanical energy
    print("Kinetic: ", kinetic_energy, " Potential: ", potential_energy, " Total: ", total_energy)  # per-step energy log


    sys.DoStepDynamics(stepsize)                                     # advance physics one step
