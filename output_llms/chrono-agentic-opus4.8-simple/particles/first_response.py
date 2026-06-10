import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemNSC()                                            # NSC rigid-body system
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)      # collision system required (particles have shapes)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))          # no global gravity; attraction is applied by hand

emitter = chrono.ChParticleEmitter()                                  # the particle emitter
emitter.SetParticlesPerSecond(2000)                                   # emission rate [particles/s]
emitter.SetUseParticleReservoir(True)                                 # cap the total number emitted
emitter.SetParticleReservoirAmount(200)                               # at most 200 particles created

pos_emitter = chrono.ChRandomParticlePositionRectangleOutlet()        # particles spawn over a rectangular outlet
pos_emitter.OutletWidth = 9.0                                         # outlet width  [m] (wide cloud so particles fall inward)
pos_emitter.OutletHeight = 9.0                                        # outlet height [m]
pos_emitter.Outlet().pos = chrono.ChVector3d(0, 0, 0)                 # outlet centered at the origin
pos_emitter.Outlet().rot = chrono.QuatFromAngleX(chrono.CH_PI_2)      # face the outlet plane upward
emitter.SetParticlePositioner(pos_emitter)                            # register the positioner

vel_emitter = chrono.ChRandomParticleVelocityAnyDirection()           # random direction velocities
vel_emitter.SetModulusDistribution(chrono.ChUniformDistribution(0.0, 0.05))  # small random speed [m/s] so attraction dominates
emitter.SetParticleVelocity(vel_emitter)                              # register the velocity model

align_emitter = chrono.ChRandomParticleAlignmentUniform()            # random orientation for each particle
emitter.SetParticleAligner(align_emitter)                            # register the aligner

ang_vel = chrono.ChRandomParticleVelocityAnyDirection()             # random angular velocity
ang_vel.SetModulusDistribution(chrono.ChUniformDistribution(0.0, 0.4))  # random spin magnitude [rad/s]
emitter.SetParticleAngularVelocity(ang_vel)                         # register the angular velocity model

creator_hulls = chrono.ChRandomShapeCreatorConvexHulls()           # random convex-hull (random shapes)
creator_hulls.SetChordDistribution(chrono.ChUniformDistribution(0.3, 0.6))  # random characteristic size [m]
creator_hulls.SetDensityDistribution(chrono.ChConstantDistribution(1000))   # density [kg/m^3]
creator_hulls.SetAddCollisionShape(True)                           # give each particle a collision shape
creator_hulls.SetAddVisualizationAsset(True)                       # give each particle a visual shape


class ColorCallback(chrono.ChRandomShapeCreator_AddBodyCallback):    # tint each new particle on creation
    def OnAddBody(self, body, coords, creator):
        col = chrono.ChColor(chrono.ChRandom.Get(), chrono.ChRandom.Get(), chrono.ChRandom.Get())  # random color
        body.GetVisualShape(0).SetColor(col)                        # apply the random color


color_cb = ColorCallback()                                          # instantiate the callback
creator_hulls.RegisterAddBodyCallback(color_cb)                     # fire it for every emitted body
emitter.SetParticleCreator(creator_hulls)                           # register the shape creator

vis = chronoirr.ChVisualSystemIrrlicht()                            # Irrlicht render window
vis.AttachSystem(sys)                                               # bind the physical system
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)                  # Z-up camera convention
vis.SetWindowSize(1280, 720)                                       # window resolution
vis.SetWindowTitle("Particle gravitational attraction")           # window title
vis.Initialize()                                                   # create the device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # PyChrono logo
vis.AddSkyBox()                                                    # sky box
vis.AddCamera(chrono.ChVector3d(0, -14, 6), chrono.ChVector3d(0, 0, 0))  # eye / target
vis.AddTypicalLights()                                            # standard two-light setup

G_const = 5.0e-1                                                  # boosted gravitational constant so attraction visibly dominates
time_step = 0.01                                                  # integration step [s]
sim_end = 20.0                                                    # simulation duration [s]
render_fps = 50.0                                                # target review frame rate
render_every = max(1, round(1.0 / (render_fps * time_step)))     # untagged cadence constant
bound = 0                                                         # how many emitted bodies are already bound to Irrlicht

while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()                                            # start frame
    vis.Render()                                               # draw the scene
    vis.EndScene()                                             # finish frame
    for _ in range(render_every):
        emitter.EmitParticles(sys, time_step)                 # spawn new particles this step
        bodies = sys.GetBodies()                              # current particle bodies
        while bound < len(bodies):                            # register visuals of newly emitted particles
            vis.BindItem(bodies[bound])                       # bind each new body to the renderer
            bound += 1
        for b in bodies:
            b.EmptyAccumulators()                             # clear last step's applied forces
        n = len(bodies)
        for i in range(n):                                    # pairwise Newtonian attraction
            bi = bodies[i]
            pi = bi.GetPos()
            mi = bi.GetMass()
            for j in range(i + 1, n):
                bj = bodies[j]
                d = bj.GetPos() - pi                          # vector from i to j
                dist2 = d.Length2() + 0.5                     # softened squared distance (avoid close-pair blowup)
                dist = dist2 ** 0.5
                fmag = G_const * mi * bj.GetMass() / dist2    # gravitational magnitude
                fvec = d * (fmag / dist)                      # force on i toward j
                bi.AccumulateForce(fvec, pi, False)           # pull i toward j (world force at COM)
                bj.AccumulateForce(-fvec, bj.GetPos(), False) # equal and opposite on j
        sys.DoStepDynamics(time_step)                         # advance one step
        if sys.GetChTime() >= sim_end:
            break
