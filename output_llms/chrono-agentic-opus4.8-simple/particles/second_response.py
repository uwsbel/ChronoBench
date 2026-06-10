import os
import itertools                                                       # pairwise body combinations for potential energy
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemNSC()                                            # rigid-body system for the orbiting particles
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))          # no uniform gravity; bodies attract pairwise
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)      # collision system for the particles scene

emitter = chrono.ChParticleEmitter()                                  # particle factory
emitter.SetParticlesPerSecond(2000)                                   # high rate so the cloud fills quickly
emitter.SetUseParticleReservoir(True)                                 # cap the total number of particles
emitter.SetParticleReservoirAmount(200)                               # number of particles to emit in total

mcreator_spheres = chrono.ChRandomShapeCreatorSpheres()               # emit spherical particles
mcreator_spheres.SetDiameterDistribution(chrono.ChZhangDistribution(0.6, 0.23))   # Zhang(avg=0.6, min=0.23)
mcreator_spheres.SetDensityDistribution(chrono.ChConstantDistribution(1600))      # constant density 1600 kg/m^3
mcreator_spheres.SetAddVisualizationAsset(True)                       # give each sphere a visual asset
emitter.SetParticleCreator(mcreator_spheres)                          # use the sphere creator

emitter_positions = chrono.ChRandomParticlePositionOnGeometry()       # spawn particles spread over a region
emitter_positions.SetGeometry(chrono.ChBox(12, 12, 12), chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))  # 12 m cube at origin
emitter.SetParticlePositioner(emitter_positions)                      # use the box positioner

emitter_velocities = chrono.ChRandomParticleVelocityAnyDirection()    # random isotropic launch direction
emitter_velocities.SetModulusDistribution(chrono.ChConstantDistribution(0.0))     # start at rest; attraction does the rest
emitter.SetParticleVelocity(emitter_velocities)                       # use the velocity distribution

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)                     # Z-up
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Particle galaxy with gravitational attraction")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, -28, 10), chrono.ChVector3d(0, 0, 0))  # frame the whole cloud
vis.AddTypicalLights()

time_step = 0.01                                                      # integration step [s]
sim_end = 20.0                                                        # simulation end time [s]
softening = 1.0                                                       # Plummer softening length [m] (tames close encounters)
render_fps = 50.0                                                     # frames per second for the review video
render_every = max(1, round(1.0 / (render_fps * time_step)))         # untagged cadence constant
bound = False                                                        # bind emitted particles to the renderer once

while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        emitter.EmitParticles(sys, time_step)                        # keep emitting until the reservoir is empty

        G_constant = 6.674e-2                                         # attraction constant (scaled for a visible orbit)
        bodies = sys.GetBodies()                                      # every particle currently in the system
        for bi in bodies:                                            # clear last step's accumulated attraction
            bi.EmptyAccumulators()
        for body_A, body_B in itertools.combinations(bodies, 2):     # every unordered pair of bodies
            D_attract = body_B.GetPos() - body_A.GetPos()            # vector from A to B
            r_attract = D_attract.Length()                          # distance between the pair
            soft = (r_attract * r_attract + softening * softening) ** 0.5   # Plummer-softened separation
            F_mag = G_constant * (body_A.GetMass() * body_B.GetMass()) / (soft * soft)  # |F| = G mA mB / (r^2+e^2)
            F_dir = D_attract * (F_mag / max(r_attract, 1e-6))       # force on A toward B
            body_A.AccumulateForce(F_dir, body_A.GetPos(), False)    # pull A toward B
            body_B.AccumulateForce(-F_dir, body_B.GetPos(), False)   # pull B toward A

        sys.DoStepDynamics(time_step)                               # advance one step
        if not bound and len(bodies) > 0:                          # render the particles once they exist
            vis.BindAll()
            bound = True

        kinetic_energy = 0.0                                        # total kinetic energy this step
        for bi in bodies:                                          # sum 0.5 m v^2 over all bodies
            kinetic_energy += 0.5 * bi.GetMass() * bi.GetPosDt().Length2()
        potential_energy = 0.0                                      # total gravitational potential energy this step
        for body_A, body_B in itertools.combinations(bodies, 2):   # sum -G mA mB / r over all pairs
            r_attract = (body_B.GetPos() - body_A.GetPos()).Length()
            soft = (r_attract * r_attract + softening * softening) ** 0.5   # matching softened separation
            potential_energy += -G_constant * (body_A.GetMass() * body_B.GetMass()) / soft
        total_energy = kinetic_energy + potential_energy            # total mechanical energy
        print("Kinetic: ", kinetic_energy, " Potential: ", potential_energy, " Total: ", total_energy)

        if sys.GetChTime() >= sim_end:
            break
