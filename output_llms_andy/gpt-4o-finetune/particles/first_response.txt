import pychrono as chrono
import pychrono.irrlicht as chronoirr
from pychrono import fcl
import numpy as np
import random as rand

rest_density = 1000.0
kernel_radius = 0.03
batch_size = 200
bce_thickness = kernel_radius
bce_step = kernel_radius / 2

class MyCustomForce(chrono.ChForceCustom):
    def __init__(self, chassis, t_offset, fixed=False):
        super().__init__(chassis, fixed)
        self.ref_body = chassis.GetSystem().GetBody(0)
        self.t_offset = t_offset
        self.dt = 0
        self.SetValidDomain(chrono.ValidDomain_Systems)  # enable this force only in the system domain

    def Update(self, time, dt):
        self.dt = dt
        np.random.seed(100)
        # ------ SPH parameters --------
        m = 0.4 * 0.4 * 0.4 * rest_density  # particle mass
        # ------------------------------
        # Current system
        sys = self.GetBody().GetSystem()

        # get the number of particles in the custom force body (the particle container)
        n = self.GetBody().GetPositives().size()

        for i in range(n):
            pi = self.GetBody().GetPositives().at(i)
            xi = pi.GetPos()  # position of particle i
            fi = chrono.ChVector3d(0, 0, 0)  # initialize the current force for this particle

            for j in range(n):
                if j != i:
                    pj = self.GetBody().GetPositives().at(j)
                    xj = pj.GetPos()  # position of particle j
                    # vector from particle j to i
                    r_ = xi - xj
                    r = r_.Length()
                    if r < kernel_radius:
                        # direction of the force
                        r_ = r_ / r
                        # particle mass (assuming equality here for simplicity)
                        m_j = m
                        # neighbors density (assuming equality here for simplicity)
                        rho_j = rest_density
                        mj_by_rho_j = m_j / rho_j
                        # particle j pressure (equal and opposite to i)
                        p_j = -1 * pj.GetPressure()
                        # particle j density gradient
                        grad_rho_j = -1 * pj.GetDensityGradient()
                        # pressure and viscosity force term: -m_j * (p_j / rho_j^2 + p_i / rho_i^2) * grad_W_ij
                        # (symmetric form, using here p_j and grad_rho_j)
                        f_pi = m * (p_j / (rho_j * rho_j)) * grad_rho_j
                        # non-symmetric form: (p_i - p_j) / rho_i^2 * grad_rho_j
                        f_pi = f_pi + m_j_by_rho_j * (p_j) * grad_rho_j

                        c_pi = 0.01
                        # viscosity term: c_pi * m_j * (vj - vi) / rho_j / r * grad_W_ij
                        if p_j > 0:
                            f_pi = f_pi + (-1) * c_pi * m_j_by_rho_j * (pj.GetDiffPos() - pi.GetDiffPos()) / r * grad_rho_j
                        f_pi = -1 * f_pi
                        f_pi = f_pi if r > 1e-15 else chrono.ChVector3d(0, 0, 0)
                        fi = fi + f_pi

            # gravitational force, adjusted by a time-offset vector to simulate movement
            g = chrono.ChVector3d(0, -9.81, 0)  # gravity vector
            d = self.dt * self.dt * g + self.t_offset * (self.dt * self.dt * self.dt * self.dt)
            fi = fi + m * g * (1 - self.t_offset) - m * d / (self.dt * self.dt)

            pi.SetForce(pi.GetForce() + fi)

def main():
    # create the Chrono system
    sys = chrono.ChSystemSMC()
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))  # disable global gravity
    sys.GetSettings().collision.collision_envelope = 0.001
    sys.GetSettings().collision.narrowphase_algorithm = chrono.ChNarrowphaseAlgorithmAlgorithm_DYN_BINS
    # create the Irrlicht visualization
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('FEa demonstration')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0, 1, 0.3))
    vis.AddTypicalLights()

    # create the ramp body
    ramp = chrono.ChBodyEasyBox(1, 0.2, 1, 1000, True, True)
    ramp.SetPos(chrono.ChVector3d(0, -0.2, 0))
    ramp.SetFixed(True)  # fix the ramp to the world
    ramp.SetCollide(False)
    ramp.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile('textures/concrete.jpg'))
    sys.Add(ramp)

    # create the particle container body
    container = chrono.ChBody()
    container.SetPos(chrono.ChVector3d(0, 0, 0))
    sys.Add(container)

    # Create the particle emitter
    emittor = chrono.ChParticleEmitter()
    emittor.SetParticlesPerSecond(200)  # set particle emission rate
    emittor.SetUseParticleReservoir(True)  # use particle reservoir
    emittor.SetParticlesInReservoir(200)  # set number of particles in reservoir
    emittor.SetFinalTime(20)  # set emitter's final time
    emittor.SetMakeStatic(True)  # make particles static
    emittor.SetParticleLifetime(2)  # set lifetime of particles

    # Define possible collision shapes (sphere or box) for emitted particles:
    # create and configure sph collision model
    sph_model = chrono.ChCollisionModel()
    sph_model.AddSphere(0, chrono.ChVector3d(0, 0, 0), 0.01)  # add sphere to collision model
    sph_model.BuildModel()  # build the collision model
    # create collision shape from sph_model
    sph_shape = chrono.ChParticlesClonesSphericCollisionShape()
    sph_shape.SetModel(sph_model)
    # create corresponding visual shape for the sphere
    sph_visual = chrono.ChVisualShapeSphere(0.01)
    # create pile of shapes to hold collision and visual shape
    sph_pile_shapes = chrono.ChPileBodyShape()
    sph_pile_shapes.AddShape(sph_shape)
    sph_pile_shapes.AddShape(sph_visual)

    # create and configure box collision model 
    box_model = chrono.ChCollisionModel()
    box_model.AddBox(0, chrono.ChVector3d(0, 0, 0), 0.01, 0.01, 0.01)
    box_model.BuildModel()
    # create collision shape from box_model
    box_shape = chrono.ChParticlesClonesSphericCollisionShape()
    box_shape.SetModel(box_model)
    # create corresponding visual shape for the box
    box_visual = chrono.ChVisualShapeTriangleMesh()
    box_visual.SetFilename(chrono.GetChronoDataFile('mesh/cube.obj'))
    box_visual.SetMutable(False)
    # create pile of shapes to hold collision and visual shape
    box_pile_shapes = chrono.ChPileBodyShape()
    box_pile_shapes.AddShape(box_shape)
    box_pile_shapes.AddShape(box_visual)

    # Add the pile shapes to the emitter when emitted:
    ch_pile_shapes = chrono.vector_ChSharedBodyShapePtr()
    ch_pile_shapes.append(sph_pile_shapes)
    ch_pile_shapes.append(box_pile_shapes)
    emittor.AddEmittedPileShapes(ch_pile_shapes)

    # create and configure emitted particle body
    emitted_body = chrono.ChBody()
    # Add the emitted body to the emitter:
    emittor.SetOtherBody(emitted_body)

    # Set emitted particles' initial conditions:

    # create position distribution for particles
    pos_distrib = chrono.ChRandomParticlePosition()
    pos_distribinsideboxA = chrono.ChRandomPositionInsideBox()
    pos_distribinsideboxA.SetBoxLengths(chrono.ChVector3d(0.3, 0.3, 0.3))  # set box lengths
    pos_distrib.SetDistribution(pos_distribinsideboxA)  # set position distribution
    emittor.SetParticlePositionDistribution(pos_distrib)  # set position distribution for emitter

    # create velocity distribution for particles
    vel_distrib = chrono.ChRandomParticleVelocity()
    constant_vel = chrono.ChConstantRandomDistribution(1.0)
    vel_distribX = chrono.ChRandomVelocityOnXAxis()
    vel_distribX.SetModulusDistribution(constant_vel)  # set modulus distribution
    vel_distrib.SetDistribution(vel_distribX)  # set velocity distribution
    emittor.SetParticleVelocityDistribution(vel_distrib)  # set velocity distribution for emitter

    # create orientation distribution for particles
    orientation_distrib = chrono.ChRandomParticleAlignment()
    uniform_orientation = chrono.ChUniformRandomDistribution(-1, 1)
    orientation_distribquaternion = chrono.ChRandomQuaternion()
    orientation_distribquaternion.SetWDistribution(uniform_orientation)  # set W distribution
    orientation_distribquaternion.SetXDistribution(uniform_orientation)  # set X distribution
    orientation_distribquaternion.SetYDistribution(uniform_orientation)  # set Y distribution
    orientation_distribquaternion.SetZDistribution(uniform_orientation)  # set Z distribution
    orientation_distrib.SetDistribution(orientation_distribquaternion)  # set orientation distribution
    emittor.SetParticleOrientationDistribution(orientation_distrib)  # set orientation distribution for emitter

    # create angular velocity distribution for particles
    ang_velocity_distribution = chrono.ChRandomParticleAngularVelocity()
    null_angular_velocity = chrono.ChNullRandomDistribution()
    ang_velocity_distributionquaternion = chrono.ChRandomQuaternion()
    ang_velocity_distributionquaternion.SetXDistribution(null_angular_velocity)  # set X distribution
    ang_velocity_distributionquaternion.SetYDistribution(null_angular_velocity)  # set Y distribution
    ang_velocity_distributionquaternion.SetZDistribution(null_angular_velocity)  # set Z distribution
    ang_velocity_distribution.SetDistribution(ang_velocity_distributionquaternion)  # set angular velocity distribution
    emittor.SetParticleAngularVelocityDistribution(ang_velocity_distribution)  # set angular velocity distribution for emitter

    # Create SPH force container for emitted_body
    mforces = chrono.ChForceContainer()
    emitted_body.AddForce(mforces)  # add forces to emitted body

    # create and configure SPH FSI force
    forceFSI = chrono.ChForceSPH()
    forceFSI.SetKernelRadius(kernel_radius)  # set kernel radius
    forceFSI.SetRestDensity(rest_density)  # set rest density
    mforces.Add(forceFSI)  # add SPH FSI force to forces container

    # create for custom force and add to container
    forceCustom = MyCustomForce(emitted_body, 0.3, False)
    mforces.Add(forceCustom)

    emitted_body.SetVisualizeRefFrames(True)  # visualize reference frames of emitted body

    # simulation loop
    time = 0
    time_step = 1e-3
    while vis.Run():
        time = sys.GetChTime()  # update simulation time

        if time < 1:
            forceCustom.t_offset = time  # set offset time for custom force
        else:
            forceCustom.t_offset = 1  # set max offset time for custom force

        # Draw particles as colored point clouds
        n = emitted_body.GetPositives().size()
        particles_pos = chrono.ChVector3d() * n
        particles_vel = chrono.ChVector3d() * n
        c = chrono.ChColor(0, 0, 1)  # blue color for particles

        # Add particles to visualizer
        for i in range(n):
            particles_pos[i] = emitted_body.GetPositives().at(i).GetPos()
            particles_vel[i] = emitted_body.GetPositives().at(i).GetDiffPos()
        vis.GetData().AddPointCloud(n, particles_pos, c)

        # emit particles
        emittor.EmitParticles(time, time_step)

        sys.DoStepDynamics(time_step)  # advance simulation by one time step
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

if __name__ == "__main__":
    main()