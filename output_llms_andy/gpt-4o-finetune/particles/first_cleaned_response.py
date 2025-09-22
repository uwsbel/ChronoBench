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
        self.SetValidDomain(chrono.ValidDomain_Systems)  

    def Update(self, time, dt):
        self.dt = dt
        np.random.seed(100)
        
        m = 0.4 * 0.4 * 0.4 * rest_density  
        
        
        sys = self.GetBody().GetSystem()

        
        n = self.GetBody().GetPositives().size()

        for i in range(n):
            pi = self.GetBody().GetPositives().at(i)
            xi = pi.GetPos()  
            fi = chrono.ChVector3d(0, 0, 0)  

            for j in range(n):
                if j != i:
                    pj = self.GetBody().GetPositives().at(j)
                    xj = pj.GetPos()  
                    
                    r_ = xi - xj
                    r = r_.Length()
                    if r < kernel_radius:
                        
                        r_ = r_ / r
                        
                        m_j = m
                        
                        rho_j = rest_density
                        mj_by_rho_j = m_j / rho_j
                        
                        p_j = -1 * pj.GetPressure()
                        
                        grad_rho_j = -1 * pj.GetDensityGradient()
                        
                        
                        f_pi = m * (p_j / (rho_j * rho_j)) * grad_rho_j
                        
                        f_pi = f_pi + m_j_by_rho_j * (p_j) * grad_rho_j

                        c_pi = 0.01
                        
                        if p_j > 0:
                            f_pi = f_pi + (-1) * c_pi * m_j_by_rho_j * (pj.GetDiffPos() - pi.GetDiffPos()) / r * grad_rho_j
                        f_pi = -1 * f_pi
                        f_pi = f_pi if r > 1e-15 else chrono.ChVector3d(0, 0, 0)
                        fi = fi + f_pi

            
            g = chrono.ChVector3d(0, -9.81, 0)  
            d = self.dt * self.dt * g + self.t_offset * (self.dt * self.dt * self.dt * self.dt)
            fi = fi + m * g * (1 - self.t_offset) - m * d / (self.dt * self.dt)

            pi.SetForce(pi.GetForce() + fi)

def main():
    
    sys = chrono.ChSystemSMC()
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))  
    sys.GetSettings().collision.collision_envelope = 0.001
    sys.GetSettings().collision.narrowphase_algorithm = chrono.ChNarrowphaseAlgorithmAlgorithm_DYN_BINS
    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('FEa demonstration')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0, 1, 0.3))
    vis.AddTypicalLights()

    
    ramp = chrono.ChBodyEasyBox(1, 0.2, 1, 1000, True, True)
    ramp.SetPos(chrono.ChVector3d(0, -0.2, 0))
    ramp.SetFixed(True)  
    ramp.SetCollide(False)
    ramp.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile('textures/concrete.jpg'))
    sys.Add(ramp)

    
    container = chrono.ChBody()
    container.SetPos(chrono.ChVector3d(0, 0, 0))
    sys.Add(container)

    
    emittor = chrono.ChParticleEmitter()
    emittor.SetParticlesPerSecond(200)  
    emittor.SetUseParticleReservoir(True)  
    emittor.SetParticlesInReservoir(200)  
    emittor.SetFinalTime(20)  
    emittor.SetMakeStatic(True)  
    emittor.SetParticleLifetime(2)  

    
    
    sph_model = chrono.ChCollisionModel()
    sph_model.AddSphere(0, chrono.ChVector3d(0, 0, 0), 0.01)  
    sph_model.BuildModel()  
    
    sph_shape = chrono.ChParticlesClonesSphericCollisionShape()
    sph_shape.SetModel(sph_model)
    
    sph_visual = chrono.ChVisualShapeSphere(0.01)
    
    sph_pile_shapes = chrono.ChPileBodyShape()
    sph_pile_shapes.AddShape(sph_shape)
    sph_pile_shapes.AddShape(sph_visual)

    
    box_model = chrono.ChCollisionModel()
    box_model.AddBox(0, chrono.ChVector3d(0, 0, 0), 0.01, 0.01, 0.01)
    box_model.BuildModel()
    
    box_shape = chrono.ChParticlesClonesSphericCollisionShape()
    box_shape.SetModel(box_model)
    
    box_visual = chrono.ChVisualShapeTriangleMesh()
    box_visual.SetFilename(chrono.GetChronoDataFile('mesh/cube.obj'))
    box_visual.SetMutable(False)
    
    box_pile_shapes = chrono.ChPileBodyShape()
    box_pile_shapes.AddShape(box_shape)
    box_pile_shapes.AddShape(box_visual)

    
    ch_pile_shapes = chrono.vector_ChSharedBodyShapePtr()
    ch_pile_shapes.append(sph_pile_shapes)
    ch_pile_shapes.append(box_pile_shapes)
    emittor.AddEmittedPileShapes(ch_pile_shapes)

    
    emitted_body = chrono.ChBody()
    
    emittor.SetOtherBody(emitted_body)

    

    
    pos_distrib = chrono.ChRandomParticlePosition()
    pos_distribinsideboxA = chrono.ChRandomPositionInsideBox()
    pos_distribinsideboxA.SetBoxLengths(chrono.ChVector3d(0.3, 0.3, 0.3))  
    pos_distrib.SetDistribution(pos_distribinsideboxA)  
    emittor.SetParticlePositionDistribution(pos_distrib)  

    
    vel_distrib = chrono.ChRandomParticleVelocity()
    constant_vel = chrono.ChConstantRandomDistribution(1.0)
    vel_distribX = chrono.ChRandomVelocityOnXAxis()
    vel_distribX.SetModulusDistribution(constant_vel)  
    vel_distrib.SetDistribution(vel_distribX)  
    emittor.SetParticleVelocityDistribution(vel_distrib)  

    
    orientation_distrib = chrono.ChRandomParticleAlignment()
    uniform_orientation = chrono.ChUniformRandomDistribution(-1, 1)
    orientation_distribquaternion = chrono.ChRandomQuaternion()
    orientation_distribquaternion.SetWDistribution(uniform_orientation)  
    orientation_distribquaternion.SetXDistribution(uniform_orientation)  
    orientation_distribquaternion.SetYDistribution(uniform_orientation)  
    orientation_distribquaternion.SetZDistribution(uniform_orientation)  
    orientation_distrib.SetDistribution(orientation_distribquaternion)  
    emittor.SetParticleOrientationDistribution(orientation_distrib)  

    
    ang_velocity_distribution = chrono.ChRandomParticleAngularVelocity()
    null_angular_velocity = chrono.ChNullRandomDistribution()
    ang_velocity_distributionquaternion = chrono.ChRandomQuaternion()
    ang_velocity_distributionquaternion.SetXDistribution(null_angular_velocity)  
    ang_velocity_distributionquaternion.SetYDistribution(null_angular_velocity)  
    ang_velocity_distributionquaternion.SetZDistribution(null_angular_velocity)  
    ang_velocity_distribution.SetDistribution(ang_velocity_distributionquaternion)  
    emittor.SetParticleAngularVelocityDistribution(ang_velocity_distribution)  

    
    mforces = chrono.ChForceContainer()
    emitted_body.AddForce(mforces)  

    
    forceFSI = chrono.ChForceSPH()
    forceFSI.SetKernelRadius(kernel_radius)  
    forceFSI.SetRestDensity(rest_density)  
    mforces.Add(forceFSI)  

    
    forceCustom = MyCustomForce(emitted_body, 0.3, False)
    mforces.Add(forceCustom)

    emitted_body.SetVisualizeRefFrames(True)  

    
    time = 0
    time_step = 1e-3
    while vis.Run():
        time = sys.GetChTime()  

        if time < 1:
            forceCustom.t_offset = time  
        else:
            forceCustom.t_offset = 1  

        
        n = emitted_body.GetPositives().size()
        particles_pos = chrono.ChVector3d() * n
        particles_vel = chrono.ChVector3d() * n
        c = chrono.ChColor(0, 0, 1)  

        
        for i in range(n):
            particles_pos[i] = emitted_body.GetPositives().at(i).GetPos()
            particles_vel[i] = emitted_body.GetPositives().at(i).GetDiffPos()
        vis.GetData().AddPointCloud(n, particles_pos, c)

        
        emittor.EmitParticles(time, time_step)

        sys.DoStepDynamics(time_step)  
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

if __name__ == "__main__":
    main()