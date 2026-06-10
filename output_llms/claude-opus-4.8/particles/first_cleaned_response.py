import pychrono as chrono
import pychrono.irrlicht as chronoirr
import random
import math








sys = chrono.ChSystemNSC()



sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))


sys.SetSolverType(chrono.ChSolver.Type_PSOR)
sys.GetSolver().AsIterative().SetMaxIterations(40)










emitter = chrono.ChParticleEmitter()


emitter.SetParticlesPerSecond(2000)





emitter_positions = chrono.ChRandomParticlePositionOnGeometry()
emitter_positions.SetGeometry(
    chrono.ChBox(0.50, 0.50, 0.50),     
    chrono.ChFramed()                   
)
emitter.SetParticlePositioner(emitter_positions)




emitter_rotations = chrono.ChRandomParticleAlignmentUniform()
emitter.SetParticleAligner(emitter_rotations)




mvelo = chrono.ChRandomParticleVelocityAnyDirection()
mvelo.SetModulusDistribution(chrono.ChUniformDistribution(0.0, 0.5))
emitter.SetParticleVelocity(mvelo)




mangvelo = chrono.ChRandomParticleVelocityAnyDirection()
mangvelo.SetModulusDistribution(chrono.ChUniformDistribution(0.0, 0.2))
emitter.SetParticleAngularVelocity(mangvelo)








creator_spheres = chrono.ChRandomShapeCreatorSpheres()
creator_spheres.SetDiameterDistribution(
    chrono.ChZhangDistribution(0.15, 0.06))   
creator_spheres.SetDensityDistribution(
    chrono.ChConstantDistribution(1000))


creator_boxes = chrono.ChRandomShapeCreatorBoxes()
creator_boxes.SetXsizeDistribution(
    chrono.ChZhangDistribution(0.20, 0.08))
creator_boxes.SetSizeRatioZDistribution(
    chrono.ChMinMaxDistribution(0.5, 1.0))
creator_boxes.SetSizeRatioYZDistribution(
    chrono.ChMinMaxDistribution(0.5, 1.0))
creator_boxes.SetDensityDistribution(
    chrono.ChConstantDistribution(1000))


creator_cyls = chrono.ChRandomShapeCreatorCylinders()
creator_cyls.SetDiameterDistribution(
    chrono.ChZhangDistribution(0.20, 0.08))
creator_cyls.SetLengthFactorDistribution(
    chrono.ChMinMaxDistribution(0.5, 1.5))
creator_cyls.SetDensityDistribution(
    chrono.ChConstantDistribution(1000))


combined_creator = chrono.ChRandomShapeCreatorFromFamilies()
combined_creator.AddFamily(creator_spheres, 0.4)   
combined_creator.AddFamily(creator_boxes,   0.3)   
combined_creator.AddFamily(creator_cyls,    0.3)   
combined_creator.Setup()


class ParticleColorCallback(chrono.AddBodyCallback):
    def __init__(self):
        chrono.AddBodyCallback.__init__(self)

    def OnAddBody(self, body, coords, creator):
        vis_color = chrono.ChColor(random.random(),
                                   random.random(),
                                   random.random())
        body.GetVisualShape(0).SetColor(vis_color)

color_callback = ParticleColorCallback()
combined_creator.RegisterAddBodyCallback(color_callback)


emitter.SetParticleCreator(combined_creator)










G_constant = 6.674e-3


def apply_attraction_forces(system):
    
    bodies = system.GetBodies()
    n = len(bodies)

    
    for b in bodies:
        b.EmptyAccumulators()

    
    for i in range(n):
        bi = bodies[i]
        pi = bi.GetPos()
        mi = bi.GetMass()
        for j in range(i + 1, n):
            bj = bodies[j]
            pj = bj.GetPos()
            mj = bj.GetMass()

            
            dir_vec = pj - pi
            dist2 = dir_vec.Length2()
            if dist2 < 1e-6:
                continue
            dist = math.sqrt(dist2)
            dir_unit = dir_vec * (1.0 / dist)

            
            force_mag = G_constant * mi * mj / dist2
            force_vec = dir_unit * force_mag

            
            bi.AccumulateForce(force_vec,  pi, False)
            bj.AccumulateForce(-force_vec, pj, False)








vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Particle Gravitational Attraction')
vis.Initialize()
vis.AddLogo(chronoirr.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(4, 4, 4),    
              chrono.ChVector3d(0, 0, 0))    
vis.AddTypicalLights()
vis.AddLight(chrono.ChVector3d(0, 5, 0), 10)








time_step = 0.01

while vis.Run():
    
    emitter.EmitParticles(sys, time_step)

    
    apply_attraction_forces(sys)

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    sys.DoStepDynamics(time_step)