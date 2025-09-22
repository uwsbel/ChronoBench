import pychrono as chrono
import pychrono.irrlicht as chronoirr
from itertools import combinations





class MyCreatorForAll(chrono.ChRandomShapeCreator_AddBodyCallback):
    def __init__(self, vis):
        super().__init__()
        self.vis = vis

    def OnAddBody(self, body, coords, creator):
        body.GetVisualShape(0).SetTexture(
            chrono.GetChronoDataFile("textures/bluewhite.png"))
        self.vis.BindItem(body)
        body.SetUseGyroTorque(False)





system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


system.Set_G_acc(chrono.ChVector3d(0, 0, 0))




common_mat = chrono.ChContactMaterialNSC()
common_mat.SetFriction(0.2)


def make_big_sphere(radius, density, pos, vel, texture):
    body = chrono.ChBodyEasySphere(radius, density, True, True, common_mat)
    body.SetPos(chrono.ChVector3d(*pos))
    body.SetPosDt(chrono.ChVector3d(*vel))
    body.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile(texture))
    body.SetUseGyroTorque(False)
    system.AddBody(body)
    return body





sphere1 = make_big_sphere(
    radius=2.1, density=1800,
    pos=(1, 1, 0), vel=(0.5, 0, 0.1),
    texture="textures/concrete.jpg")

sphere2 = make_big_sphere(
    radius=2.1, density=1800,
    pos=(-10, -10, 0), vel=(-0.5, 0, -0.1),
    texture="textures/concrete.jpg")

sphere3 = make_big_sphere(
    radius=2.1, density=1800,
    pos=(0, 20, 0), vel=(0, -0.5, 0.2),
    texture="textures/concrete.jpg")

big_spheres = [sphere1, sphere2, sphere3]




emitter = chrono.ChParticleEmitter()
emitter.SetParticlesPerSecond(2000)
emitter.SetUseParticleReservoir(True)
emitter.SetParticleReservoirAmount(200)


pos_gen = chrono.ChRandomParticlePositionOnGeometry()
box = chrono.ChBox(chrono.ChVector3d(0, 0, 0), chrono.ChRotation3d(),
                   chrono.ChVector3d(50, 50, 50))
pos_gen.SetGeometry(box)
emitter.SetParticlePositioner(pos_gen)


emitter.SetParticleAligner(chrono.ChRandomParticleAlignmentUniform())

lin_vel_gen = chrono.ChRandomParticleVelocityAnyDirection()
lin_vel_gen.SetModulusDistribution(chrono.ChUniformDistribution(0.0, 0.5))
emitter.SetParticleVelocity(lin_vel_gen)

ang_vel_gen = chrono.ChRandomParticleVelocityAnyDirection()
ang_vel_gen.SetModulusDistribution(chrono.ChUniformDistribution(0.0, 0.2))
emitter.SetParticleAngularVelocity(ang_vel_gen)


shape_creator = chrono.ChRandomShapeCreatorSpheres()
shape_creator.SetDiameterDistribution(chrono.ChZhangDistribution(0.6, 0.23))
shape_creator.SetDensityDistribution(chrono.ChConstantDistribution(1600))
emitter.SetParticleCreator(shape_creator)




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Three-body simulation with particle emitter")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 14, -30))
vis.AddTypicalLights()


emitter.RegisterAddBodyCallback(MyCreatorForAll(vis))




system.SetSolverType(chrono.ChSolver.Type_PSOR)
system.GetSolver().AsIterative().SetMaxIterations(40)




dt = 1e-2
G = 6.674e-3     




while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    emitter.EmitParticles(system, dt)

    
    for bod in system.Get_bodylist():
        bod.Empty_forces_accumulators()

    
    
    
    kinetic_E = sum(0.5 * b.GetMass() * b.GetPosDt().Length2()
                    for b in big_spheres)

    potential_E = 0.0
    for a, b in combinations(big_spheres, 2):
        delta = b.GetPos() - a.GetPos()
        r = max(delta.Length(), 1e-4)       
        potential_E += -G * a.GetMass() * b.GetMass() / r

        
        F_mag = G * a.GetMass() * b.GetMass() / (r * r)
        F_vec = delta * (F_mag / r)         
        a.Accumulate_force( F_vec, a.GetPos(), False)
        b.Accumulate_force(-F_vec, b.GetPos(), False)

    total_E = kinetic_E + potential_E
    print(f"Kinetic: {kinetic_E: .6f} | Potential: {potential_E: .6f} | Total: {total_E: .6f}")

    
    system.DoStepDynamics(dt)