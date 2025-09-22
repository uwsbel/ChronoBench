import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.cascade as cascade
import math as m
import random as rand
from OCC.Core import BRepPrimAPI
from OCC.Core import BRepAlgoAPI
from OCC.Core import TopoDS







change_particles = False



def main() : 
    

    
    
    

    sys = chrono.ChSystemNSC()
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0,0,-9.81))
    sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    sys.GetSolver().AsIterative().SetMaxIterations(150)
    sys.SetMaxPenetrationRecoverySpeed(4.0)

    
    
    

    cube = cascade.CastToCascadeShape(TopoDS.TopoDS_Shape())
    cascade.Primitives_CreateBox(cube, 10, 10, 10)
    cube.SetMass(0)
    cube.SetFixed(False)
    cube.SetPos(chrono.ChVector3d(0,0,5))
    cube.SetRot(chrono.ChQuaterniond(1,0,0,0))

    mcoll = chrono.ChCollisionModelNSC()
    mcoll.SetShapeType(chrono.ChCollisionShape.Type_BULLET)
    mcoll.SetMargin(0.0025)
    cube_collision = chrono.ChCollisionShapeBullet(cube, mcoll)

    body = chrono.ChBody()
    body.SetName("large_cube")
    body.AddCollisionShape(cube_collision)
    body.AddVisualShape(cascade.ChVisualShapeCascadeShape(cube))
    sys.Add(body)

    
    
    
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(0, 0, 1.5), chrono.ChQuaterniond(1, 0, 0, 0))
    emitter = cascade.ChCascadeParticleEmitterRelative(offset_pose, 0.05)

    
    shape_prob = [0.5, 0.3, 0.2]
    shape_list = [cascade.ChCascadeShapeData.BULLET_SPHERE, cascade.ChCascadeShapeData.BULLET_BOX, 
                  cascade.ChCascadeShapeData.CASCADE_PRIMITIVES_BOX]

    
    
    
    max_particles_per_second = 1000

    
    system = cascade.ChCascadeParticleSystem(sys)
    system.SetGravitationalForceType(cascade.ChCascadeParticleSystem.GRAVITY_TYPE_SIMULATED)
    system.SetEmissiveColor(chrono.ChColor(0, 0, 0))
    system.SetParticleRadius(0.01)
    system.SetLifespan(0.5)
    system.SetDragType(cascade.ChCascadeParticleSystem.DRAG_TYPE_NONE)
    system.Initialize()

    
    
    
    vis = cascade.ChCascadeVisualSystemIrrlicht()
    vis.SetWindowTitle('Particle Emitter')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachSystem(sys)

    
    
    
    def change_particle_properties(particle, time):
        
        r = m.fabs(m.sin(particle.GetPos().y) * 0.5 + 0.5)
        g = m.fabs(m.cos(particle.GetPos().x) * 0.5 + 0.5)
        b = 0.3
        particle.SetEmissiveColor(chrono.ChColor(r,g,b))
        particle.SetPos(emitter.GetOffsetPose() * chrono.ChVector3d(0.1, 0.1, 0.1))

    
    
    
    render_time = 0

    sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    sys.GetSolver().AsIterative().SetMaxIterations(150)

    while vis.Run() :
        time = sys.GetChTime()

        
        if not vis.Run():
            break
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        
        offset_pose = emitter.GetOffsetPose()

        
        num_particles = int(max_particles_per_second / 50)  
        particles = []
        for i in range(num_particles):
            shape_id = rand.choices(range(len(shape_list)), weights=shape_prob)[0]
            shape = cascade.ChCascadeShapeData(shape_list[shape_id]).createShape()
            cascade.CastToCascadeShape(shape).SetData(cascade.ChCascadeShapeData.BULLET_PRIMITIVES_BOX, 0.02, 0.02, 0.02)
            particle = cascade.ChCascadeParticle(shape, offset_pose)
            particle.SetLifespan(1.0 / max_particles_per_second)
            particle.SetPos(chrono.ChVector3d(0.1, 0.1, 0.1))
            particle.SetDragType(cascade.ChCascadeParticle.DRAG_TYPE_NONE)
            particle.SetEmissiveColor(chrono.ChColor(0.5, 0.5, 0.3))
            system.AddParticle(particle)
            particles.append(particle)

        
        system.Update()

        
        sys.DoStepDynamics(0.05)

        
        sys.CheckForNullBodyPointers()

        
        vis.Synchronize(time)
        emitter.Synchronize(time)

        if change_particles:
            for particle in particles:
                change_particle_properties(particle, time)

    return 0







main()