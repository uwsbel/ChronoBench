import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.cascade as cascade
import pychrono.parsers as parsers
import os
import math as m

def main():
    

    
    system = cascade.CascadeSystemNSC()

    
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    
    
    
    

    
    table = cascade.CCascadeBodyEasyBox(system, chrono.ChVector3d(0,0,0.5), chrono.ChVector3d(2,1.5,1), 1000,True,0.01)
    table.SetFixed(True)
    system.GetCollisionSystem().AddCollisionShape(table.GetCollisionShape())

    
    offset_pos = chrono.ChVector3d(-5,0,0)
    offset_pos2 = chrono.ChVector3d(5,0,0)
    cloth_mat = cascade.IsotropicKirchhoff(1000*1000, 0, 0.02)
    mesh = cascade.GetMeshFromWavefrontMesh(chrono.GetChronoDataFile('models/tablecloth/tablecloth.obj'), True, True)
    mesh.Transform(chrono.ChMatrix33dFromTranslationRotation(chrono.ChVector3d(0,0,0), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0,1,0))))
    mesh.Transform(chrono.ChMatrix33dFromTranslationRotation(chrono.ChVector3d(0,0,0), chrono.QuatFromAngleAxis(m.pi, chrono.ChVector3d(1,0,0))))
    mesh.Transform(chrono.ChMatrix33dFromTranslationRotation(chrono.ChVector3d(offset_pos.x,offset_pos.y,offset_pos.z), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0,1,0))))
    mesh.Transform(chrono.ChMatrix33dFromTranslationRotation(chrono.ChVector3d(offset_pos2.x,offset_pos2.y,offset_pos2.z), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0,1,0))))
    cloth = cascade.CCascadeShellMesh(system, mesh, cloth_mat,  True, 0.01)
    cloth.SetTexture(chrono.GetChronoDataFile('models/tablecloth/textures/tile4.jpg'), 2, 2)
    cloth.Initialize()

    
    rope_mat = chrono.ChContactMaterialNSC()
    rope_mat.SetFriction(0.5)
    rope_mat.SetRestitution(0.01)
    rope = cascade.CCascadeLinkShellMesh(system)
    rope.Initialize(table, cloth, chrono.ChFramed(chrono.ChVector3d(offset_pos.x,offset_pos.y,offset_pos.z), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0,1,0))))
    rope.SetConstraintType(cascade.ConstraintType_SHAFT)
    rope.SetCollisionType(cascade.CollisionType_NONE)
    rope.Initialize()

    
    system.EnableGravity(True)

    
    light = irr.ChLightPointNSC()
    light.SetAttenuation(0,0,0)
    light.SetIntensity(20000000)
    light.SetPosition(chrono.ChVector3d(100,100,100))
    lamp = irr.ChVisualSystemIrrlicht()
    lamp.AttachLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    lamp.AttachLight(light)
    lamp.SetCameraVertical(chrono.CameraVerticalDir_Z)
    lamp.Initialize()
    lamp.AddSkyBox()
    lamp.AttachCamera(chrono.ChVector3d(0.5,0.5,-0.5), chrono.ChVector3d(0,0,0))

    
    driver = cascade.CCascadeDriverIRR(lamp)
    driver.SetChaseCamera(chrono.ChVector3d(0,0,0.5), 6.0, 0.5)
    driver.Initialize()

    
    system.SetSolverType(chrono.ChSolver.Type_PARDISO)

    
    step_size = 1 / 50  

    
    steps = 0

    system.GetSolver().AsIterative().SetMaxIterations(300)

    while (lamp.Run()) :

        time = system.GetChTime()

        
        if (time >= 1):
            break

        
        lamp.BeginScene()
        lamp.Render()
        lamp.EndScene()

        
        driver_inputs = driver.GetInputs()

        
        driver.Synchronize(time)
        rope.Synchronize(time)
        cloth.Synchronize(time)
        table.Synchronize(time)
        floor.Synchronize(time)
        lamp.Synchronize(time, driver_inputs)
        system.Synchronize(time, driver_inputs)

        
        driver.Advance(step_size)
        rope.Advance(step_size)
        cloth.Advance(step_size)
        table.Advance(step_size)
        floor.Advance(step_size)
        lamp.Advance(step_size)
        system.Advance(step_size)

        
        steps += 1

    return 0








set_custom_collision_margins = False


def custom_collision_margins(body):
    body.GetCollisionModel().SetMargin(0.001)

cascade.SetDataPath(chrono.GetChronoDataPath() + 'cascade/')

main()