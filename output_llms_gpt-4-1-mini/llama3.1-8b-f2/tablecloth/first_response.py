import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.cascade as cascade
import pychrono.parsers as parsers
import os
import math as m

def main():
    #print("Copyright (c) 2017 projectchrono.org\nChrono version: ", CHRONO_VERSION , "\n\n")

    #  Create the Cascade system
    system = cascade.CascadeSystemNSC()

    # Set the collision system type
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # Create the floor
    #floor = cascade.CCascadeBodyEasyBox(system, chrono.ChVector3d(0,0,0), chrono.ChVector3d(10,10,1), 1000,True,0.01)
    #floor.SetFixed(True)
    #system.GetCollisionSystem().AddCollisionShape(floor.GetCollisionShape())

    # Create the table
    table = cascade.CCascadeBodyEasyBox(system, chrono.ChVector3d(0,0,0.5), chrono.ChVector3d(2,1.5,1), 1000,True,0.01)
    table.SetFixed(True)
    system.GetCollisionSystem().AddCollisionShape(table.GetCollisionShape())

    # Create the cloth
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

    # Create the rope
    rope_mat = chrono.ChContactMaterialNSC()
    rope_mat.SetFriction(0.5)
    rope_mat.SetRestitution(0.01)
    rope = cascade.CCascadeLinkShellMesh(system)
    rope.Initialize(table, cloth, chrono.ChFramed(chrono.ChVector3d(offset_pos.x,offset_pos.y,offset_pos.z), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0,1,0))))
    rope.SetConstraintType(cascade.ConstraintType_SHAFT)
    rope.SetCollisionType(cascade.CollisionType_NONE)
    rope.Initialize()

    # Enable gravity
    system.EnableGravity(True)

    # Create the large point light
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

    # Create the driver
    driver = cascade.CCascadeDriverIRR(lamp)
    driver.SetChaseCamera(chrono.ChVector3d(0,0,0.5), 6.0, 0.5)
    driver.Initialize()

    # Simulation loop
    system.SetSolverType(chrono.ChSolver.Type_PARDISO)

    # Number of simulation steps between miscellaneous events
    step_size = 1 / 50  # FPS = 50

    # Initialize simulation frame counter
    steps = 0

    system.GetSolver().AsIterative().SetMaxIterations(300)

    while (lamp.Run()) :

        time = system.GetChTime()

        # End simulation
        if (time >= 1):
            break

        # Draw scene
        lamp.BeginScene()
        lamp.Render()
        lamp.EndScene()

        # Get driver inputs
        driver_inputs = driver.GetInputs()

        # Update modules (process inputs from other modules)
        driver.Synchronize(time)
        rope.Synchronize(time)
        cloth.Synchronize(time)
        table.Synchronize(time)
        floor.Synchronize(time)
        lamp.Synchronize(time, driver_inputs)
        system.Synchronize(time, driver_inputs)

        # Advance simulation for one timestep for all modules
        driver.Advance(step_size)
        rope.Advance(step_size)
        cloth.Advance(step_size)
        table.Advance(step_size)
        floor.Advance(step_size)
        lamp.Advance(step_size)
        system.Advance(step_size)

        # Increment frame number
        steps += 1

    return 0


# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

# If true, use provided callback to set custom rigid body collision margins
set_custom_collision_margins = False

# Callback function to set custom rigid body collision margins
def custom_collision_margins(body):
    body.GetCollisionModel().SetMargin(0.001)

cascade.SetDataPath(chrono.GetChronoDataPath() + 'cascade/')

main()