import os
import math
import numpy as np
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "data"))


mysystem = chrono.ChSystemNSC()
mysystem.SetSolverType(chrono.ChSolver.Type_APGD)


gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisCollisionType(chrono.ChCollisionType_AABBTREE)
gator.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
gator.SetInitVelocity(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 0))
gator.Initialize(mysystem)


gator.GetChassisBody().AddVisualShape(chrono.ChVisualShapeBox(gator.GetChassisWidth(), gator.GetChassisLength(), gator.GetChassisHeight()))
gator.GetChassisBody().GetVisualShape(0).SetColor(chrono.ChColor(1, 0, 0))


terrain = veh.RigidTerrain(mysystem)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT), 100, 100)
terrain.Initialize()


driver = veh.ChIrrGuiDriver()
driver.SetVehicle(gator)
driver.SetSteeringMode(veh.ChIrrGuiDriver.STEERINGMODE_keyboard)
driver.Initialize(mysystem)


sensor_manager = veh.ChSensorManager(gator.GetSystem())
point_light = sensor_manager.AddPointLight(chrono.ChVectorD(0, 5, 0), chrono.ChColor(1, 1, 1))
camera = sensor_manager.AddCamera(chrono.ChVectorD(0, 1.5, 3), chrono.ChVectorD(0, 0, 0))
camera.SetImageSize(chrono.ChVector2D(800, 600))
camera.SetFocalLength(35)
camera.SetAperture(2.2)
camera.SetExposureTime(1/60)
camera.SetISO(100)
camera.SetAntiAliasingSamples(4)
camera.SetOutputPath("camera_images")


while driver.GetSystem().GetChTime() < 10:
    
    driver.Update(1e-3)

    
    terrain.Synchronize(1e-3)

    
    gator.Synchronize(1e-3, True)

    
    sensor_manager.Update(1e-3)

    
    driver.GetSystem().DoStepDynamics(1e-3)

    
    camera.Render()

    
    print("Simulation time: ", driver.GetSystem().GetChTime())