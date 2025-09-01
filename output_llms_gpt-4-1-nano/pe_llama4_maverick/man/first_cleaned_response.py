import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr








veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')






initLoc = chrono.ChVector3d(0, 0, 1.0)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)
vehicle = veh.MAN_10t(initLoc, initRot)


vehicle.SetTireType(veh.TireModelType_TMEASY)


vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetTireCollisionType(veh.CollisionType_ENVELOPE)


vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)


terrain = veh.RigidTerrain(vehicle.GetSystem())
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, 
                         chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
                         100.0, 100.0)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('MAN 10t')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle)


driver = veh.ChIrrGuiDriver(vis)


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   
driver.SetSteeringDelta(driver.GetSteeringMax() / steering_time * 0.02)
driver.SetThrottleDelta(driver.GetThrottleMax() / throttle_time * 0.02)
driver.SetBrakingDelta(driver.GetBrakingMax() / braking_time * 0.02)


while vis.Run() :
    time = vehicle.GetSystem().GetChTime()

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(driver.GetInputModeAsString(), driver_inputs)

    
    driver.Advance(0.02)
    terrain.Advance(0.02)
    vehicle.Advance(0.02)
    vis.Advance(0.02)

    
    vehicle.GetSystem().DoStepDynamics(0.02)


del vis
del driver
del vehicle
del terrain