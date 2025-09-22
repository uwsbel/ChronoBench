import pychrono as chrono
import pychrono_vehicle as veh
import pychrono.vehicle.citybus as citybus
import pychrono_irrlicht as chronoirr


chrono.SetChronoDataPath(chrono.GetChronoDataPath())  
chrono.SetChronoVerbose(False)

time_step = 1.0 / 50  



init_loc = chrono.ChVectorD(0, 0, 0.5)                         
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)                    


vehicle = citybus.CityBus()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.ChassisCollisionType_NONE)  
vehicle.SetInitPosition(chrono.ChCoordsysD(init_loc, init_rot))

vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.Initialize()


vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)


terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
                         sizeX=100, sizeY=100)
patch.SetTexture(veh.GetDataFile("terrain/textures/grass_green_detail.png"), 200, 200)
patch.SetColor(chrono.ChColor(0.4, 0.6, 0.4))
patch.SetFriction(0.9f)
patch.SetMaterialProperties(2e7, 0.3, 1e7)  
terrain.Initialize()



driver = veh.ChIrrGuiDriver(vehicle)
driver.Initialize()
driver.SetInputDelay(0.15)
driver.SetSteeringDelta(render_step=0.015)
driver.SetThrottleDelta(render_step=0.1)
driver.SetBrakingDelta(render_step=0.1)


application = chronoirr.ChIrrApp(vehicle.GetSystem(), "CityBus on RigidTerrain Demo",
                                chronoirr.dimension2du(1280, 720))

application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chronoirr.vector3df(0, -12, 3))  


application.SetTimestep(time_step)
application.SetTryRealtime(True)  


app_camera_offset = chrono.ChVectorD(-8, 0, 2.5)
application.GetDevice().getSceneManager().getActiveCamera().setPosition(
    chronoirr.vector3df(float(init_loc.x - 8), float(init_loc.y), float(init_loc.z + 2.5)))
application.GetDevice().getSceneManager().getActiveCamera().setTarget(
    chronoirr.vector3df(float(init_loc.x), float(init_loc.y), float(init_loc.z + 1)))


application.AssetBindAll()
application.AssetUpdateAll()
application.AddShadowAll()


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()

    
    driver_inputs = driver.GetInputs()
    steering = driver_inputs.m_steering
    throttle = driver_inputs.m_throttle
    braking = driver_inputs.m_braking

    
    vehicle.GetSteeringController().SetInput(steering)
    vehicle.GetPowertrain().SetThrottle(throttle)
    vehicle.ApplyBrake(braking)

    
    time = vehicle.GetSystem().GetChTime()
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    application.Synchronize("", steering, throttle, braking)

    
    driver.Advance(time_step)
    terrain.Advance(time_step)
    vehicle.Advance(time_step)
    application.Advance(time_step)

    application.EndScene()