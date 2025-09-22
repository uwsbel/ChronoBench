import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr




chrono.SetChronoDataPath(chrono.GetChronoDataPath())  
chrono.SetChronoThread(0)  


time_step = 1.0 / 50.0   
contact_method = chrono.ChContactMethod_SMC





vehicle = veh.HMMWV_Full(contact_method)


vehicle.SetChassisCollisionType(veh.ChassisCollisionType_NONE)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.2, 0),
                                           chrono.Q_from_AngY(0.0)))
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(1e-3)


vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)

vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)


vehicle.Initialize()




terrain = veh.ChMeshTerrain(vehicle.GetSystem())


terrain.LoadMesh('Highway_col.obj',        
                 'Highway_vis.obj')        


terrain.SetMaterialSurface(0.9, 0.1, 0.02)

terrain.Initialize()




app = chronoirr.ChIrrApp(vehicle.GetSystem(),
                         "HMMWV Mesh Terrain",
                         chronoirr.dimension2du(1280, 720),
                         chronoirr.HorizontalDir_Z)


app.AddTypicalCamera(chronoirr.vector3df(-10, 3,  0),
                     chronoirr.vector3df(  0, 0,  0))


app.AddTypicalLights()
app.AddSkyBox()


app.SetTimestep(time_step)
app.SetTryRealtime(True)




driver = veh.ChIrrGuiDriver(app)
driver.SetThrottleDelta(0.06)
driver.SetBrakingDelta(0.1)
driver.SetSteeringDelta(0.04)
driver.Initialize()




app.AssetBindAll()
app.AssetUpdateAll()




while app.GetDevice().run():
    t = vehicle.GetSystem().GetChTime()

    
    steering = driver.GetSteering()
    throttle = driver.GetThrottle()
    braking  = driver.GetBraking()

    
    driver.Synchronize(t)
    terrain.Synchronize(t)
    vehicle.Synchronize(t, steering, throttle, braking, terrain)

    
    app.BeginScene()
    app.DrawAll()
    app.EndScene()

    
    driver.Advance(time_step)
    terrain.Advance(time_step)
    vehicle.Advance(time_step)
    app.Advance(time_step)