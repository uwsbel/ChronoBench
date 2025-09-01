import os
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.vehicle.m113 as m113
import pychrono.irrlicht as irr




chrono.SetChronoDataPath(os.environ.get('CHRONO_DATA_DIR', ''))   
chrono.SetChronoEngineUpdateMode(chrono.ChEngineUpdateMode_FULL_UPDATE)





contact_method = chrono.ChMaterialSurfaceSMC

vehicle = m113.M113_Vehicle(contact_method)
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTrackShoeVisualizationType(veh.VisualizationType_MESH)
vehicle.SetBrakeType(m113.M113_BrakeType_SHAFTS)
vehicle.SetTrackShoeType(m113.M113_TrackShoeType_SINGLE_PIN)


init_loc = chrono.ChVectorD(0, 0.4, 0)
init_rot = chrono.QUNIT
vehicle.Initialize(chrono.ChCoordsysD(init_loc, init_rot))




terrain = veh.RigidTerrain(vehicle.GetSystem())


patch_mat = chrono.SMC_SurfaceSMC()
patch = terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVectorD(0,0,0), chrono.QUNIT),
                         chrono.ChVectorD(50,50,1))
patch.SetMaterialSurface(patch_mat)


patch_mat.SetFriction(0.8)
patch_mat.SetRestitution(0.1)
patch_mat.SetYoungModulus(2e7)
patch_mat.SetPoissonRatio(0.3)

terrain.Initialize()





app = veh.ChIrrApp(vehicle,                                       
                   "M113 Tracked Vehicle Demo",                  
                   irr.dimension2du(1024, 768),                  
                   False, False,                                  
                   True,                                          
                   chrono.GetChronoDataPath() + 'textures/')     

app.AddTypicalLogo()
app.AddTypicalSky()
app.AddTypicalLights()

app.AddTypicalCamera(irr.vector3df(0.0, 2.0, -6.0),
                     irr.vector3df(0.0, 0.4, 0.0))


app.AssetBindAll()
app.AssetUpdateAll()




driver = veh.ChIrrGuiDriver(app)



driver.SetThrottleDelta(0.1)
driver.SetBrakingDelta(0.2)
driver.SetSteeringDelta(0.0)  
driver.Initialize()




timestep = 1.0 / 60.0
while app.GetDevice().run():
    time = vehicle.GetSystem().GetChTime()

    
    driver_inputs = driver.GetInputs()

    
    vehicle.Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time)
    app.Synchronize(time)

    
    app.BeginScene()
    app.DrawAll()
    app.EndScene()

    
    vehicle.Advance(timestep)
    terrain.Advance(timestep)
    driver.Advance(timestep)
    app.Advance(timestep)