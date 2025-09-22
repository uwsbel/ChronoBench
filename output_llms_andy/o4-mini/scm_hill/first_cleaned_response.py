import os
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import pychrono.vehicle.hmmwv as hmmwv




chrono.SetChronoDataPath(os.environ.get('CHRONO_DATA_PATH', '')) 
system = chrono.ChSystemSMC()
step_size = 1e-3





vehicle = hmmwv.HMMWV_Full()
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
vehicle.SetChassisCollisionType(veh.ChassisCollisionType_BOX)

init_pos = chrono.ChCoordsysD(chrono.ChVectorD(0, 0.8, 0), chrono.QUNIT)
vehicle.SetInitPosition(init_pos)
vehicle.SetInitFwdVel(0.0)    
vehicle.Initialize(system)


for t in vehicle.GetTireList():
    t.SetRollingMode(veh.TireRollingMode::ROLLING_MODE_RIGID)




terrain = veh.SCMDeformableTerrain(system)

terrain.SetPlane(0, 0, 1, 0)  

terrain.SetSoilParameters(
    Kphi=1.0e6,   
    Kc=1.0e5,     
    n=1.1,        
    coh=0.0,      
    mu=0.9,       
    cr=0.01,      
    beta=0.0      
)

terrain.SetMeshResolutions(0.05, 0.05)

terrain.SetTextureScale(8, 8)

heightmap_file = chrono.GetChronoDataPath() + '/vehicle/hmmwv/terrain/height_maps/height_map.png'
terrain.Initialize(heightmap_file)

terrain.SetPlotType(veh.SCMDeformableTerrain.PLOT_PRESSURE_Y)




app = chronoirr.ChIrrApp(
    system,                                    
    'HMMWV on Deformable Terrain (SCM)',       
    chronoirr.dimension2du(1280, 720)          
)
app.AddTypicalSky()
app.AddTypicalLights()
app.AddCamera(chronoirr.ChVectorD(0, 2, -6), chronoirr.ChVectorD(0, 0, 0))
app.SetTryRealtime(True)
app.SetTimestep(step_size)


app.AssetBindAll()
app.AssetUpdateAll()




driver = veh.ChIrrGuiDriverIrrlicht(vehicle, app)
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.02)
driver.Initialize()




while app.Run():
    t = system.GetChTime()

    
    driver_inputs = driver.GetInputs()

    
    vehicle.Synchronize(t, driver_inputs, terrain)
    terrain.Synchronize(t)
    driver.Synchronize(t)
    
    
    app.BeginScene(True, True, chronoirr.SColor(255, 140, 161, 192))
    app.DrawAll()
    app.EndScene()

    
    vehicle.Advance(step_size)
    terrain.Advance(step_size)
    driver.Advance(step_size)
    system.DoStepDynamics(step_size)