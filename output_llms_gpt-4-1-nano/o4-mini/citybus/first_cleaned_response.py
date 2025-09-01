import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh






chrono.SetChronoDataPath("C:/Chrono/chrono_data/")  
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))







bus = veh.WheeledVehicle(system, 
    veh.GetDataFile("vehicle/citybus/CityBus.json"), 
    chrono.ChContactMethod_NSC)


init_loc = chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.QUNIT)
bus.Initialize(init_loc)


bus.SetChassisVisualizationType(veh.VisualizationType_MESH)
bus.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVE)
bus.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVE)
bus.SetWheelVisualizationType(veh.VisualizationType_MESH)



for axle in range(bus.GetNumberAxles()):
    for side in [0, 1]:
        tire_json = "vehicle/citybus/tire/tm_tire.json"
        tire = veh.TMeasyTire(tire_json, veh.ChTire.TMEASY)
        bus.InitializeTire(tire, axle, side)





terrain = veh.RigidTerrain(system)


patch = terrain.AddPatch(
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
    sizeX=200, sizeY=200)


patch.SetMaterialSurface(veh.ChMaterialSurfaceNSC())
mat = patch.GetMaterialSurface()
mat.SetFriction(0.9)
mat.SetRestitution(0.1)


patch.SetTexture(
    veh.GetDataFile("terrain/textures/tile4.jpg"),
    scale_x=20, scale_y=20)

terrain.Initialize()





app = chronoirr.ChIrrApp(
    system,
    "CityBus on Rigid Terrain",
    chronoirr.dimension2du(1280, 720))

app.AddTypicalLogo()
app.AddTypicalSky()
app.AddTypicalLights()
app.AddTypicalCamera(chronoirr.ChVectorD(-8, 4, -8))
app.SetChaseCameraPosition(chronoirr.ChVectorD(0, 3, -8))


app.AssetBindAll()
app.AssetUpdateAll()





driver = veh.ChIrrGuiDriver(app)
driver.SetSteeringDelta(0.04)   
driver.SetThrottleDelta(0.05)
driver.SetBrakingDelta(0.1)
driver.Initialize()





time_step = 1.0 / 50.0
max_time   = 100.0

while app.GetDevice().run() and system.GetChTime() < max_time:
    
    steering, throttle, braking = driver.GetInput()
    
    
    time = system.GetChTime()
    driver.Synchronize(time)
    bus.Synchronize(time, steering, throttle, braking, terrain)
    terrain.Synchronize(time)
    
    
    system.DoStepDynamics(time_step)
    
    
    app.BeginScene()
    app.DrawAll()
    app.EndScene()


app.GetDevice().drop()