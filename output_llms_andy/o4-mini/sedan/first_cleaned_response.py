import os
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr






chrono.SetChronoDataPath(os.getenv('CHRONO_DATA_DIR', '/usr/local/share/chrono/data/'))


system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))




vehicle = veh.WheeledVehicle(
    system,
    chrono.GetChronoDataFile("vehicle/GenericSedan/Sedan_FullCoord.json"),
    veh.CollisionType_PRIMITIVES)

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)


init_loc = chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT)
vehicle.Initialize(init_loc)
vehicle.SetChassisFixed(False)


tire_vis = veh.VisualizationType_MESH
tire_collision = veh.ChTire.TerrainType_SPHERE  
tire_mesh = chrono.GetChronoDataFile("vehicle/tire/hmmwv_tire_tmeasy/meshes/wheel.obj")

for i in range(vehicle.GetNumTires()):
    tire = veh.TMeasyTire()
    
    tire.SetTMeasyParams(chrono.GetChronoDataFile("vehicle/tire/TMeasyTire.json"))
    tire.SetVisualizationType(tire_vis)
    vehicle.InitializeTire(tire, i, tire_collision)


terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
    chrono.ChVectorD(20, 20, 1))
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
patch.SetTexture(chrono.GetChronoDataFile("vehicle/terrain/textures/tile4.jpg"), 8, 8)
terrain.Initialize()



driver = veh.ChInteractiveDriver(vehicle)
driver.SetDeadband(0.0)
driver.SetSteeringDelta(0.04)
driver.SetThrottleDelta(0.04)
driver.SetBrakingDelta(0.04)
driver.Initialize()


app = veh.ChWheeledVehicleIrrApp(
    vehicle,                  
    "BMW E90 Sedan Demo",     
    irr.dimension2du(1280, 720),
    False,                    
    False)                    


app.SetChaseCameraPosition(chrono.ChVectorD(0.0, -6.0, 2.0))
app.SetChaseCameraSmoothness(30.0)


app.AddTypicalSky()
app.AddTypicalLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"),               
                   chrono.GetChronoDataFile("logo_pychrono.png"),                     
                   chrono.GetChronoDataFile("logo_pychrono_logo.png"),                
                   50, 50)                                                           

app.AddTypicalLights()
app.SetLightDirection(chrono.ChVector3D(-0.5, 0.5, -1))
app.AddCamera(irr.vector3df(0, -10, 5), irr.vector3df(0, 0, 1))


app.AssetBindAll()
app.AssetUpdateAll()
app.SetTimestep(1e-3)





time = 0.0
while app.Run():
    
    app.BeginScene()
    app.DrawAll()

    
    steering = driver.GetSteering()
    throttle = driver.GetThrottle()
    braking = driver.GetBraking()

    
    time = system.GetChTime()
    driver.Synchronize(time)
    vehicle.Synchronize(time, driver.GetInputs(), terrain)
    terrain.Synchronize(time)
    app.Synchronize("", steering, throttle, braking)

    
    driver.Advance(app.GetTimestep())
    vehicle.Advance(app.GetTimestep())
    terrain.Advance(app.GetTimestep())
    app.Advance(app.GetTimestep())

    
    app.EndScene()





print("Simulation finished.")