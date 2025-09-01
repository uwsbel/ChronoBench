import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr




chrono.SetChronoDataPath(chrono.GetChronoDataFilePath())   
SCENE_TITLE = "PyChrono FEDA Demo"


timestep = 1.0 / 50.0   





contact_method = chrono.ChMaterialSurface.ContactMethod.DVI


vehicle = veh.FedaVehicle(contact_method=contact_method)

init_loc = chrono.ChVectorD(0, 0, 0.5)  
init_rot = chrono.QUNIT
vehicle.Initialize(chrono.ChCoordsysD(init_loc, init_rot))


vehicle.SetChassisVisualizationType(veh.VisualizationType.MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType.MESH)
vehicle.SetSteeringVisualizationType(veh.VisualizationType.MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType.MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType.MESH)


tire_left = veh.TMeasyTire("TMeasy", veh.VisualizationType.MESH)
tire_right = veh.TMeasyTire("TMeasy", veh.VisualizationType.MESH)
for axle in vehicle.GetAxles():
    axle.left_tire = tire_left
    axle.right_tire = tire_right




terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0),
                                           chrono.QUNIT),
                         chrono.VectorD(200, 200, 1))
patch.SetMaterialSurface(veh.CCMaterials.CreateMaterial(contact_method))
patch.SetTexture(chrono.GetChronoDataFile("terrain/grass.jpg"), 200, 200)
terrain.Initialize()




app = irr.ChIrrApp(vehicle.GetSystem(),
                   SCENE_TITLE,
                   irr.dimension2du(1280, 720))

app.AddTypicalSky()
app.AddTypicalLogo()
app.AddTypicalLights()
app.AddTypicalCamera(irr.vector3df(0.0, 5.0, -10.0),   
                     irr.vector3df(0.0, 0.5, 0.0))     
app.SetTimestep(timestep)




driver = veh.ChIrrGuiDriver(app)
driver.SetSteeringDelta(0.04)
driver.SetThrottleDelta(0.04)
driver.SetBrakingDelta(0.1)
driver.Initialize()




app.AssetBindAll()
app.AssetUpdateAll()




time = 0.0
while app.GetDevice().run():
    
    driver_inputs = driver.GetInputs()
    steering = driver_inputs.get_steering()
    throttle = driver_inputs.get_throttle()
    braking = driver_inputs.get_braking()

    
    time = vehicle.GetSystem().GetChTime()
    driver.Synchronize(time)
    vehicle.Synchronize(time, steering, throttle, braking, terrain)
    terrain.Synchronize(time)

    
    driver.Advance(timestep)
    vehicle.Advance(timestep)
    terrain.Advance(timestep)

    
    app.BeginScene()
    app.DrawAll()
    app.EndScene()