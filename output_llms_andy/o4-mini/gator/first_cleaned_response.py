import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr






chrono.SetChronoDataPath("/usr/local/share/chrono/")


contact_method = chrono.ChContactMethod_NSM






gator = veh.Gator(contactMethod=contact_method, fixed=False)


init_location = chrono.ChVectorD(0, 0.5, 0)
init_rotation = chrono.ChQuaternionD(1, 0, 0, 0)
gator.Initialize(init_location, init_rotation)


gator.SetTireType(veh.TireModelType_TMEASY)


vis = veh.VisualizationType_MESH
gator.SetChassisVisualizationType(vis)
gator.SetSuspensionVisualizationType(vis)
gator.SetSteeringVisualizationType(vis)
gator.SetWheelVisualizationType(vis)
gator.SetTireVisualizationType(vis)






system = gator.GetSystem()
terrain = veh.RigidTerrain(system)


patch = terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0),
                                           chrono.ChQuaternionD(1, 0, 0, 0)),
                         100.0,   
                         100.0)   


terrain.SetContactFriction(0.9)
terrain.SetContactRestitution(0.01)
terrain.SetContactMaterialProperties(2e7, 0.3, 0.4)


patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 6, 6)


terrain.Initialize()





app = veh.ChIrrApp(system, "Gator on Rigid Terrain", irr.dimension2du(1280, 720))
app.AddTypicalLogo()
app.AddTypicalSky()
app.AddTypicalLights()

app.AddTypicalCamera(irr.vector3df(0, 5, -15), irr.vector3df(0, 1, 0))


app.AssetBindAll()
app.AssetUpdateAll()





driver = veh.ChIrrGuiDriver(app)
driver.SetSteeringDelta(0.1)   
driver.SetThrottleDelta(0.1)   
driver.SetBrakingDelta(0.2)    
driver.Initialize()





step_size = 1.0 / 50    

app.SetTimestep(step_size)
app.SetTryRealtime(True)

while app.GetDevice().run():
    t = system.GetChTime()

    
    driver.Synchronize(t)

    
    terrain.Synchronize(t)
    gator.Synchronize(t, driver.GetInputs(), terrain)

    
    app.BeginScene(True, True, irr.SColor(255, 140, 161, 192))
    app.DrawAll()
    app.EndScene()

    
    app.DoStep()