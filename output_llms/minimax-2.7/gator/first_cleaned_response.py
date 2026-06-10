import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr





chrono.SetDataPath("C:/Chrono/data/")   




system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVector(0, -9.81, 0))   




terrain = veh.RigidTerrain(system)


patch = terrain.AddPatch(
    chrono.ChVector(0, 0, 0),                
    chrono.ChQuaternion(1, 0, 0, 0),         
    200,                                      
    200                                       
)


patch.SetFriction(0.9)
patch.SetRestitution(0.0)


patch.SetTexture(chrono.GetChronoDataFile("textures/tile4.png"))

terrain.Initialize()





init_loc = chrono.ChVector(0.0, 0.5, 0.0)    
init_rot = chrono.ChQuaternion(1, 0, 0, 0)    


gator = veh.Gator(init_loc, init_rot, veh.TMeasyTire)


gator.SetChassisVisualizationType(veh.ChassisVisType.Mesh)
gator.SetWheelVisualizationType(veh.WheelVisType.Mesh)
gator.SetTireVisualizationType(veh.TireVisType.Mesh)


gator.Initialize(system)





driver = veh.ChIrrVehicleInteractiveDriver(gator.GetVehicle())
driver.Initialize()




app = irr.ChIrrApp(
    system,
    "Gator – PyChrono Demo",               
    irr.dimension2du(1280, 720)            
)


app.SetSkyBox()
app.AddTypicalLights()
app.AddCamera(irr.vector3df(0, 5, -8), irr.vector3df(0, 1, 0))


app.SetDriver(driver)




app.SetTimestep(0.02)      
app.SetRealTime(True)      








device = app.GetDevice()
while device.run():
    app.BeginScene()
    app.DrawAll()
    app.EndScene()

    
    app.Synchronize()
    app.Advance(app.GetTimestep())


print("Simulation finished.")