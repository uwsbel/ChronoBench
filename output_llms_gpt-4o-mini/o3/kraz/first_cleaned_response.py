import math
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr





chrono.SetChronoDataPath(chrono.GetChronoDataPath())          
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")      

contact_method        = chrono.ChContactMethod_NSC            
system_track_time_step = 2e-3                                 
render_fps             = 50                                   




print("Initialising Kraz vehicle ...")
vehicle = veh.Kraz(contact_method,               
                   True,                         
                   chrono.ChassisCollisionType_NONE)

init_pos = chrono.ChVectorD(0.0, 0.0, 0.5)       
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)      
vehicle.Initialize(chrono.ChCoordsysD(init_pos, init_rot))


vehicle.SetChassisFixed(False)                   
vehicle.SetTireType(veh.TireType_RIGID)          




print("Creating rigid terrain ...")
terrain = veh.RigidTerrain(vehicle.GetSystem())


mat = chrono.ChMaterialSurfaceNSC()
mat.SetFriction(0.9)
mat.SetRestitution(0.01)


patch = terrain.AddPatch(mat,
                         chrono.ChVectorD(0, 0, 0),           
                         chrono.ChVectorD(0, 0, 1),           
                         200, 200)                            
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.6, 0.6, 0.6))
terrain.Initialize()




print("Starting Irrlicht application ...")
app = veh.ChWheeledVehicleIrrApp(vehicle,
                                 "Kraz Rigid-Terrain Demo",
                                 irr.dimension2du(1280, 720))

app.SetSkyBox()
app.AddTypicalLights(chrono.ChVectorD(30, 100, 30),
                     chrono.ChVectorD(-30, 100, 30),
                     250, 130)
app.AddLightWithShadow(chrono.ChVectorD(60, 120, 60),
                       chrono.ChVectorD(0, 0, 0),
                       100, 1, 80, 80)
app.SetChaseCamera(chrono.ChVectorD(0, 0.5, 0),   
                   8.0,                           
                   0.5)                           
app.SetStepsize(system_track_time_step)
app.Initialize()




driver = veh.ChIrrGuiDriver(app)
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.1)
driver.Initialize()




realtime_timer = chrono.ChRealtimeStepTimer()
render_interval = math.ceil((1.0 / render_fps) / system_track_time_step)
step_number = 0

print("Simulation loop started. Press W/A/S/D, Space (brake), and P (parking).")
while app.GetDevice().run():

    
    if step_number % render_interval == 0:
        app.BeginScene()
        app.DrawAll()
        app.EndScene()

    
    time = vehicle.GetSystem().GetChTime()

    
    
    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver.GetInputs(), terrain)
    app.Synchronize("Kraz demo", driver.GetInputs())

    
    driver.Advance(system_track_time_step)
    terrain.Advance(system_track_time_step)
    vehicle.Advance(system_track_time_step)
    app.Advance(system_track_time_step)

    
    realtime_timer.Spin(system_track_time_step)
    step_number += 1

print("Demo terminated.")