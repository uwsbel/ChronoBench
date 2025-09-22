import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr




step_size          = 2.5e-3        
tire_step_size     = 1.0e-3        
render_steps       = 5             
contact_method     = chrono.ChContactMethod_SMC
data_path          = chrono.GetChronoDataPath()               
heightmap_file     = data_path + "terrain/heightmap.bmp"      
heightmap_sizeX    = 40.0         
heightmap_sizeY    = 40.0
heightmap_maxZ     = 0.50         
start_loc          = chrono.ChVectorD(0, 0, 1.0)              
start_yaw          = 0.0




chrono.SetChronoDataPath(data_path)
sys = chrono.ChSystemSMC()               
sys.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))




print("Creating the HMMWV vehicle ...")
vehicle = veh.HMMWV_Full(sys,
                         driveType   = veh.DriveType_4WD,
                         contactMethod=contact_method)

vehicle.SetInitPosition( chrono.ChCoordsysD(start_loc,
                          chrono.Q_from_AngZ(start_yaw)) )
vehicle.SetChassisFixed(False)
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(tire_step_size)


vehicle.SetChassisVisualizationType(   veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(  veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(     veh.VisualizationType_MESH)
vehicle.Initialize()




print("Creating SCM deformable terrain ...")
terrain = veh.SCMDeformableTerrain(sys)


terrain.SetPlane( chrono.ChCoordsysD( chrono.ChVectorD(0,0,0),
                                      chrono.QUNIT ) )


terrain.Initialize(0.04)          


terrain.SetSoilParameters( 12e5,     
                           0,        
                           1.1,      
                           1.3e4,    
                           30.0,     
                           0.02,     
                           4e7,      
                           3e4)      


print("Loading height-map  …")
terrain.LoadHeightMap(heightmap_file,
                      chrono.ChVectorD(-heightmap_sizeX/2,
                                       -heightmap_sizeY/2, 0),
                      heightmap_sizeX, heightmap_sizeY,
                      heightmap_maxZ)


terrain.SetTexture( data_path + "terrain/textures/soil.jpg", 10, 10)




print("Starting Irrlicht interface …")
app = veh.ChVehicleIrrApp(vehicle,
                          "HMMWV on SCM deformable terrain",
                          irr.dimension2du(1280, 720))
app.SetSkyBox()
app.AddTypicalLights( chrono.ChVectorD( 30,  30, 100),
                      chrono.ChVectorD(-30, -30, 100),
                      250, 130)
app.AddTypicalLogo()
app.SetChaseCamera( chrono.ChVectorD(0,0.6,0),
                    4.0,        
                    0.5)        

app.SetTimestep(step_size)
app.Initialize()


driver = veh.ChIrrGuiDriver(app)
driver.SetSteeringDelta( 0.04 )
driver.SetThrottleDelta( 0.02 )
driver.SetBrakingDelta(  0.20 )
driver.SetGains( 0.50, 0.005 )      
driver.Initialize()




realtime_timer = chrono.ChRealtimeStepTimer()
step_number    = 0

print("Simulation loop started, press ESC to quit.")
while app.GetDevice().run():
    
    if step_number % render_steps == 0:
        app.BeginScene(True, True, irr.SColor(255,140,161,192))
        app.DrawAll()
        app.EndScene()

    
    time    = sys.GetChTime()
    inputs  = driver.GetInputs()

    
    driver.Synchronize (time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, inputs, terrain)
    app.Synchronize    ("HMMWV demo", inputs)

    
    driver.Advance (step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    sys.DoStepDynamics(step_size)

    
    realtime_timer.Spin(step_size)
    step_number += 1