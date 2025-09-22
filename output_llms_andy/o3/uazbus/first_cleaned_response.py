import pychrono                         as chrono
import pychrono.vehicle                 as veh
import pychrono.irrlicht                as irr




step_size       = 2e-3          
t_end           = 60.0          
gravity         = chrono.ChVectorD(0, 0, -9.81)
contact_fric    = 0.9           
contact_rest    = 0.1           




chrono.SetChronoDataPath(chrono.GetChronoDataPath())         
system      = chrono.ChSystemSMC()
system.Set_G_acc(gravity)


realtime_timer = chrono.ChRealtimeStepTimer()




init_pos   = chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT)  

vehicle = veh.WheeledVehicle(system,
                              veh.GetDataFile("vehicle/uaz/UAZBUS.json"),
                              veh.ChContactMethod_SMC)

vehicle.Initialize(init_pos)
vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_NONE)
vehicle.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)


powertrain = veh.ShaftsPowertrain(veh.GetDataFile("vehicle/uaz/UAZBUS_ShaftsPowertrain.json"))
vehicle.InitializePowertrain(powertrain)




terrain = veh.RigidTerrain(system)
patch   = terrain.AddPatch(chrono.ChCoordsysD(),            
                           chrono.ChVectorD(400, 400, 1),   
                           0,                               
                           True)                            

patch.SetContactFrictionCoefficient(contact_fric)
patch.SetContactRestitutionCoefficient(contact_rest)
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)
terrain.Initialize()




driver = veh.ChIrrGuiDriver(vehicle.GetSystem())   
driver.SetSteeringDelta( 1.0 * chrono.CH_2PI / 180)    
driver.SetThrottleDelta( 0.1)
driver.SetBrakingDelta(  0.2)




app = veh.ChWheeledVehicleIrrApp(vehicle,
                                 "UAZBUS real-time demo",
                                 irr.dimension2du(1280, 720))

app.SetSkyBox()
app.AddTypicalLights(chrono.ChVectorD( 30,  30, 60), 300, chrono.ChColor(1,1,1))
app.AddTypicalLights(chrono.ChVectorD(-30, -30, 60), 300, chrono.ChColor(1,1,1))
app.SetChaseCamera(chrono.ChVectorD(0, 0, 1.75),      
                   6.0,                               
                   0.5)                               
app.Initialize()




print("Controls:  arrow L/R = steer, arrow U/D = throttle/brake, SPACE = full brake, q=quit")

time = 0.0
while app.GetDevice().run() and time < t_end:

    
    app.BeginScene()
    app.DrawAll()               
    app.EndScene()

    
    driver_inputs = driver.GetInputs()
    vehicle.Synchronize(time, driver_inputs.steering, driver_inputs.throttle, driver_inputs.braking)
    terrain.Synchronize(time)
    driver.Synchronize(time)

    
    system.DoStepDynamics(step_size)
    driver.Advance(step_size)
    vehicle.Advance(step_size)
    terrain.Advance(step_size)

    
    realtime_timer.Spin(step_size)

    time += step_size

print("Execution finished.")