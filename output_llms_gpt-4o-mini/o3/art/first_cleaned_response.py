import os
import math
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr




chrono.SetChronoDataPath(chrono.GetChronoDataDir())            
veh.SetDataPath(chrono.GetChronoDataDir() + "vehicle/")        


step_size      = 1.0e-3                                        
render_fps     = 50                                            
render_interval= 1.0 / float(render_fps)                       
render_steps   = math.ceil(render_interval / step_size)




contact_method = chrono.ChContactMethod_NSC
system         = chrono.ChSystemNSC() if contact_method==chrono.ChContactMethod_NSC \
                 else chrono.ChSystemSMC()




init_loc     = chrono.ChVectorD(0, 0, 0.1)                     
init_rot     = chrono.ChQuaternionD(1, 0, 0, 0)                

vehicle = veh.WheeledVehicle(
            system,
            veh.GetDataFile("ARTcar/ARTcar_Vehicle.json"),
            contact_method)

vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)

vehicle.Initialize(chrono.ChCoordsysD(init_loc, init_rot))


powertrain = veh.ShaftsPowertrain(system,
                veh.GetDataFile("ARTcar/ARTcar_Powertrain.json"))
vehicle.InitializePowertrain(powertrain)

tire_file = veh.GetDataFile("ARTcar/ARTcar_TMeasyTire.json")
for axle in range(vehicle.GetNumberAxles()):
    for side in range(2):
        vehicle.GetWheelBody(axle, side).SetCollide(True)
        tire = veh.TMeasyTire(tire_file)
        vehicle.InitializeTire(tire, axle, side)




terrain = veh.RigidTerrain(system)
patch   = terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVectorD(0,0,0),chrono.QUNIT),
                           chrono.ChVectorD(60, 10, 0.2),       
                           False)                               
patch.SetTexture(veh.GetDataFile("textures/5.jpg"), 10, 10)     
terrain.Initialize()




app = veh.ChVehicleIrrApp(vehicle, "ARTcar on rigid terrain",
                          irr.dimension2du(1024,768))

app.SetSkyBox()
app.AddTypicalLights()
app.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 1.2), 6.0, 0.5)   
app.Initialize()


driver = veh.ChIrrGuiDriver(app)
driver.SetSteeringDelta(1.0 / 200.0)
driver.SetThrottleDelta(1.0 / 150.0)
driver.SetBrakingDelta (1.0 / 150.0)
driver.Initialize()




realtime_timer = chrono.ChRealtimeStepTimer()
frame          = 0

while app.GetDevice().run():
    
    if frame % render_steps == 0:
        app.BeginScene(True, True, irr.SColor(255,140,161,192))
        app.DrawAll()
        app.EndScene()

    
    driver_inputs = driver.GetInputs()

    
    time = system.GetChTime()
    driver.Synchronize (time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    app.Synchronize  ("ARTcar demo", driver_inputs)

    
    driver.Advance  (step_size)
    terrain.Advance (step_size)
    vehicle.Advance (step_size)
    app.Advance     (step_size)

    
    realtime_timer.Spin(step_size)

    frame += 1