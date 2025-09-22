import math
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr




contact_method          = chrono.ChContactMethod_NSC   
vehicle_vis_type        = veh.VisualizationType_MESH   
tire_vis_type           = veh.VisualizationType_MESH
powertrain_model        = veh.PowertrainModelType_SHAFTS
tire_model              = veh.TireModelType.TMEASY


init_loc   = chrono.ChVectorD(0, 0, 0.5)               
init_yaw   = 0.0                                       


step_size          = 1.0e-3          
render_fps         = 50              
render_interval    = 1.0 / render_fps
simulation_end     = 60.0            


chase_dist   = 6.0      
chase_height = 1.2      


chrono.SetChronoDataPath(chrono.GetChronoDataPath())




system = chrono.ChSystemNSC() if contact_method == chrono.ChContactMethod_NSC \
         else chrono.ChSystemSMC()




terrain = veh.RigidTerrain(system)

ground_mat = chrono.ChMaterialSurfaceNSC()  
patch = terrain.AddPatch(
    ground_mat,
    chrono.ChVectorD(0, 0, 0),             
    chrono.ChVectorD(0, 0, 1),             
    300, 300)                              


patch.GetVisualShape().SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"), 200, 200)
terrain.Initialize()




vehicle = veh.FEDA(system,
                   contact_method,
                   powertrain_model,
                   tire_model,
                   veh.ChassisCollisionType_NONE)

vehicle.SetChassisVisualizationType(vehicle_vis_type)
vehicle.SetSuspensionVisualizationType(vehicle_vis_type)
vehicle.SetSteeringVisualizationType(vehicle_vis_type)
vehicle.SetWheelVisualizationType(vehicle_vis_type)
vehicle.SetTireVisualizationType(tire_vis_type)

vehicle.Initialize(chrono.ChCoordsysD(init_loc,
                                      chrono.ChQuaternionD(chrono.Q_from_AngZ(init_yaw))))




app = veh.ChWheeledVehicleIrrApp(vehicle.GetVehicle(),
                                 "FEDA on Rigid Terrain",
                                 irr.dimension2du(1280, 720))


app.SetTerrain(terrain)

app.AddTypicalLights()
app.SetChaseCamera(vehicle.GetChassis().GetFrame_REF_to_abs().GetPos(),
                   chase_dist, chase_height)
app.SetSkyBox()                      
app.AddLogo()                        


driver = veh.ChIrrGuiDriver(app)
driver.SetSteeringDelta(2.5 * chrono.CH_C_DEG_TO_RAD)   
driver.SetThrottleDelta(0.05)
driver.SetBrakingDelta(0.2)
driver.Initialize()




print("======================================================")
print("Controls: W/S throttle | A/D steering | SPACE brake")
print("Esc to quit.")
print("======================================================")


time = 0.0
render_step = max(1, math.floor(1.0 / (render_fps * step_size)))
step_number = 0

while app.GetDevice().run():
    time = system.GetChTime()
    if time >= simulation_end:
        break

    
    if step_number % render_step == 0:
        app.BeginScene(True, True, irr.SColor(255, 140, 161, 192))
        app.DrawAll()
        app.EndScene()

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    app.Synchronize("", driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    system.DoStepDynamics(step_size)

    step_number += 1




print("Simulation ended at t = {:.2f} s".format(time))