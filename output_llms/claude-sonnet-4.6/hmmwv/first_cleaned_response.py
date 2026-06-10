import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr




step_size = 1e-3          
tire_step_size = 1e-3     
render_fps = 50           
render_step_size = 1.0 / render_fps




initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_PRIMITIVES


chassis_collision_type = veh.CollisionType_NONE


contact_method = chrono.ChContactMethod_NSC





hmmwv = veh.HMMWV_Full()

hmmwv.SetContactMethod(contact_method)
hmmwv.SetChassisCollisionType(chassis_collision_type)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
hmmwv.SetEngineType(veh.EngineModelType_SIMPLE)
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(tire_step_size)

hmmwv.Initialize()


hmmwv.SetChassisVisualizationType(vis_type)
hmmwv.SetSuspensionVisualizationType(vis_type)
hmmwv.SetSteeringVisualizationType(vis_type)
hmmwv.SetWheelVisualizationType(vis_type)
hmmwv.SetTireVisualizationType(vis_type)




terrain = veh.RigidTerrain(hmmwv.GetSystem())


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.CSYSNORM.rot),
    200.0,   
    200.0    
)

patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

terrain.Initialize()




vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV Full Vehicle Demo")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.75), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(hmmwv.GetVehicle())




driver = veh.ChInteractiveDriverIRR(vis)


driver.SetSteeringDelta(render_step_size / 1.0)   
driver.SetThrottleDelta(render_step_size / 1.0)   
driver.SetBrakingDelta(render_step_size / 0.3)    

driver.Initialize()


print("HMMWV Full Vehicle Simulation")
print(f"  Contact method     : NSC")
print(f"  Tire model         : TMEASY")
print(f"  Visualization      : Primitives")
print(f"  Simulation step    : {step_size}")
print(f"  Render FPS         : {render_fps}")
print("-------------------------------")
print("Controls:")
print("  W/S       : throttle / brake")
print("  A/D       : steer left / right")
print("  ESC       : quit")
print("-------------------------------")




realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_steps = int(render_step_size / step_size)

while vis.Run():
    time = hmmwv.GetSystem().GetChTime()

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    hmmwv.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    hmmwv.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1

    
    realtime_timer.Spin(step_size)