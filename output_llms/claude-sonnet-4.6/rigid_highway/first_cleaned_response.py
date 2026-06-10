import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math
import os




initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_TMEASY


contact_method = chrono.ChContactMethod_SMC


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50  




hmmwv = veh.HMMWV_Full()

hmmwv.SetContactMethod(contact_method)
hmmwv.SetChassisCollisionType(chassis_collision_type)

hmmwv.SetChassisFixed(False)

hmmwv.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
hmmwv.SetTireType(tire_model)
hmmwv.SetTireStepSize(tire_step_size)

hmmwv.Initialize()


hmmwv.SetChassisVisualizationType(vis_type)
hmmwv.SetSuspensionVisualizationType(vis_type)
hmmwv.SetSteeringVisualizationType(vis_type)
hmmwv.SetWheelVisualizationType(vis_type)
hmmwv.SetTireVisualizationType(vis_type)




terrain = veh.RigidTerrain(hmmwv.GetSystem())


if contact_method == chrono.ChContactMethod_NSC:
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
elif contact_method == chrono.ChContactMethod_SMC:
    patch_mat = chrono.ChContactMaterialSMC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch_mat.SetYoungModulus(2e7)


col_mesh_file = chrono.GetChronoDataFile("vehicle/terrain/meshes/Highway_col.obj")
vis_mesh_file = chrono.GetChronoDataFile("vehicle/terrain/meshes/Highway_vis.obj")


use_mesh_terrain = os.path.exists(col_mesh_file)

if use_mesh_terrain:
    print("Using mesh terrain: Highway_col.obj / Highway_vis.obj")
    
    
    patch = terrain.AddPatch(
        patch_mat,
        chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
        col_mesh_file,
        False,   
        0.01,    
        False    
    )
    
    
    patch.SetMeshVisualization(vis_mesh_file)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    
else:
    print("Mesh files not found. Using flat terrain patch as fallback.")
    patch = terrain.AddPatch(
        patch_mat,
        chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
        200.0, 200.0
    )
    patch.SetTexture(chrono.GetChronoDataFile("vehicle/terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

terrain.Initialize()




vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV Full - Mesh Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.75), 6.0, 0.5)
vis.SetHUDLocation(500, 20)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(hmmwv.GetVehicle())




driver = veh.ChInteractiveDriverIRR(vis)


steering_time = 1.0   
throttle_time = 1.0   
braking_time = 0.3    

driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()




out_dir = chrono.GetChronoOutputPath() + "HMMWV_MESH"
if not os.path.exists(out_dir):
    try:
        os.makedirs(out_dir)
    except:
        pass




print("HMMWV simulation on mesh terrain")
print(f"Contact method: {'SMC' if contact_method == chrono.ChContactMethod_SMC else 'NSC'}")
print(f"Tire model: TMEASY")
print(f"Step size: {step_size}")
print(f"Render FPS: {1.0/render_step_size}")
print()
print("Controls:")
print("  Arrow keys: Steering")
print("  W/S: Throttle/Brake")
print("  Space: Brake")
print("  Q or Esc: Quit")
print()


render_steps = math.ceil(render_step_size / step_size)


step_number = 0
render_frame = 0


realtime_timer = chrono.ChRealtimeStepTimer()

while vis.Run():
    time = hmmwv.GetSystem().GetChTime()

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
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

print("Simulation completed.")