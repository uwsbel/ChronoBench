import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import os
import math




chrono.SetChronoDataPath(os.environ.get('CHRONO_DATA_DIR', '../../../chrono_data/'))
veh.SetDataPath(os.path.join(chrono.GetChronoDataPath(), 'vehicle', ''))




step_size = 0.005  
simulation_time = 60 


init_loc = chrono.ChVectorD(-70, 0, 0.5)
init_rot = chrono.ChQuaternionD(1, 0, 0, 0) 


engine_model = veh.EngineModelType_SHAFTS
drive_type = veh.DrivelineTypeWV_AWD
tire_model = veh.TireModelType_RIGID 


vis_type_chassis = veh.VisualizationType_MESH
vis_type_suspension = veh.VisualizationType_MESH 
vis_type_steering = veh.VisualizationType_MESH 
vis_type_wheel = veh.VisualizationType_MESH
vis_type_tire = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_PRIMITIVES


trackPoint = chrono.ChVectorD(0.0, 0.0, 1.75)





system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN) 
system.SetSolverMaxIterations(150)
system.SetMaxPenetrationRecoverySpeed(4.0)




hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChMaterialSurface.NSC)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysD(init_loc, init_rot))
hmmwv.SetEngineType(engine_model)
hmmwv.SetDrivelineType(drive_type)
hmmwv.SetTireType(tire_model)
hmmwv.SetChassisCollisionType(chassis_collision_type)


hmmwv.Initialize()


hmmwv.SetChassisVisualizationType(vis_type_chassis)
hmmwv.SetSuspensionVisualizationType(vis_type_suspension)
hmmwv.SetSteeringVisualizationType(vis_type_steering)
hmmwv.SetWheelVisualizationType(vis_type_wheel)
hmmwv.SetTireVisualizationType(vis_type_tire)


vehicle = hmmwv.GetVehicle()

vehicle.SetSystem(system)





terrain = veh.RigidTerrain(system)
terrain.SetContactFrictionCoefficient(0.9)
terrain.SetContactRestitutionCoefficient(0.01)
terrain.SetContactMaterialProperties(2e7, 0.3) 


patch_material = chrono.ChMaterialSurfaceNSC()
patch_material.SetFriction(0.8)
patch_material.SetRestitution(0.01)

texture_file_asphalt = chrono.GetChronoDataFile("vehicle/terrain/textures/tile4.jpg")
terrain.AddPatch(patch_material,
                 chrono.CSYSNORM, 
                 60.0, 20.0, 
                 1.0, 
                 True, 
                 texture_file_asphalt,
                 20, 20) 




bump_mesh_file = chrono.GetChronoDataFile("vehicle/terrain/meshes/bump.obj")
bump_texture_file = chrono.GetChronoDataFile("vehicle/terrain/textures/concrete.jpg")
bump_trimesh = chrono.ChTriangleMeshConnected()
bump_trimesh.LoadWavefrontMesh(bump_mesh_file, False, True)


bump_transform = chrono.ChTransformD()
bump_transform.SetPos(chrono.ChVectorD(40, 0, 0)) 
bump_transform.SetRot(chrono.Q_from_AngZ(0))      















mesh_body = chrono.ChBody()
mesh_body.SetPos(chrono.ChVectorD(40, 0, 0)) 
mesh_body.SetBodyFixed(True)
mesh_body.GetCollisionModel().ClearModel()
mesh_body.GetCollisionModel().AddTriangleMesh(patch_material, bump_trimesh, False, False, chrono.ChVectorD(0,0,0), chrono.ChMatrix33D(1), 0.05)
mesh_body.GetCollisionModel().BuildModel()
system.Add(mesh_body)


bump_shape = chrono.ChTriangleMeshShape()
bump_shape.SetMesh(bump_trimesh)
bump_shape.SetTexture(bump_texture_file, 10, 10) 
mesh_body.AddVisualShape(bump_shape)



heightmap_file = chrono.GetChronoDataFile("vehicle/terrain/height_maps/slope.bmp")
texture_file_grass = chrono.GetChronoDataFile("vehicle/terrain/textures/grass.jpg")





terrain.AddPatch(patch_material,
                 chrono.ChVectorD(90, 0, -0.5), 
                 heightmap_file, "heightmap_patch",
                 40.0, 30.0, 
                 0.0, 2.0, 
                 True, 
                 texture_file_grass,
                 10, 10) 


texture_file_concrete = chrono.GetChronoDataFile("vehicle/terrain/textures/concrete.jpg")
terrain.AddPatch(patch_material,
                 chrono.ChVectorD(140, 0, 0), 
                 60.0, 20.0,
                 1.0, 
                 True,
                 texture_file_concrete,
                 15, 15)


terrain.Initialize()




app = chronoirr.ChIrrApp(system, "HMMWV on Complex Rigid Terrain", chronoirr.dimension2du(1280, 720))
app.SetTimestep(step_size)
app.AddTypicalSky()
app.AddTypicalLights()


app.SetChaseCamera(trackPoint, 6.0, 0.5) 
app.SetChaseCameraPosition(vehicle.GetPos() + chrono.ChVectorD(0,0,20)) 


app.AssetBindAll()
app.AssetUpdateAll()




driver = veh.ChIrrGuiDriver(app)




steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   

driver.SetSteeringDelta(step_size / steering_time)
driver.SetThrottleDelta(step_size / throttle_time)
driver.SetBrakingDelta(step_size / braking_time)

driver.Initialize()




realtime_timer = chrono.ChRealtimeStepTimer()
time = 0

while app.GetDevice().run():
    time = system.GetChTime()
    if time > simulation_time:
        break

    app.BeginScene(True, True, chronoirr.SColor(255, 140, 160, 190))
    app.DrawAll()
    
    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time) 
    app.Synchronize(driver.GetInputModeAsString(), driver_inputs) 

    
    driver.Advance(step_size)
    vehicle.Advance(step_size)
    terrain.Advance(step_size) 
    app.Advance(step_size) 
    
    
    system.DoStepDynamics(step_size)

    app.EndScene()
    realtime_timer.Spin(step_size)

print("Simulation finished.")