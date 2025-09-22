import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import os




chrono.SetChronoDataPath(os.environ.get('CHRONO_DATA_DIR', '../../chrono_data/'))
veh.SetDataPath(os.path.join(chrono.GetChronoDataPath(), 'vehicle/'))





step_size = 1.0 / 50.0  


initLoc = chrono.ChVectorD(0, 0, 0.5)  
initRot = chrono.ChQuaternionD(1, 0, 0, 0)  


contact_method = chrono.ChContactMethod_NSC  


tire_model = veh.TireModelType_TMEASY


vis_type_chassis = veh.VisualizationType_MESH
vis_type_suspension = veh.VisualizationType_MESH
vis_type_steering = veh.VisualizationType_MESH
vis_type_wheel = veh.VisualizationType_MESH
vis_type_tire = veh.VisualizationType_MESH 


terrain_length = 200.0  
terrain_width = 200.0   
terrain_height = 0.1    
terrain_friction = 0.8



terrain_texture_file = chrono.GetChronoDataFile("vehicle/terrain/textures/rock.jpg")
terrain_texture_scale_x = 20  
terrain_texture_scale_y = 20  




system = chrono.ChSystemNSC() if contact_method == chrono.ChContactMethod_NSC else chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN) 
system.SetSolverMaxIterations(150)
system.SetMaxPenetrationRecoverySpeed(4.0)




gator = veh.Gator(system)

gator.SetContactMethod(contact_method)
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
gator.SetPowertrainType(veh.PowertrainModelType_SHAFTS)  
gator.SetDriveType(veh.DrivelineTypeWV_AWD)             
gator.SetTireType(tire_model)
gator.SetTireStepSize(step_size) 
gator.Initialize()


gator.SetChassisVisualizationType(vis_type_chassis)
gator.SetSuspensionVisualizationType(vis_type_suspension, 0, veh.VisualizationType_PRIMITIVES) 
gator.SetSuspensionVisualizationType(vis_type_suspension, 1, veh.VisualizationType_PRIMITIVES) 
gator.SetSteeringVisualizationType(vis_type_steering, 0, veh.VisualizationType_PRIMITIVES) 
gator.SetWheelVisualizationType(vis_type_wheel)
gator.SetTireVisualizationType(vis_type_tire)

print("Gator vehicle initialized.")
print("Location:", gator.GetVehicle().GetChassisBody().GetPos())
print("Orientation (Quat):", gator.GetVehicle().GetChassisBody().GetRot())
print("Tire model:", gator.GetTireType())




terrain = veh.RigidTerrain(system)
material = chrono.ChMaterialSurfaceNSC() if contact_method == chrono.ChContactMethod_NSC else chrono.ChMaterialSurfaceSMC()
material.SetFriction(terrain_friction)
material.SetRestitution(0.01)









patch = terrain.AddPatch(material,
                         chrono.CSYSNORM,  
                         terrain_length,
                         terrain_width)

patch.SetTexture(terrain_texture_file, terrain_texture_scale_x, terrain_texture_scale_y)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5)) 

terrain.Initialize()
print("Rigid terrain initialized.")




app_title = "Gator on Rigid Terrain - TMEASY Tires"
app_width = 1280
app_height = 720

application = irr.ChIrrApp(system, app_title, irr.dimension2du(app_width, app_height))
application.SetTimestep(step_size) 


application.AddTypicalLights()




trackPoint = chrono.ChVectorD(0.0, 0.0, 1.75) 
camera_dist = 8.0 
camera_height = 1.5 
application.SetChaseCamera(gator.GetVehicle().GetChassisBody(), camera_dist, 0.5)
application.SetChaseCameraPosition(gator.GetVehicle().GetChassisBody().TransformPointLocalToParent(trackPoint) + chrono.ChVectorD(0, camera_dist * 0.707 , camera_height))
application.SetChaseCameraLookAt(gator.GetVehicle().GetChassisBody().TransformPointLocalToParent(trackPoint))



application.AssetBindAll()
application.AssetUpdateAll()




driver = veh.ChIrrGuiDriver(application)



steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   

driver.SetSteeringDelta(step_size / steering_time)
driver.SetThrottleDelta(step_size / throttle_time)
driver.SetBrakingDelta(step_size / braking_time)

driver.Initialize()
print("Interactive driver initialized. Use A/D for steering, W/S for throttle/brake.")








time = 0

while application.GetDevice().run():
    time = system.GetChTime()

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    gator.Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time) 
    application.Synchronize(driver.GetInputModeAsString(), driver_inputs) 

    
    driver.Advance(step_size)
    gator.Advance(step_size)
    terrain.Advance(step_size)
    application.AdvanceModel() 

    
    application.BeginScene(True, True, chrono.ChColor(0.2, 0.2, 0.3)) 
    application.DrawAll()
    
    
    
    driver.DrawGUI() 

    application.EndScene()

    
    

del application 
print("Simulation finished.")