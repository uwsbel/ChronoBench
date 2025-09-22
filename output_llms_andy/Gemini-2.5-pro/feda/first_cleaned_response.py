import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import os




chrono.SetChronoDataPath(os.environ.get('CHRONO_DATA_DIR', '../../../../chrono_data/'))
veh.SetDataPath(os.path.join(chrono.GetChronoDataPath(), 'vehicle/'))





step_size = 1.0 / 50.0  


initLoc = chrono.ChVectorD(0, 0.7, 0)  
initRot = chrono.ChQuaternionD(1, 0, 0, 0)  


contact_method = chrono.ChContactMethod_NSC 











terrain_height = 0  
terrain_size_x = 200 
terrain_size_z = 200 
terrain_texture_file = veh.GetDataFile("terrain/textures/tile4.jpg") 
terrain_texture_scale_x = 200 
terrain_texture_scale_z = 200


camera_chase_track_point = chrono.ChVectorD(0.0, 0.0, 0.0) 
camera_chase_distance = 8.0 
camera_chase_height = 1.5   




print("Creating Chrono system...")
if contact_method == chrono.ChContactMethod_NSC:
    sys = chrono.ChSystemNSC()
    print("Using NSC contact method.")
elif contact_method == chrono.ChContactMethod_SMC:
    sys = chrono.ChSystemSMC()
    print("Using SMC contact method.")
else:
    raise ValueError("Invalid contact method specified.")

sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN) 
sys.SetSolverMaxIterations(150)
sys.SetMaxPenetrationRecoverySpeed(4.0)




print("Creating FEDA vehicle...")
feda = veh.FEDA()
feda.SetContactMethod(contact_method)
feda.SetChassisFixed(False) 
feda.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))

feda.SetTireStepSize(step_size) 
feda.Initialize()


print("Setting MESH visualization for vehicle parts...")
feda.SetChassisVisualizationType(veh.VisualizationType_MESH)
feda.SetSuspensionVisualizationType(veh.VisualizationType_MESH) 
feda.SetSteeringVisualizationType(veh.VisualizationType_MESH)   
feda.SetWheelVisualizationType(veh.VisualizationType_MESH)
feda.SetTireVisualizationType(veh.VisualizationType_MESH) 

vehicle = feda.GetVehicle()
chassis_body = feda.GetChassisBody()




print(f"Creating rigid terrain with texture: {terrain_texture_file}")
terrain = veh.RigidTerrain(sys)


if contact_method == chrono.ChContactMethod_NSC:
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
else: 
    patch_mat = chrono.ChMaterialSurfaceSMC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch_mat.SetYoungModulus(2e7) 



patch = terrain.AddPatch(patch_mat,
                         chrono.CSYSNORM, 
                         terrain_size_x, terrain_size_z, terrain_height,
                         False, 
                         terrain_texture_file, "terrain_texture",
                         0.02 
                         )

patch.SetTexture(terrain_texture_file, terrain_texture_scale_x, terrain_texture_scale_z)
patch.GetGroundBody().GetVisualShape(0).SetTexture(terrain_texture_file, terrain_texture_scale_x, terrain_texture_scale_z)
terrain.Initialize()




print("Creating Irrlicht application...")

app = veh.ChWheeledVehicleIrrApp(vehicle, "FEDA on Rigid Terrain", irr.dimension2du(1280, 720))




app.SetChaseCamera(camera_chase_track_point, camera_chase_distance, camera_chase_height)



app.SetTimestep(step_size)
app.AddTypicalLights() 
app.SetSkyBox() 
app.AssetBindAll()
app.AssetUpdateAll()





print("Creating interactive driver...")
driver = veh.ChInteractiveDriverIRR(app)


driver.SetSteeringDelta(0.04)  
driver.SetThrottleDelta(0.04)  
driver.SetBrakingDelta(0.1)   
driver.Initialize()

print("\nControls:")
print("Steering: A/D")
print("Throttle: W")
print("Braking: S")
print("Camera Zoom: Z/X")
print("Camera Orbit: LMB drag")
print("------------------------------------------------------------------------------")




time = 0.0
while app.GetDevice().run():
    current_time = sys.GetChTime()

    
    app.BeginScene(True, True, irr.SColor(255, 140, 160, 190)) 
    app.DrawAll() 

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(current_time)
    vehicle.Synchronize(current_time, driver_inputs, terrain) 
    terrain.Synchronize(current_time) 
    app.Synchronize(driver.GetInputModeAsString(), driver_inputs) 

    
    driver.Advance(step_size)
    vehicle.Advance(step_size)
    terrain.Advance(step_size) 
    app.Advance(step_size)

    
    sys.DoStepDynamics(step_size)

    app.EndScene()

    
    if chassis_body.GetPos().y < -1 : 
        print(f"Vehicle Y position {chassis_body.GetPos().y} too low. Ending simulation.")
        break
    if abs(chassis_body.GetRot().Q_to_Euler123().x) > 1.5 or abs(chassis_body.GetRot().Q_to_Euler123().z) > 1.5 :
        
        print(f"Vehicle excessively tilted. Ending simulation.")
        

    
    time += step_size

del app 
print("Simulation finished.")