import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import os
import math






chrono.SetChronoDataPath(os.environ.get('CHRONO_DATA_DIR', '../../../chrono_data/'))
veh.SetDataPath(os.path.join(chrono.GetChronoDataPath(), 'vehicle/'))





time_step = 0.005  


time_end = 100     


init_loc = chrono.ChVectorD(0, 0.5, 0)  
init_rot = chrono.QUNIT                


chassis_vis_type = veh.VisualizationType_MESHES
wheel_vis_type = veh.VisualizationType_MESHES



chassis_collision_type = veh.CollisionType_PRIMITIVES


camera_track_point = chrono.ChVectorD(0.0, 0.0, 0.0)
camera_chase_dist = 6.0
camera_chase_height = 1.5





my_system = chrono.ChSystemNSC() 


my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


my_system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN) 
my_system.SetSolverMaxIterations(150)
my_system.SetMaxPenetrationRecoverySpeed(4.0)





ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(0.9)
ground_mat.SetRestitution(0.01)
ground_mat.SetYoungModulus(2e7) 


terrain = veh.RigidTerrain(my_system)



patch_size_x = 200.0
patch_size_y = 200.0
patch = terrain.AddPatch(ground_mat,
                         chrono.ChVectorD(0, 0, 0),    
                         chrono.ChVectorD(0, 1, 0),    
                         patch_size_x, patch_size_y)   


patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200) 


patch.GetGroundBody().GetVisualShape(0).SetColor(chrono.ChColor(0.4, 0.4, 0.6))


terrain.Initialize()





rover = veh.Curiosity_Rigid(my_system)


rover.SetChassisVisualizationType(chassis_vis_type)
rover.SetChassisCollide(True) 


rover.SetWheelVisualizationType(wheel_vis_type)


rover.Initialize(chrono.ChCoordsysD(init_loc, init_rot))






app = irr.ChIrrApp(my_system, "Curiosity Rover Simulation", irr.dimension2du(1280, 720), False, True, True)


app.SetChaseCamera(rover.GetChassisBody(), camera_track_point, camera_chase_dist, camera_chase_height)
app.SetCameraPosition(init_loc + chrono.ChVectorD(5,3,5)) 
app.SetCameraTarget(init_loc)


app.AddTypicalLights()








app.AddShadowAll()



sky_path = veh.GetDataFile("skybox/")
app.GetSceneManager().addSkyBoxSceneNode(
    irr.readImage(sky_path + "sky_up.jpg"),    
    irr.readImage(sky_path + "sky_dn.jpg"),    
    irr.readImage(sky_path + "sky_lf.jpg"),    
    irr.readImage(sky_path + "sky_rt.jpg"),    
    irr.readImage(sky_path + "sky_ft.jpg"),    
    irr.readImage(sky_path + "sky_bk.jpg")     
)


logo_path = veh.GetDataFile("chrono_logo.png")
if os.path.exists(logo_path):
    app.GetGUIEnvironment().addImage(
        app.GetVideoDriver().getTexture(logo_path),
        irr.position2d_s32(10, 10) 
    )
else:
    print(f"Warning: Logo file not found at {logo_path}")



app.AssetBindAll()
app.AssetUpdateAll()





driver = veh.ChInteractiveDriverIRR(app)


driver.SetSteeringDelta(0.04)  
driver.SetThrottleDelta(0.1)   
driver.SetBrakingDelta(0.2)    


driver.Initialize()





app.SetTimestep(time_step)
app.SetTryRealtime(True) 


current_time = 0
while app.GetDevice().run():
    current_time = my_system.GetChTime()
    if current_time >= time_end:
        break

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(current_time)      
    rover.Synchronize(current_time, driver_inputs, terrain) 
    terrain.Synchronize(current_time)     

    
    driver.Advance(time_step)
    rover.Advance(time_step)
    terrain.Advance(time_step)
    my_system.DoStepDynamics(time_step) 

    
    app.BeginScene(True, True, irr.SColor(255, 140, 160, 190)) 
    app.DrawAll()

    
    
    
    
    
    speed = rover.GetChassisBody().GetFrame_REF_to_abs().GetPos_dt().Length()
    app.GetVideoDriver().draw2DContext() 
    font = app.GetFont()
    if font:
         font.draw("Time: {:.2f} s".format(current_time), irr.rect_s32(10, 50, 200, 70), irr.SColor(255, 0, 0, 0))
         font.draw("Speed: {:.2f} m/s".format(speed), irr.rect_s32(10, 70, 200, 90), irr.SColor(255, 0, 0, 0))
         font.draw("Throttle: {:.2f}".format(driver_inputs.m_throttle), irr.rect_s32(10, 90, 200, 110), irr.SColor(255, 0, 0, 0))
         font.draw("Steering: {:.2f}".format(driver_inputs.m_steering), irr.rect_s32(10, 110, 200, 130), irr.SColor(255, 0, 0, 0))
         font.draw("Braking: {:.2f}".format(driver_inputs.m_braking), irr.rect_s32(10, 130, 200, 150), irr.SColor(255, 0, 0, 0))


    app.EndScene()





del app
print("Simulation finished.")