import os
import math
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens






step_size            = 1e-3          
render_step           = 0.03         
end_time              = 10.0         
contact_method        = chrono.ChContactMethod_NSC
data_path             = chrono.GetChronoDataPath()
vehicle_data_path     = veh.GetDataPath()


chrono.ChRandomGenerator.SetSeed(12345)





system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))






init_loc  = chrono.ChVectorD(0, 0, 0.4)
init_rot  = chrono.ChQuaternionD(1, 0, 0, 0)          

gator = veh.Gator(system)
gator.SetContactMethod(contact_method)
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChCoordsysD(init_loc, init_rot))


gator.SetChassisVisualizationType(veh.VisualizationType.MESH)
gator.SetWheelVisualizationType  (veh.VisualizationType.PRIMITIVES)
gator.SetTireVisualizationType   (veh.VisualizationType.SPRINGS)   
gator.SetSuspensionVisualizationType(veh.VisualizationType.PRIMITIVES)
gator.Initialize()





terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_size = 400
terrain_patch = terrain.AddPatch(patch_mat,
                                 chrono.ChCoordsysD(chrono.ChVectorD(0,0,0), chrono.QUNIT),
                                 patch_size, patch_size)
terrain_patch.SetColor(chrono.ChColor(0.4, 0.5, 0.4))
terrain_patch.SetTexture(data_path + "terrain/textures/grass.jpg", 10, 10)
terrain.Initialize()





app = veh.ChVehicleIrrApp(gator, "Gator with Sensors", chrono.dimension(1280, 720))
app.AddTypicalLogo()
app.AddTypicalSky()
app.AddTypicalCamera(chrono.vectorD(0.0, 2.0, 1.2), chrono.vectorD(0, 0, 0.4))
app.AddLightWithShadow(chrono.vectorD(5,5,5), chrono.vectorD(0,0,0), 15, 4, 10, 60)
app.AddLightDirectional()


chrono.ChAssetLevel().BindAll(system)
chrono.ChAssetLevel().UpdateAll(system)

driver = veh.ChIrrGuiDriver(app)
driver.SetSteeringDelta(0.03)
driver.SetThrottleDelta(0.05)
driver.SetBrakingDelta(0.2)
driver.Initialize()






sensor_manager = sens.ChSensorManager(system)
sensor_manager.SetKeyframeSizeFromTime(0.1)           


pl_1 = sens.ChPointLight(chrono.ChVectorF(10, 10, 10),      
                         chrono.ChColor(1.0, 1.0, 1.0),     
                         400.0)                             
sensor_manager.AddSensor(pl_1)

pl_2 = sens.ChPointLight(chrono.ChVectorF(-10, -10, 10),
                         chrono.ChColor(1.0, 0.8, 0.8),
                         200.0)
sensor_manager.AddSensor(pl_2)


cam_update_rate  = 30.0                                     
cam_resolution   = sens.ChVector2i(1280, 720)
cam_fov          = math.radians(70)


cam_offset_pose  = chrono.ChFrameD(chrono.ChVectorD(0.5, 0.0, 1.2),
                                   chrono.Q_from_AngAxis(-math.pi/6, chrono.VECT_Y))

camera = sens.ChCameraSensor(gator.GetChassis(),             
                             cam_update_rate,                
                             cam_offset_pose,                
                             cam_resolution,                 
                             cam_fov)                        


camera.PushFilter(sens.ChFilterRGBA8Access())                
camera.PushFilter(sens.ChFilterSave("output/cam/"))          

sensor_manager.AddSensor(camera)





realtime_timer      = chrono.ChRealtimeStepTimer()
realtime_timer.SetDesiredRealtimeRate(1.0)

time_last_render    = 0.0
step_number         = 0

print("Simulation start -----------------------------------------------")
while (system.GetChTime() < end_time):

    
    if system.GetChTime() - time_last_render >= render_step:
        app.BeginScene()
        app.DrawAll()
        app.EndScene()
        time_last_render = system.GetChTime()

    
    driver_inputs = driver.GetInputs()
    gator.GetVehicle().Synchronize(system.GetChTime(),
                                   driver_inputs,
                                   terrain)

    
    terrain.Synchronize(system.GetChTime())

    
    driver.Advance(step_size)
    gator.GetVehicle().Advance(step_size)
    terrain.Advance(step_size)

    
    sensor_manager.Update()

    
    system.DoStepDynamics(step_size)

    
    realtime_timer.Spin(step_size)

    step_number += 1

print("Simulation finished.")