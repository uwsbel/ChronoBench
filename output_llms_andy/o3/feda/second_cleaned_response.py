import math
import pychrono.core   as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh




chrono.SetChronoDataPath(chrono.GetChronoDataPath())          
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')      




step_size        = 1e-3                      
render_fps       = 50                        
render_step_size = 1.0 / render_fps
tire_step_size   = step_size




initLoc = chrono.ChVector3d(-50, 0.0, 0.5)   
initRot = chrono.ChQuaterniond(1, 0, 0, 0)   




terrainHeight = 0.0
terrainLength = 200.0        
terrainWidth  = 100.0




vis_type               = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE
tire_model             = veh.TireModelType_TMEASY
contact_method         = chrono.ChContactMethod_NSC

vehicle = veh.FEDA()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()


vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)


vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)




patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrainHeight), chrono.QUNIT),
    terrainLength,
    terrainWidth,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 400, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()












path_pts = [
    chrono.ChVector3d(-50,  0.0, 0.0),
    chrono.ChVector3d(-40,  0.0, 0.0),
    chrono.ChVector3d(-30,  0.0, 0.0),

    chrono.ChVector3d(-20,  3.5, 0.0),     
    chrono.ChVector3d(-10,  3.5, 0.0),

    chrono.ChVector3d(   0, -3.5, 0.0),    
    chrono.ChVector3d(  10, -3.5, 0.0),

    chrono.ChVector3d(  20,  0.0, 0.0),    
    chrono.ChVector3d(  30,  0.0, 0.0),
    chrono.ChVector3d(  50,  0.0, 0.0),
]

bezier_path = chrono.ChBezierCurve()
for p in path_pts:
    bezier_path.AddPoint(p)
bezier_path.SetClosed(False)   




target_speed = 10.0    

driver = veh.ChPathFollowerDriver(vehicle.GetVehicle(),
                                  bezier_path,
                                  "ISO_double_lane_change",
                                  target_speed)


driver.GetSteeringController().SetLookAheadDistance(5.0)         
driver.GetSteeringController().SetGains(0.8, 0.0, 0.3)           


driver.GetSpeedController().SetGains(0.4, 0.0, 0.0)              

driver.Initialize()




vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('FEDA – ISO Double Lane-Change')
vis.SetWindowSize(1280, 1024)

vis.SetChaseCamera(chrono.ChVector3d(-3, 0, 1.1), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

print("VEHICLE MASS:", vehicle.GetVehicle().GetMass())




render_steps   = math.ceil(render_step_size / step_size)
step_number    = 0
realtime_timer = chrono.ChRealtimeStepTimer()

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver.GetInputs(), terrain)
    vis.Synchronize(time, driver.GetInputs())

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)