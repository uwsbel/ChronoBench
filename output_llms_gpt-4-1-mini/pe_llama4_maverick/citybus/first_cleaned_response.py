import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr




initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_TMEASY


terrain_height = 0
terrain_length = 100.0  
terrain_width = 100.0   


step_size = 2e-3


render_step_size = 1.0 / 50



def main():
    
    car = veh.CityBus()
    car.SetContactMethod(chrono.ChContactMethod_NSC)
    car.SetChassisCollisionType(chassis_collision_type)
    car.SetChassisVisualizationType(chassis_vis_type)
    car.SetSuspensionVisualizationType(suspension_vis_type)
    car.SetSteeringVisualizationType(steering_vis_type)
    car.SetWheelVisualizationType(wheel_vis_type)
    car.SetTireVisualizationType(veh.VisualizationType_MESH)
    car.SetTireType(tire_model)
    car.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
    car.Initialize()

    
    terrain = veh.RigidTerrain(car.GetSystem())
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrain_height), chrono.QUNIT), terrain_length, terrain_width)
    patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('CityBus Demo')
    vis.SetWindowSize(1280, 720)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(car.GetVehicle())

    
    driver = veh.ChIrrGuiDriver(vis)
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()

    
    realtime_timer = chrono.ChRealtimeStepTimer()
    while vis.Run():
        time = car.GetSystem().GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        
        driver_inputs = driver.GetInputs()

        
        driver.Synchronize(time)
        terrain.Synchronize(time)
        car.Synchronize(time, driver_inputs, terrain.GetHeight(car.GetPos()))
        vis.Synchronize("", driver_inputs)

        
        driver.Advance(step_size)
        terrain.Advance(step_size)
        car.Advance(step_size)
        vis.Advance(step_size)

        
        realtime_timer.Spin(step_size)

    return 0

if __name__ == "__main__":
    main()