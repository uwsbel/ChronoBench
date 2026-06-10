import math
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


def main():
    
    
    

    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

    contact_method = chrono.ChContactMethod_NSC

    step_size = 2.0e-3
    tire_step_size = 1.0e-3
    render_step_size = 1.0 / 50.0

    
    init_loc = chrono.ChVector3d(0.0, 0.0, 1.2)
    init_rot = chrono.ChQuaterniond(1.0, 0.0, 0.0, 0.0)
    init_fwd_vel = 0.0

    
    
    

    vehicle = veh.Kraz()

    vehicle.SetContactMethod(contact_method)
    vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
    vehicle.SetChassisFixed(False)

    vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
    vehicle.SetInitFwdVel(init_fwd_vel)

    vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
    vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    vehicle.SetDriveType(veh.DrivelineTypeWV_AWD)

    
    vehicle.SetTireType(veh.TireModelType_TMEASY)
    vehicle.SetTireStepSize(tire_step_size)

    vehicle.Initialize()

    
    vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

    system = vehicle.GetSystem()

    
    
    

    terrain = veh.RigidTerrain(system)

    terrain_mat = chrono.ChContactMaterialNSC()
    terrain_mat.SetFriction(0.9)
    terrain_mat.SetRestitution(0.01)

    terrain_length = 300.0
    terrain_width = 300.0

    patch = terrain.AddPatch(
        terrain_mat,
        chrono.ChCoordsysd(
            chrono.ChVector3d(0.0, 0.0, 0.0),
            chrono.ChQuaterniond(1.0, 0.0, 0.0, 0.0),
        ),
        terrain_length,
        terrain_width,
    )

    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.45, 0.45, 0.45))

    terrain.Initialize()

    
    
    

    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("PyChrono Kraz Vehicle Simulation")
    vis.SetWindowSize(1280, 720)

    
    vis.SetChaseCamera(
        chrono.ChVector3d(0.0, 0.0, 2.0),
        12.0,
        3.0,
    )

    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddLightDirectional()
    vis.AttachVehicle(vehicle.GetVehicle())

    
    
    

    driver = veh.ChInteractiveDriverIRR(vis)

    steering_time = 1.0
    throttle_time = 1.0
    braking_time = 0.3

    driver.SetSteeringDelta(render_step_size / steering_time)
    driver.SetThrottleDelta(render_step_size / throttle_time)
    driver.SetBrakingDelta(render_step_size / braking_time)

    driver.Initialize()

    
    
    

    realtime_timer = chrono.ChRealtimeStepTimer()

    render_steps = math.ceil(render_step_size / step_size)
    step_number = 0

    while vis.Run():
        time = system.GetChTime()

        
        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        
        driver_inputs = driver.GetInputs()

        
        driver.Synchronize(time)
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        
        driver.Advance(step_size)
        terrain.Advance(step_size)
        vehicle.Advance(step_size)
        vis.Advance(step_size)

        
        realtime_timer.Spin(step_size)

        step_number += 1


if __name__ == "__main__":
    main()