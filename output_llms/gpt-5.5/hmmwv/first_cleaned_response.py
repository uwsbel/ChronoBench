import math
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr





try:
    ChVector = chrono.ChVector3d
except AttributeError:
    ChVector = chrono.ChVectorD

try:
    ChCoordsys = chrono.ChCoordsysd
except AttributeError:
    ChCoordsys = chrono.ChCoordsysD

try:
    ContactMaterialNSC = chrono.ChContactMaterialNSC
except AttributeError:
    ContactMaterialNSC = chrono.ChMaterialSurfaceNSC


def main():
    
    
    
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

    
    
    
    contact_method = chrono.ChContactMethod_NSC

    step_size = 1.0e-3
    tire_step_size = 1.0e-3

    render_fps = 50
    render_step_size = 1.0 / render_fps
    render_steps = int(math.ceil(render_step_size / step_size))

    
    init_loc = ChVector(0.0, 0.0, 0.7)
    init_yaw = 0.0
    init_rot = chrono.QuatFromAngleZ(init_yaw)

    
    terrain_length = 200.0
    terrain_width = 200.0
    terrain_friction = 0.9
    terrain_restitution = 0.01

    
    steering_time = 1.0
    throttle_time = 1.0
    braking_time = 0.3

    
    
    
    hmmwv = veh.HMMWV_Full()

    hmmwv.SetContactMethod(contact_method)
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)

    hmmwv.SetInitPosition(ChCoordsys(init_loc, init_rot))

    
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)

    
    hmmwv.SetTireType(veh.TireModelType_TMEASY)
    hmmwv.SetTireStepSize(tire_step_size)

    hmmwv.Initialize()

    
    hmmwv.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)

    
    
    
    terrain = veh.RigidTerrain(hmmwv.GetSystem())

    terrain_mat = ContactMaterialNSC()
    terrain_mat.SetFriction(terrain_friction)
    terrain_mat.SetRestitution(terrain_restitution)

    terrain_pose = ChCoordsys(ChVector(0.0, 0.0, 0.0), chrono.QuatFromAngleZ(0.0))

    patch = terrain.AddPatch(
        terrain_mat,
        terrain_pose,
        terrain_length,
        terrain_width
    )

    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)

    terrain.Initialize()

    
    
    
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("PyChrono Full HMMWV - Rigid Terrain")
    vis.SetWindowSize(1280, 720)

    
    vis.SetChaseCamera(ChVector(0.0, 0.0, 1.75), 6.0, 0.5)

    vis.AttachVehicle(hmmwv.GetVehicle())
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddLightDirectional()

    
    
    
    driver = veh.ChInteractiveDriverIRR(vis)

    driver.SetSteeringDelta(render_step_size / steering_time)
    driver.SetThrottleDelta(render_step_size / throttle_time)
    driver.SetBrakingDelta(render_step_size / braking_time)

    driver.Initialize()

    
    
    
    realtime_timer = chrono.ChRealtimeStepTimer()
    step_number = 0

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

        
        realtime_timer.Spin(step_size)

        step_number += 1


if __name__ == "__main__":
    main()