import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

def main():
    

    
    feda = veh.FEDA()
    feda.SetContactMethod(contact_method)
    feda.SetChassisFixed(False) 
    feda.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
    feda.SetEngineType(veh.EngineModelType_SIMPLE_MAP)
    feda.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    feda.SetTireType(tire_model)
    feda.Initialize()

    feda.SetChassisVisualizationType(chassis_vis_type)
    feda.SetSuspensionVisualizationType(suspension_vis_type)
    feda.SetSteeringVisualizationType(steering_vis_type)
    feda.SetWheelVisualizationType(wheel_vis_type)
    feda.SetTireVisualizationType(tire_vis_type)

    feda.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    
    terrain = veh.RigidTerrain(feda.GetSystem())
    if (contact_method == chrono.ChContactMethod_NSC):
        patch_mat = chrono.ChContactMaterialNSC()
        patch_mat.SetFriction(0.9)
        patch_mat.SetRestitution(0.01)
    elif (contact_method == chrono.ChContactMethod_SMC):
        patch_mat = chrono.ChContactMaterialSMC()
        patch_mat.SetFriction(0.9)
        patch_mat.SetRestitution(0.01)
        patch_mat.SetYoungModulus(2e7)
    patch = terrain.AddPatch(patch_mat, 
                             chrono.CSYSNORM, 
                             200, 200)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    terrain.Initialize()

    
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('FEDA')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(track_point, 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(feda.GetVehicle())

    
    driver = veh.ChInteractiveDriverIRR(vis)

    
    steering_time = 1.0  
    throttle_time = 1.0  
    braking_time = 0.3   
    driver.SetSteeringDelta(render_step_size / steering_time)
    driver.SetThrottleDelta(render_step_size / throttle_time)
    driver.SetBrakingDelta(render_step_size / braking_time)

    driver.Initialize()

    
    
    driver.SynchronizeFromVehicle()

    
    feda.GetVehicle().EnableRealtime(True)

    while vis.Run() :
        time = feda.GetSystem().GetChTime()

        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        
        driver_inputs = driver.GetInputs()

        
        driver.SynchronizeFromVehicle()
        terrain.SynchronizeFromVehicle()
        feda.SynchronizeFromVehicle(terrain, driver)
        vis.SynchronizeFromVehicle(driver)

        
        driver.Advance(step_size)
        terrain.Advance(step_size)
        feda.Advance(step_size)
        vis.Advance(step_size)

        
        assert(feda.GetInitPosition() == chrono.ChCoordsysd(initLoc, initRot))
        assert(feda.GetChassis().GetPos() == feda.GetChassisPos())
        assert(feda.GetChassis().GetRot() == feda.GetChassisRot())
        assert(feda.GetSuspension(suspension_link).GetLength() == feda.GetSuspensionLength(suspension_link))
        assert(feda.GetSteering(steering_link).GetAngle() == feda.GetSteeringAngle(steering_link))
        assert(feda.GetWheel(wheel_link).GetAngularVelocity() == feda.GetWheelAngularVelocity(wheel_link))
        assert(feda.GetTire(tire_link).GetRadius() == feda.GetTireRadius(tire_link))

    return 0







veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(0, 0, 1.6)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


contact_method = chrono.ChContactMethod_SMC


engine_model = veh.EngineModelType_SIMPLE_MAP


transmission_model = veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP


tire_model = veh.TireModelType_TMEASY


chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type =  veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_MESH
tire_vis_type = veh.VisualizationType_MESH


tire_tire_vis_type = veh.VisualizationType_MESH


chassis_mesh = veh.GetDataFile('feda/chassis.BW.o3o mesh')
suspension_mesh = veh.GetDataFile('feda/wishbone.BW.o3o mesh')
steering_mesh = veh.GetDataFile('feda/steering.AXLE.o3o mesh')
tire_mesh = veh.GetDataFile('feda/tire.RIGID.TMEASY.o3o mesh')
wheel_mesh = veh.GetDataFile('feda/hub.RIGID.o3o mesh')


tire_rim_vis_type = veh.VisualizationType_PRIMITIVES


tire_rim_mesh = veh.GetDataFile('feda/rim.o3o mesh')


tire_tire_vis_type = veh.VisualizationType_MESH


tire_tire_mesh = veh.GetDataFile('feda/tmeasy/tmeasy.o3o mesh')


tire sidewall_vis_type = veh.VisualizationType_MESH


tire sidewall_mesh = veh.GetDataFile('feda/tmeasy sidewall.o3o mesh')


tire_tread_vis_type = veh.VisualizationType_MESH


tire_tread_mesh = veh.GetDataFile('feda/tmeasy tread.o3o mesh')


tire_vis_type = veh.VisualizationType_PRIMITIVES


tire_noise_mesh = veh.GetDataFile('feda/tmeasy noise.o3o mesh')


tire_rim_vis_type = veh.VisualizationType_PRIMITIVES


tire_rim_mesh = veh.GetDataFile('feda/rim.o3o mesh')


track_point = chrono.ChVector3d(0, 0.08, 0.7)
patch_length = 15.0
patch_width = 10.0


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   


step_size = 2e-3


main()