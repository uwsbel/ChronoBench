import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math as m
import errno
import os
import math




def main() : 
    

    
    hmmwv = veh.HMMWV_Full()
    
    
    hmmwv.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(-75, 0, 0.6),chrono.ChQuaterniond(1, 0, 0, 0)))
    hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

    hmmwv.Initialize()

    
    engine = veh.ReadEngineJSON(chrono.GetChronoDataFile('vehicle/hmmwv/engines/hmmwv.json'))
    transmission = veh.ReadTransmissionJSON(chrono.GetChronoDataFile('vehicle/hmmwv/transmissions/hmmwv.json'))
    drive = veh.ReadDrivelineJSON(chrono.GetChronoDataFile('vehicle/hmmwv/drivelines/hmmwv.json'))

    hmmwv.SetVehicleEngine(engine)
    hmmwv.SetVehicleTransmission(transmission)
    hmmwv.SetVehicleDriveline(drive)

    hmmwv.GetVehicle().EnableRealtime(True)

    
    terrain = veh.RigidTerrain(hmmwv.GetSystem())
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    if (contact_method == veh.ContactMethod_NSC):
        patch_mat.SetYoungModulus(1e7)
    patch = terrain.AddPatch(patch_mat, 
                             chrono.CSYSNORM, 
                             200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 1.0))
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    terrain.Initialize()

    
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('HMMWV')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(hmmwv.GetVehicle())

    
    driver = veh.ChInteractiveDriverIRR(vis)

    
    steering_time = 1.0  
    throttle_time = 1.0  
    braking_time = 0.3   
    driver.SetSteeringDelta(10 * step_size / steering_time)
    driver.SetThrottleDelta(10 * step_size / throttle_time)
    driver.SetBrakingDelta(10 * step_size / braking_time)

    
    render_step_size = 1. / 50  

    
    left_rear_tire = veh.ReadTireJSON(chrono.GetChronoDataFile("vehicle/hmmwv/HMMWV_TMEASY_JSON.json"))
    right_rear_tire = veh.ReadTireJSON(chrono.GetChronoDataFile("vehicle/hmmwv/HMMWV_TMEASY_JSON.json"))
    left_front_tire = veh.ReadTireJSON(chrono.GetChronoDataFile("vehicle/hmmwv/HMMWV_TMEASY_JSON.json"))
    right_front_tire = veh.ReadTireJSON(chrono.GetChronoDataFile("vehicle/hmmwv/HMMWV_TMEASY_JSON.json"))

    hmmwv.GetVehicle().SetTire(0,  veh-left_rear_tire)
    hmmwv.GetVehicle().SetTire(1, veh-right_rear_tire)
    hmmwv.GetVehicle().SetTire(2, veh-left_front_tire)
    hmmwv.GetVehicle().SetTire(3, veh-right_front_tire)

    
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('HMMWV')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(hmmwv.GetVehicle())

    
    driver = veh.ChInteractiveDriverIRR(vis)

    
    
    steering_time = 1.0  
    throttle_time = 1.0  
    braking_time = 0.3   
    driver.SetSteeringDelta(render_step_size / steering_time)
    driver.SetThrottleDelta(render_step_size / throttle_time)
    driver.SetBrakingDelta(render_step_size / braking_time)
    driver.Initialize()

    
    
    
    
    
    step_size = 2e-3

    
    
    
    hmmwv.GetVehicle().EnableRealtime(True)

    while vis.Run() :
        time = hmmwv.GetSystem().GetChTime()

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



contact_method = veh.ContactMethod_NSC

main()