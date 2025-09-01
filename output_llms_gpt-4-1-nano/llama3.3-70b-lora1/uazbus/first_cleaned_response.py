import pychrono as chrono
import pychrono.vehicle as veh
import errno
import os



def main() : 
    

    
    vehicle = veh.UAZBUS()
    vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
    vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
    vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(-10, -2, 0.6), chrono.ChQuaterniond(1, 0, 0, 0)))
    vehicle.SetEngineType(veh.EngineModelType_SHAFTS);
    vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS);
    vehicle.SetDriveType(veh.DrivelineTypeWV_RWD)
    vehicle.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    vehicle.SetTireType(veh.TireModelType_TMEASY)
    vehicle.SetTireStepSize(tire_step_size)
    vehicle.Initialize()

    
    terrain = veh.RigidTerrain(vehicle.GetSystem())
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    if (contact_method == chrono.ChContactMethod_NSC):
        patch_mat.SetYoungModulus(2e7)
    patch = terrain.AddPatch(patch_mat, 
                             chrono.CSYSNORM, 
                             200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 1.0))
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    terrain.Initialize()

    
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('UAZ bus')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(vehicle)

    
    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()

    
    vehicle.EnableRealtime(True)

    while vis.Run() :
        time = vehicle.GetSystem().GetChTime()

        
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








veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


step_size = 3e-3;


tire_step_size = 1e-3;


t_end = 1000;


contact_method = chrono.ChContactMethod_SMC

main()