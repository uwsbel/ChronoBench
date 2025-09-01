import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math as m
import errno
import os
import math




def main() : 
    

    
    vehicle = veh.BMW_E90_Sedan()
    vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
    vehicle.SetChassisFixed(False);
    vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(-10, -2, 0.6),chrono.QUNIT))
    vehicle.SetEngineType(veh.EngineModelType_SIMPLE)
    vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    vehicle.SetDriveType(veh.DrivelineTypeWV_RWD)
    vehicle.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    vehicle.SetTireType(veh.TireModelType_TMEASY)
    vehicle.SetTireStepSize(tire_step_size)
    vehicle.Initialize()

    vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    vehicle.EnableRealtime(True)

    
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('BMW E90 Sedan')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(vehicle.GetVehicle())

    
    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta(0.06)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()

    vehicle.GetVehicle().SetDriver(driver.GetDriver())

    
    terrain = veh.RigidTerrain(vehicle.GetSystem())

    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)

    terrain.AddPatch(patch_mat, 
                     chrono.ChCoordsysd(chrono.ChVector3d(-10, 0, 0.4), chrono.QUNIT), 
                     300, 60)
    terrain.Initialize()

    
    vis.AddTerrain(terrain);

    
    vehicle.Initialize()

    terrain.Initialize()

    
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

main()