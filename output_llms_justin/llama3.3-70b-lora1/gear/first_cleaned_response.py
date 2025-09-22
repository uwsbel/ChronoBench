import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math







change_camera_height = True


def main():
    

    
    vehicle = veh.RigidChassisVehicle(chassis_file, None)

    
    for ia in vehicle.GetTireAssemblies():
        tire = ia.GetTire()
        
        vis = veh.VisualizationType_NONE
        if tire.HasVisualizationMesh():
            vis = veh.VisualizationType_MESH

        
        tire.SetPressure(35000)
        
        tire.SetVisualizationType(vis)
        
        vehicle.InitializeTire(ia.GetId(), veh.PointContactMethod_HARD)

    
    engine = veh.ReadEngineJSON(engine_file)
    transmission = veh.ReadTransmissionJSON(transmission_file)
    powertrain = veh.ChPowertrainAssembly(engine, transmission)
    vehicle.InitializePowertrain(powertrain, veh.ThrottleControlType_NONE)

    vehicle.GetVehicle().SetChassisCollisionType(veh.CollisionType_NONE);
    vehicle.GetVehicle().SetChassisFixed(False);
    vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET);

    vehicle.Initialize()

    
    vehicle.GetChassisBody().Translate(chrono.ChVector3d(-75, 0, 0.5))
    vehicle.GetChassisBody().Rotate(chrono.ChQuaternion1d(1, 0, 0, 0))
    vehicle.GetChassisBody().SetPos(chrono.ChVector3d(-75, 0, 1.6))
    vehicle.GetChassisBody().SetRot(chrono.ChQuaternion1d(1, 0, 0, 0))

    
    vehicle.SetThrottle(0.1)
    vehicle.SetGear(1)
    vehicle.GetVehicle().EnableRealtime(True)

    
    terrain = veh.RigidTerrain(vehicle.GetSystem())
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    terrain.AddPatch(patch_mat, 
                     chrono.CSYSNORM, 
                     200, 200)
    terrain.Initialize()

    
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('ChronicARGo: ARGo (single track, Rigid Chassis)')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(vehicle.GetVehicle())

    
    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()

    
    while vis.Run() :
        
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        
        driver_inputs = driver.GetInputs()

        
        time = vehicle.GetSystem().GetChTime()
        driver.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs)
        terrain.Synchronize(time)
        vis.Synchronize(time, driver_inputs)

        
        driver.Advance(step_size)
        vehicle.Advance(step_size)
        terrain.Advance(step_size)
        vis.Advance(step_size)

if __name__ == "__main__":
    main()