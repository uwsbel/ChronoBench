import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math as m



def main():
    

    
    truck = veh.MAN_10t()
    truck.SetContactMethod(chrono.ChContactMethod_NSC)
    truck.SetChassisFixed(False);
    truck.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(-10, -2, 0.6), chrono.ChQuaterniond(1, 0, 0, 0)))
    truck.SetEngineType(veh.EngineModelType_SHAFTS)
    truck.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    truck.SetTireType(veh.TireModelType_TMEASY)
    truck.SetTireStepSize(1e-3)
    truck.Initialize()

    truck.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    truck.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    truck.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    truck.SetWheelVisualizationType(veh.VisualizationType_NONE)
    truck.SetTireVisualizationType(veh.VisualizationType_MESH)

    truck.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    
    terrain = veh.RigidTerrain(truck.GetSystem())
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 1.0))
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    terrain.Initialize()

    
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('MAN 10t')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(truck.GetVehicle())

    
    driver = veh.ChInteractiveDriverIRR(vis)

    
    steering_time = 1.0  
    throttle_time = 1.0  
    braking_time = 0.3   
    driver.SetSteeringDelta(10 * step_size / steering_time)
    driver.SetThrottleDelta(10 * step_size / throttle_time)
    driver.SetBrakingDelta(10 * step_size / braking_time)

    driver.Initialize()

    
    truck.GetVehicle().EnableRealtime(True)

    while vis.Run() :
        time = truck.GetSystem().GetChTime()

        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        
        driver_inputs = driver.GetInputs()

        
        driver.Synchronize(time)
        terrain.Synchronize(time)
        truck.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        
        driver.Advance(step_size)
        terrain.Advance(step_size)
        truck.Advance(step_size)
        vis.Advance(step_size)

    return 0







veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


step_size = 1e-3

main()