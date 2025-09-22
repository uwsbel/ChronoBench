import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

def main():
    

    
    
    

    
    vehicle = veh.CityBus()
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    vehicle.SetChassisFixed(False) 
    vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(-10, -2, 0.75), chrono.QUNIT))
    vehicle.SetEngineType(veh.EngineModelType_SIMPLE_MAP)
    vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    vehicle.SetTireType(veh.TireModelType_TMEASY)
    vehicle.Initialize()
    vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH);
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES);
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES);
    vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH);

    vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    
    terrain = veh.RigidTerrain(vehicle.GetSystem())
    if (veh.HasOFFAPI()):
        patch_mat = chrono.ChContactMaterialNSC()
        patch_mat.SetFriction(0.9)
        patch = terrain.AddPatch(patch_mat, 
                                 chrono.ChCoordsysd(chrono.VNULL, chrono.QUNIT), 
                                 200., 100.)
        patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
        patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('CityBus')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(vehicle)

    
    driver = veh.ChInteractiveDriverIRR(vis)

    
    steering_time = 1.0  
    throttle_time = 1.0  
    braking_time = 0.3   
    driver.SetSteeringDelta(10 * steering_time)
    driver.SetThrottleDelta(10 * throttle_time)
    driver.SetBrakingDelta(10 * braking_time)
    driver.Initialize()

    
    
    

    
    step_number = 0

    while vis.Run() :
        
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        
        

        
        driver_inputs = driver.GetInputs()
        vehicle.Update(driver_inputs, step_number)
        terrain.Update()

        
        vehicle.GetSystem().DoStepDynamics(0.01)

        
        step_number += 1


main()