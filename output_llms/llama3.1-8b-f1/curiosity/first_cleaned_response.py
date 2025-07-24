import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math as m




def main():
    

    
    

    
    car = veh.Car(chrono.ChContactMethod_NSC)
    car.SetContactMethod(chrono.ChContactMethod_SMC)
    car.SetChassisFixed(False);
    car.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(-10, -2, 0.6), chrono.ChQuaterniond(1, 0, 0, 0)))
    car.SetEngineType(veh.EngineModelType_SHAFTS);
    car.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS);
    car.SetTireType(veh.TireModelType_TMEASY)
    car.SetTireStepSize(1e-3);
    car.Initialize()

    car.SetChassisVisualizationType(veh.VisualizationType_NONE)
    car.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    car.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    car.SetWheelVisualizationType(veh.VisualizationType_MESH)
    car.SetTireVisualizationType(veh.VisualizationType_MESH)

    car.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('Toyota Celica GT')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(car.GetVehicle())

    
    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta(0.06)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()

    
    car.GetVehicle().EnableRealtime(True)

    while vis.Run() :
        time = car.GetSystem().GetChTime()

        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        
        driver_inputs = driver.GetInputs()

        
        driver.Synchronize(time)
        car.Synchronize(time, driver_inputs)
        vis.Synchronize(time, driver_inputs)

        
        driver.Advance(step_size)
        car.Advance(step_size)
        vis.Advance(step_size)

    return 0







veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


step_size = 2e-3


main()