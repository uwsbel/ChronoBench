import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math as m



def main():
    

    
    
    driver = veh.ChDriver()
    vehicle = veh.ChTruck_Full()
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
    vehicle.SetChassisFixed(False) 
    vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
    vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
    vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    vehicle.SetDriveType(veh.DrivelineTypeWV_AWD)
    vehicle.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    vehicle.SetTireType(tire_model)
    vehicle.SetTireStepSize(tire_step_size)
    vehicle.Initialize()

    vehicle.SetChassisVisualizationType(chassis_vis_type)
    vehicle.SetSuspensionVisualizationType(suspension_vis_type)
    vehicle.SetSteeringVisualizationType(steering_vis_type)
    vehicle.SetWheelVisualizationType(wheel_vis_type)
    vehicle.SetTireVisualizationType(tire_vis_type)

    vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('DAF CF 75.340')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(trackPoint, 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(vehicle)

    
    driver.Initialize()

    
    try:
        output = open(out_file, 'w')
    except FileNotFoundError:
        print("Error accessing output folder for writing.")

    
    out_json = vehicle.ExportComponentList()
    output.write(out_json)
    output.close()

    
    while vis.Run() :
        time = vehicle.GetSystem().GetChTime()

        
        driver_inputs = driver.GetInputs()

        
        driver.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs, vis)
        vis.Synchronize(time, driver_inputs)

        
        driver.Advance(step_size)
        vehicle.Advance(step_size)
        vis.Advance(step_size)

        
        
        vehicle.GetSystem().DoStepDynamics(step_size)

    return 0







veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(0, 0, 1.6)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


chassis_vis_type = veh.VisualizationType_NONE
suspension_vis_type =  veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_MESH
tire_vis_type = veh.VisualizationType_MESH 


tire_model = veh.TireModelType_TMEASY


trackPoint = chrono.ChVector3d(0.0, 0.0, 1.75)


step_size = 3e-3;
tire_step_size = 1e-3;


t_end = 1000;


out_dir = chrono.GetChronoDataPath() + 'vehicle/DAF_CF_F340/'


out_file = out_dir + 'output.json'

main()