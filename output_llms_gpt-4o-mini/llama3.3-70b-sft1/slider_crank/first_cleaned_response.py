import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math
import os




def main():
    

    

    
    mech_system = chrono.ChSystemNSC()
    mech_system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    mech_system.SetGravitationalAcceleration(chrono.ChVector3d(0,0,-9.81))
    mech_system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)

    
    car = veh.CarsimVehicle(mech_system)
    car.SetContactMethod(contact_method)
    car.SetChassisCollisionType(chassis_collision_type)
    car.SetChassisFixed(False) 
    car.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
    car.SetEngineType(veh.EngineModelType_SIMPLE_MAP)
    car.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    car.SetTireType(tire_model)
    car.SetTireStepSize(tire_step_size)
    car.Initialize()

    
    ground = veh.RigidGround(mech_system)
    ground.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    ground.Initialize()

    
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('Sedan+Trailer (JSON specification)')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(trackPoint, 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(car.GetVehicle())

    
    try:
        os.mkdir(out_dir)
    except OSError as exc:
        if exc.errno != errno.EEXIST:
           print("Error creating output directory " )
    
    
    car.SetChassisOutput(True);
    car.SetSuspensionOutput(True);
    car.SetSteeringOutput(True);
    car.SetOutput(veh.ChVehicleOutput.ASCII , out_dir, "output", 0.1);

    
    car.GetVehicle().ExportComponentList(out_dir + "/component_list.json");

    
    driver = veh.ChInteractiveDriverIRR(vis)

    
    steering_time = 1.0  
    throttle_time = 1.0  
    braking_time = 0.3   
    driver.SetSteeringDelta(render_step_size / steering_time)
    driver.SetThrottleDelta(render_step_size / throttle_time)
    driver.SetBrakingDelta(render_step_size / braking_time)

    driver.Initialize()

    
    
    render_steps = math.ceil(render_step_size / step_size)
    step_number = 0
    render_frame = 0

    maxKingpinAngle = 0.0

    car.GetVehicle().EnableRealtime(True)

    while vis.Run() :
        time = car.GetSystem().GetChTime()

        
        if (step_number % render_steps == 0) :
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            render_frame += 1

        
        driver_inputs = driver.GetInputs()

        
        driver.Synchronize(time)
        ground.Synchronize(time)
        car.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        
        driver.Advance(step_size)
        ground.Advance(step_size)
        car.Advance(step_size)
        vis.Advance(step_size)

        
        step_number += 1

    return 0







veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


chassis_vis_type = veh.VisualizationType_NONE
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_MESH
tire_vis_type = veh.VisualizationType_MESH 


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_TMEASY


terrainHeight = 0      
terrainLength = 300.0  
terrainWidth = 200.0   


trackPoint = chrono.ChVector3d(0.0, 0.0, 1.75)


contact_method = chrono.ChContactMethod_NSC
contact_vis = False


step_size = 1e-3;
tire_step_size = 1e-3;


render_step_size = 1.0 / 50;  


out_dir = os.path.join(os.path.dirname(__file__), "SENSOR_OUTPUT/")






main()