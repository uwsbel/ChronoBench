import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math as m



def main():
    

    
    rover = veh.Viper()
    rover.SetContactMethod(chrono.ChContactMethod_NSC)
    rover.SetChassisFixed(False);
    rover.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(-10, -2, 0.6), chrono.ChQuaterniond(1, 0, 0, 0)))
    rover.SetEngineType(veh.EngineModelType_SIMPLE)
    rover.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    rover.SetDriveType(veh.DrivelineTypeWV_AWD)
    rover.SetTireType(veh.TireModelType_RIGID)
    rover.Initialize()

    rover.SetChassisVisualizationType(veh.VisualizationType_NONE)
    rover.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    rover.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    rover.SetWheelVisualizationType(veh.VisualizationType_NONE)
    rover.SetTireVisualizationType(veh.VisualizationType_MESH)

    rover.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    

    terrain = veh.RigidTerrain(rover.GetSystem())
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 100.0, 100.0)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    terrain.Initialize()

    
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('Viper')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(rover.GetVehicle())

    
    driver = veh.ChInteractiveDriverIRR(vis)

    
    steering_time = 0.5  
    throttle_time = 1.0  
    braking_time = 0.3   
    driver.SetSteeringDelta(render_step_size / steering_time)
    driver.SetThrottleDelta(render_step_size / throttle_time)
    driver.SetBrakingDelta(render_step_size / braking_time)

    driver.Initialize()

    
    rover.GetSystem().SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)

    
    rover.GetVehicle().EnableRealtime(True)

    render_steps = m.ceil(render_step_size / step_size)
    step_number = 0
    render_frame = 0

    rover.GetVehicle().EnableRealtime(True)

    while vis.Run() :
        time = rover.GetSystem().GetChTime()

        
        if (step_number % render_steps == 0) :
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        
        driver_inputs = driver.GetInputs()

        
        driver.Synchronize(time)
        terrain.Synchronize(time)
        rover.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        
        driver.Advance(step_size)
        terrain.Advance(step_size)
        rover.Advance(step_size)
        vis.Advance(step_size)

        
        step_number += 1

    return 0







veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


step_size = 2e-3;


render_step_size = 1.0 / 50;  

main()