import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math as m




def main():
    

    

    
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(contact_method)
    hmmwv.SetChassisCollisionType(chassis_collision_type)
    hmmwv.SetChassisFixed(False) 
    hmmwv.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
    hmmwv.SetEngineType(engine_model)
    hmmwv.SetTransmissionType(transmission_model)
    hmmwv.SetDriveType(drive_type)
    hmmwv.SetSteeringType(steering_type)
    hmmwv.SetTireType(tire_model)
    hmmwv.SetTireStepSize(tire_step_size)
    hmmwv.Initialize()

    hmmwv.SetChassisVisualizationType(chassis_vis_type)
    hmmwv.SetSuspensionVisualizationType(suspension_vis_type)
    hmmwv.SetSteeringVisualizationType(steering_vis_type)
    hmmwv.SetWheelVisualizationType(wheel_vis_type)
    hmmwv.SetTireVisualizationType(tire_vis_type)

    hmmwv.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    
    

    terrain = veh.SCMTerrain(hmmwv.GetSystem())
    terrain.SetSoilParameters(soilKphi, soilKc, soilCs);

    if (soil_mesh_type == 0):
        grid_data = veh.GridData()
        grid_data.SetGridSpacing(0.02, 0.02, 0.02)
        grid_data.SetGridBounds(
            hmmwv.GetChassis().GetPos() - chrono.ChVector3d(1, 1, 0.1),
            hmmwv.GetChassis().GetPos() + chrono.ChVector3d(1, 1, 0.1),
        )
        terrain.Initialize(
            grid_data, alpha, eta, beamLength, beamRez, beamK, youngs, poisson, rolloff, thickness
        )
    elif (soil_mesh_type == 1):
        triangle_data = veh.TriangleData()
        triangle_data.SetFromWaveform(terrainWave, minHeight, maxHeight, scale)
        terrain.Initialize(
            triangle_data, Ks, Cs, Rs, Kt, Ct, Rt, alpha, eta, beamLength, beamRez, beamK, youngs, poisson, rolloff, thickness
        )

    if (movingPATCH):
        patch_pos = hmmwv.GetChassis().GetPos() + chrono.ChVector3d(0, -3, 0)
        terrain.AddPatch(patch_pos, cos_angle, length, width)
    else:
        terrain.AddPatch(chrono.ChVector3d(13, 3, 0.15), cos_angle, length, width)
        terrain.AddPatch(chrono.ChVector3d(13, -3, 0.15), cos_angle, length, width)

    
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('HMMWV+SCM Deformable Terrain')
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

    
    hmmwv.GetSystem().SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    hmmwv.GetSystem().GetSolver().AsIterative().SetMaxIterations(1500)
    hmmwv.GetSystem().SetGravitationalAcceleration(earth_gravity)

    
    hmmwv.GetSystem().SetChTime(0)
    render_step_size = 1.0 / 50

    
    try:
        out_dir = os.path.join(os.path.dirname(__file__), "output")
        os.makedirs(out_dir, exist_ok=True)
    except NameError:
        out_dir = "./output"
        os.makedirs(out_dir, exist_ok=True)

    step_number = 0
    while vis.Run() :
        time = hmmwv.GetSystem().GetChTime()

        
        if (time >= t_end):
            vis.Quit()

        
        driver_inputs = driver.GetInputs()
        hmmwv.Synchronize(time, driver_inputs)
        terrain.Synchronize(time)
        vis.Synchronize(time, driver_inputs)

        
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        
        driver.Advance(step_size)
        hmmwv.Advance(step_size)
        terrain.Advance(step_size)
        vis.Advance(step_size)

        if (step_number % 10 == 0) :
            print("Time = {0:.4f}".format(time))

        step_number += 1

    return 0







veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(0, 1.5, 0.4)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_MESH
tire_vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


engine_model = veh.EngineModelType_SHAFTS
transmission_model = veh.TransmissionModelType_AUTOMATIC_SHAFTS


drive_type = veh.DrivelineTypeWV_AWD


steering_type = veh.SteeringTypeWV_PITMAN_ARM


tire_model = veh.TireModelType_TMEASY


terrainHeight = 0.0  
terrainLength = 100.0  
terrainWidth = 100.0   









soilKphi = 0.2e6
soilKc = 0.012e6
soilCs = 0


patch_pos = chrono.ChVector3d(0, 0, terrainHeight)
cos_angle = 0
length = 3
width = 6


contact_method = chrono.ChContactMethod_NSC


step_size = 2e-3;
tire_step_size = 1e-3;


t_end = 1000;


soil_mesh_type = 0  
alpha = 0.0
eta = 1.0e8
beamLength = 0.05
beamRez = 1.0
beamK = 1.0e4
youngs = 3.0e7
poisson = 0.3
rolloff = 0.05
thickness = 0.05


terrainWave = veh.GetDataFile('terrain/SCM/terrainwave.txt')
minHeight = 0
maxHeight = 1
scale = 5

Ks = 4e7        
Cs = 0          
Rs = 0.05       
Kt = 8e6        
Ct = 0          
Rt = 0.02       


movingPATCH = True


earth_gravity = 9.82  

main()