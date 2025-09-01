import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math as m
import errno
import os
import math




def main() : 
    

    
    gator = veh.Gator()
    gator.SetContactMethod(chrono.ChContactMethod_SMC)
    gator.SetChassisCollisionType(veh.CollisionType_NONE)
    gator.SetChassisFixed(False); 
    gator.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(-10, -2, 0.6), chrono.ChQuaterniond(1, 0, 0, 0)))
    gator.SetEngineType(veh.EngineModelType_SIMPLE);
    gator.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP);
    gator.SetDriveType(veh.DrivelineTypeWV_AWD)
    gator.SetTireType(veh.TireModelType_TMEASY)
    gator.SetTireStepSize(tire_step_size)
    gator.Initialize()

    gator.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    gator.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    gator.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    gator.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
    gator.SetTireVisualizationType(veh.VisualizationType_MESH)

    gator.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    
    terrain = veh.RigidTerrain(gator.GetSystem())
    if (contact_method == chrono.ChContactMethod_NSC):
        patch_mat = chrono.ChContactMaterialNSC()
        patch_mat.SetFriction(0.9)
        patch_mat.SetRestitution(0.01)
    elif (contact_method == chrono.ChContactMethod_SMC):
        patch_mat = chrono.ChContactMaterialSMC()
        patch_mat.SetFriction(0.9)
        patch_mat.SetRestitution(0.01)
        patch_mat.SetYoungModulus(2e7)
    patch = terrain.AddPatch(patch_mat, 
                             chrono.CSYSNORM, 
                             terrainLength, terrainWidth)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('Gator')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(gator.GetVehicle())

    
    driver = veh.ChInteractiveDriverIRR(vis)

    
    steering_time = 1.0  
    throttle_time = 1.0  
    braking_time = 0.3   
    driver.SetSteeringDelta(render_step_size / steering_time)
    driver.SetThrottleDelta(render_step_size / throttle_time)
    driver.SetBrakingDelta(render_step_size / braking_time)

    driver.Initialize()

    
    gator.GetVehicle().EnableRealtime(True)

    while vis.Run() :
        time = gator.GetSystem().GetChTime()

        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        
        driver_inputs = driver.GetInputs()

        
        driver.Synchronize(time)
        terrain.Synchronize(time)
        gator.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        
        driver.Advance(step_size)
        terrain.Advance(step_size)
        gator.Advance(step_size)
        vis.Advance(step_size)








veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(-10, -2, 0.6)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


chassis_vis_type = veh.VisualizationType_PRIMITIVES
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_PRIMITIVES
tire_vis_type = veh.VisualizationType_MESH 


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_TMEASY


terrainHeight = 0;      
terrainLength = 100.0;  
terrainWidth = 100.0;   


trackPoint = chrono.ChVector3d(0.0, 0.0, 1.75)


contact_method = chrono.ChContactMethod_SMC


step_size = 3e-3;
tire_step_size = 1e-3;


render_step_size = 1.0 / 50;  

main()