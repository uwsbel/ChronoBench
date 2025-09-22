import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math
import os



def main():
    

    
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(contact_method)
    hmmwv.SetChassisFixed(False) 
    hmmwv.SetInitPosition(chrono.ChCoordsysd(initLoc, chrono.ChQuaterniond(1, 0, 0, 0)))
    hmmwv.SetEngineType(engine_model)
    hmmwv.SetTransmissionType(transmission_model)
    hmmwv.SetDriveType(drive_type)
    hmmwv.SetSteeringType(steering_type)
    hmmwv.SetTireType(tire_model)
    hmmwv.Initialize()

    
    terrain = veh.RigidTerrain(hmmwv.GetSystem())
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch = terrain.AddPatch(patch_mat, 
                             chrono.CSYSNORM, 
                             200, 200)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('HMMWV')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.15), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(hmmwv.GetVehicle())

    
    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta(0.06)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()

    
    cstr1 = hmmwv.GetChassis().GetConvexShape(0)
    cstr2 = hmmwv.GetChassis().GetConvexShape(1)
    cstr3 = hmmwv.GetChassis().GetConvexShape(2)
    cstr4 = hmmwv.GetChassis().GetConvexShape(3)
    cstr5 = hmmwv.GetChassis().GetConvexShape(4)
    cstr6 = hmmwv.GetChassis().GetConvexShape(5)

    
    left_vis = veh.ChVisualShapeWheeledTire(hmmwv.GetVehicle().GetLeftTire(0), 
                                            cstr1, cstr2, cstr3, cstr4, cstr5, cstr6)
    left_vis.Initialize()
    vis.AddTire(left_vis)

    
    right_vis = veh.ChVisualShapeWheeledTire(hmmwv.GetVehicle().GetRightTire(0), 
                                             cstr1, cstr2, cstr3, cstr4, cstr5, cstr6)
    right_vis.Initialize()
    vis.AddTire(right_vis)

    
    

    

    

    hmmwv.GetVehicle().GetSteering().SetPath(steering_path, 0.5, vis);
    hmmwv.GetVehicle().GetTransmission().SetPath(transmission_path, 1.0, vis);
    hmmwv.GetVehicle().GetEngine().SetPath(engine_path, 1.0, vis);

    
    
    

    
    step_number = 0

    
    m = hmmwv.GetVehicle().GetMass()
    print( "VEHICLE MASS = " , m , " , time = " , t , "\n")

    while vis.Run() :
        time = hmmwv.GetSystem().GetChTime()

        
        if (time >= t_end):
            vis.Quit()

        
        driver_inputs = driver.GetInputs()

        
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        
        driver.Advance(step_size)
        terrain.Advance(step_size)
        hmmwv.Advance(step_size)
        vis.Advance(step_size)

        
        step_number += 1

    
    raw_input("Press Return to exit...")
    




datadir = chrono.GetChronoDataPath() + 'vehicle/'


initLoc = chrono.ChVector3d(-50, 0, 10)


initRot = chrono.ChQuaterniond(1, 0, 0, 0)


tire_vis_type = veh.ChWheeledVehicle.VisualizationType_PRIMITIVES


contact_method = chrono.ChContactMethod_NSC


engine_model = veh.EngineModelType_SIMPLE


transmission_model = veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP


drive_type = veh.DrivelineTypeWV_AWD


steering_type = veh.SteeringTypeWV_PITMAN_ARM


tire_model = veh.TireModelType_TMEASY


terrain_length = 100.0  
terrain_width = 100.0   


steering_path = veh.ChPath.Load(veh.GetDataFile(steering_file, datadir), 2.0)
transmission_path = veh.ChPath.Load(veh.GetDataFile(transmission_file, datadir), 2.0)
engine_path = veh.ChPath.Load(veh.GetDataFile(engine_file, datadir), 2.0)


step_size = 2e-3


t_end = 100


vis_type = veh.ChWheeledVehicle.VisualizationType_PRIMITIVES




system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0,0,-9.81))


main()