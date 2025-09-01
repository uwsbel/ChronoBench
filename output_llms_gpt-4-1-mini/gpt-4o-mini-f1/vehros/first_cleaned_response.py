import pychrono as ch
import pychrono.vehicle as veh
import pychrono.ros as chros
import numpy as np
import math


initLoc = ch.ChVector3d(0, 0, 0.5)
initRot = ch.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_TMEASY



terrainHeight = 0      
terrainLength = 100.0  
terrainWidth = 100.0   


trackPoint = ch.ChVector3d(-3.0, 0.0, 1.1)


contact_method = ch.ChContactMethod_NSC
contact_vis = False


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50  






vehicle = veh.HMMWV_Full()  
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(ch.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.SetShaftLock(True)
vehicle.SetSteeringLock(steering_lock)
vehicle.SetTireStepSize(tire_step_size)
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)

vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type, vis_type)

vehicle.GetSystem().SetCollisionSystemType(ch.ChCollisionSystem.Type_BULLET)


patch_mat = ch.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, 
    ch.ChCoordsysd(ch.ChVector3d(0, 0, 0), ch.QUNIT), 
    terrainLength, terrainWidth)

patch.SetTexture("", 200, 200)
patch.SetColor(ch.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


ros_manager = chros.ChROSVehicleManager(vehicle.GetSystem())
ros_manager.RegisterHandler(chros.ChROSVehicleManager.CLOCK, chros.ChROSVehicleClockHandler())
ros_manager.RegisterHandler(chros.ChROSVehicleManager.DRIVER_INPUTS, chros.ChROSVehicleDriverInputsHandler())
ros_manager.RegisterHandler(chros.ChROSVehicleManager.VEHICLE_STATE, chros.ChROSVehicleVehicleStateHandler())
ros_manager.Initialize()


driver = veh.ChInteractiveDriverIRB(vehicle.GetChassisBody())
driver.Initialize()


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.SetLookAheadDistance(3.0)






print( "VEHICLE MASS: ",  vehicle.GetVehicle().GetMass())


render_steps = math.ceil(render_step_size / step_size)


realtime_timer = ch.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while (ros_manager.Run()) :
    time = vehicle.GetSystem().GetChTime()

    
    if (step_number % render_steps == 0) :
        
        ros_manager.Update()
        render_frame += 1

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    ros_manager.Synchronize(time, driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    ros_manager.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)