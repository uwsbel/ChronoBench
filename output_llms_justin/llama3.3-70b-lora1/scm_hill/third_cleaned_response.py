import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math






initLoc = chrono.ChVector3d(-15, 0, 1.2)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_RIGID


terrainHeight = 0      
terrainLength = 100.0  
terrainWidth = 100.0   


trackPoint = chrono.ChVector3d(0.0, 0.0, 1.71)


contact_method = chrono.ChContactMethod_NSC
contact_vis = False


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50  


vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


patch_mat = veh.RigidTerrain.RigidTerrainMaterialNs(0.01, 0.9)
patch = veh.RigidTerrain.RigidTerrainPatchIrregularHeight(chrono.GetChronoDataFile('terrain/rigid_plane.txt'), patch_mat, 75e3, 20)
terrain = veh.RigidTerrain(vehicle.GetSystem())
terrain.AddPatch(patch)
terrain.Initialize()


terrain.SetPlotType(veh.RigidTerrain.PLOT_PRESSURE, 0, core.ChColor(0,0,0))
patch.SetColor(veh.RigidTerrain.PLOT_PRESSURE, core.ChColor(0,0,0))
patch.SetColor(veh.RigidTerrain.PLOT_HEIGHT, core.ChColor(0,0,0))


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())


vis.BindAll()


driver = veh.ChInteractiveDriverIRR(vis)


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()






render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0



try:
    
    mbuf = open('output/data.txt', 'w')
except:
    print('Error creating file.')
    exit(13)





while vis.Run() :
    time = vehicle.GetSystem().GetChTime()

    
    if (step_number % render_steps == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    if not(vehicle.Debug() ):
        print( "Error in vehicle. Aborting.")
        break

    
    vis.Resume()

    
    if (step_number % 100 == 0) :
        print( "Time = {0:.3f}s OUTPUT:\n".format(time) + vehicle.GetVehicle().GetOutput(true))

    
    if (step_number % 1 == 0) :
        veh_data = vehicle.GetVehicle().GetState()

        
        mbuf.write( str(time) + "\t" +
                    str(veh_data.m_pos[0]) + " " + str(veh_data.m_pos[1]) + " " + str(veh_data.m_pos[2]) + " " +
                    str(veh_data.m_rot.GetX1()) + " " + str(veh_data.m_rot.GetY1()) + " " + str(veh_data.m_rot.GetZ1()) + " " +
                    str(veh_data.m_rot.GetX2()) + " " + str(veh_data.m_rot.GetY2()) + " " + str(veh_data.m_rot.GetZ2()) + " " +
                    str(veh_data.m_rot.GetX3()) + " " + str(veh_data.m_rot.GetY3()) + " " + str(veh_data.m_rot.GetZ3()) + " " +
                    str(veh_data.m_posdot[0]) + " " + str(veh_data.m_posdot[1]) + " " + str(veh_data.m_posdot[2]) + " " +
                    str(veh_data.m_w[0]) + " " + str(veh_data.m_w[1]) + " " + str(veh_data.m_w[2]) + "\n" )

    
    step_number += 1

    
    realtime_timer.Spin(step_size)





print( "End simulation \n")
mbuf.close()
print("error happened with only start ```python")