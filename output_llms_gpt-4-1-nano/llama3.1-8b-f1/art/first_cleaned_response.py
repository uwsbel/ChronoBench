import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math as m






veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_TMEASY





estimation_method = veh.RigidBodyEstimationMethod_NONE


trackPoint = chrono.ChVector3d(0.0, 0.0, 1.75)


step_size = 1e-3
tire_step_size = step_size


tend = 15


render_step_size = 1.0 / 50  










driver = veh.ChDriver()
driver.SetSteeringDelta(0.1)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.2)
driver.Initialize()

tireL = veh.ChTireVehicleTmeasy(veh.LEFT, vehicle)
tireR = veh.ChTireVehicleTmeasy(veh.RIGHT, vehicle)

vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.Initialize()

suspF = veh.ChSuspension(vehicle.GetChassis(), vehicle.GetSuspension(1, veh.LEFT), SPRINGS_STIFFNESS, DAMPERS_RESTRCTION)
suspR = veh.ChSuspension(vehicle.GetChassis(), vehicle.GetSuspension(1, veh.RIGHT), SPRINGS_STIFFNESS, DAMPERS_RESTRCTION)

springF = chrono.ChLinkSpringCollision(suspF.GetSprocket(veh.LEFT), suspF.GetWheel(veh.LEFT), SPRINGS_STIFFNESS, 0, true)
damperF = chrono.ChLinkDamperCollision(suspF.GetSprocket(veh.LEFT), suspF.GetWheel(veh.LEFT), DAMPERS_RESTRCTION, 0, true)
jntF = chrono.ChLinkLockCollision(suspF.GetSprocket(veh.LEFT), suspF.GetChassis(), true)

springR = chrono.ChLinkSpringCollision(suspR.GetSprocket(veh.RIGHT), suspR.GetWheel(veh.RIGHT), SPRINGS_STIFFNESS, 0, true)
damperR = chrono.ChLinkDamperCollision(suspR.GetSprocket(veh.RIGHT), suspR.GetWheel(veh.RIGHT), DAMPERS_RESTRCTION, 0, true)
jntR = chrono.ChLinkLockCollision(suspR.GetSprocket(veh.RIGHT), suspR.GetChassis(), true)

springF.Initialize()
damperF.Initialize()
jntF.Initialize()
springR.Initialize()
damperR.Initialize()
jntR.Initialize()

suspF.GetLink0().SetFixed(True)
suspR.GetLink0().SetFixed(True)


ground = chrono.ChBodyEasyBox(10, 10, 1, 1000, True, True)
ground.SetPos(chrono.ChVector3d(0, 0, 0))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 10, 10)
vehicle.GetSystem().Add(ground)


chassis = vehicle.GetChassis()
chassis.SetPos(initLoc)
chassis.SetRot(initRot)
chassis.SetFixed(False)

chassis.GetVisualShape(1).SetMesh(veh.GetDataFile("vehicle/chassis.obj"), 10, True)


collision_shape = chrono.ChCollisionShapeBox(chassis.GetVisualShape(1).GetHalfSize(), 1000)
chassis.AddCollisionShape(collision_shape)


chassis.SetChassisVisualizationType(chassis_vis_type)






print( "VEHICLE MASS: ",  vehicle.GetMass())


render_steps = m.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while (vehicle.Run()) :
    time = vehicle.GetChTime()

    
    if (time >= tend):
        break

    
    if (step_number % render_steps == 0) :
        render_frame = render_frame + 1
        
        cam = irr.ChCameraSceneNode(orb, 60.0, 0.1, 1000.0)
        cam.SetPosition(IRR_CH_VECTOR3D(trackPoint.x, trackPoint.y, trackPoint.z))
        cam.SetTarget(IRR_CH_VECTOR3D(0, 0, 0))
        cam.SetUp(IRR_CH_VECTOR3D(0, 1, 0))
        cam.AttachTo(orb)
        orb.AddCamera(cam)
        
        
        vis.Run()
        
        
        if (save_povray):
            out = open('scene.pov','w')
            out.write(pov_header)
            out.write('global_light1.Location = '+str(trackPoint.x)+' '+str(trackPoint.y)+' '+str(trackPoint.z)+'\n')
            out.write('global_light2.Location = '+str(trackPoint.x)+' '+str(trackPoint.y)+' '+str(trackPoint.z)+'\n')
            out.write('camera.Location = '+str(trackPoint.x)+' '+str(trackPoint.y)+' '+str(trackPoint.z)+'\n')
            out.write('camera.Target = 0 0 0\n')
            out.write('camera.Up = 0 1 0\n')
            out.write('background color = '+str(bcolor.x)+' '+str(bcolor.y)+' '+str(bcolor.z)+'\n')
            out.write('light1.Location = '+str(groundPos.x)+' '+str(groundPos.y)+' '+str(groundPos.z)+'\n')
            out.write('light2.Location = '+str(groundPos.x)+' '+str(groundPos.y)+' '+str(groundPos.z)+'\n')
            out.write('box1.Location = 0 0 0\n')
            out.write('box2.Location = 0 0 0\n')
            out.write('box3.Location = 0 0 0\n')
            out.write('box4.Location = 0 0 0\n')
            out.write('box5.Location = 0 0 0\n')
            out.write('box6.Location = 0 0 0\n')
            out.write('render { render_width '+str(render_width)+ ' render_height '+str(render_height)+' }\n')
            out.write(''\n')
            out.close()
            vis.SavePOV-Ray('scene', render_width, render_height)
    
    
    driver_inputs = driver.GetInputs()
    driver.Reset()
    vehicle.Synchronize(time)
    driver.Synchronize(time)
    
    
    
    
    
    
    
    
    
    
    
    
    
    driver.Synchronize(step_size, time, delta_time)
    vehicle.Synchronize(step_size, time, delta_time)
    ground.Synchronize(step_size, time, delta_time)
    
    
    
    
    vis.Synchronize(step_size, time, delta_time, render_steps)
    
    
    driver.Advance(step_size)
    vehicle.Advance(step_size)
    ground.Advance(step_size)
    vis.Advance(step_size)
    
    
    step_number += 1