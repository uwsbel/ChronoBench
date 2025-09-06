import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis = chrono.ChVisualSystemIrrlicht()
vis.AttachSystem(my_system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Simulation Window')
vis.SetWindowSize(1024, 768)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 0), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())


vis.SetChassisVisualizationType(veh.VisualizationType_MESH)
vis.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
vis.SetSteeringVisualizationType(veh.VisualizationType_MESH)
vis.SetWheelVisualizationType(veh.VisualizationType_MESH)
vis.SetTireVisualizationType(veh.VisualizationType_Mesh)


chassis_collision_type = chrono.ChCollisionModel.Type_BULLET


body = chrono.ChBody()
body.SetPos(chrono.ChVector3d(0, -size_table_y / 2, 0))
body.SetMass(100)
body.SetInertiaXX(chrono.ChVector3d(10, 10, 10))
body.SetRayleighDamping(0.000)
hnode1 = chrono.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
hnode2 = chrono.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_L, 0, 0)))
mesh = chrono.ChMesh()
mesh.AddNode(hnode1)
mesh.AddNode(hnode2)
body.AddMesh(mesh)


rev_joint = chrono.ChLinkLockRevolute()
rev_joint.Initialize(body1, body2, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
rev_joint.SetCollisionModel(chrono.ChCollisionModel.Type_BULLET)
rev_joint.SetCollisionType(chassis_collision_type)
rev_joint.SetFixed(False)
rev_joint.SetInitialPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
rev_joint.SetRotation(chrono.ChQuaterniond(1, 0, 0, 0))


vehicle = chrono.ChBodyEasySphere(radius=1.0, density=1000, visualize=True)
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.SetInitialRotation(chrono.ChQuaterniond(1, 0, 0, 0))
vehicle.SetMass(100)
vehicle.SetInertiaXX(chrono.ChVector3d(10, 10, 10))
vehicle.SetRayleighDamping(0.000)
vehicle.SetDampingF(0.1)
vehicle.SetCompliance(0.01)
vehicle.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
vehicle.SetInitialPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
vehicle.SetRotation(chrono.ChQuaterniond(1, 0, 0, 0))
vehicle.SetInitialVelocity(chrono.ChVector3d(0, 0, 0))
vehicle.SetInitialSpeed(chrono.ChFunctionSine(0.001, 1.5))
vehicle.SetInitialSteering(chrono.ChFunctionSine(0.001, 1.5))
vehicle.SetInitialThrottle(chrono.ChFunctionSine(0.001, 1.5))
vehicle.SetBraking(0.3)
vehicle.SetInitialVelocity(chrono.ChVector3d(0, 0, 0))
vehicle.SetInitialSpeed(chrono.ChFunctionSine(0.001, 1.5))
vehicle.SetInitialRotation(chrono.ChQuaterniond(1, 0, 0, 0))
vehicle.SetInitialVelocity(chrono.ChVector3d(0, 0, 0))
vehicle.SetInitialSpeed(chrono.ChFunctionSine(0.001, 1.5))


while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1
    if step_number % log_steps == 0:
        
        gps_coor = vehicle.GetMostRecentGPSBuffer().GetGPSData()
        gps_data.append([gps_coor[0], gps_coor[1], gps_coor[2]])
    
    if time < 2.0:
        driver.SetThrottle(0.6)
        driver.SetSteering(0.0)
    elif time < 4.0 and time > 2.0:
        driver.SetThrottle(0.6)
        driver.SetSteering(0.5)
    elif time < 6.0 and time > 4.0:
        driver.SetThrottle(0.6)
        driver.SetSteering(-0.5)
    else:
        driver.SetBraking(1.0)
    driver_inputs = driver.GetInputs()
    
    
    driver.Synchronize(time)
    terrain.Synchronize(time, driver_inputs, terrain)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    manager = sens.ChSensorManager(vehicle.GetSystem())
    manager.Update()
    
    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    manager.Update()
    
    
    step_number += 1

    
    realtime_timer.Spin(step_size)
    
print("GPS Data: ", gps_data)