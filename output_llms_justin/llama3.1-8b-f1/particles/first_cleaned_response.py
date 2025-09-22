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





inertia_estimation = veh.InertiaEstimationType_NONE


trackPoint = chrono.ChVector3d(0.0, 0.0, 1.75)


step_size = 1e-3
tire_step_size = step_size


tend = 15


render_step_size = 1.0 / 50  










driver = veh.ChDriver()
driver.SetSteeringDelta(0.1)
driver.SetThrottleDelta(0.1)
driver.SetBrakingDelta(0.3)
driver.Initialize()

tireL = veh.ChTMeasyTire(tire_model, veh.TIRE_RADIUS, True,  veh.COLLISION_TYPE_NONE, 0.1, 0.03)
tireR = veh.ChTMeasyTire(tire_model, veh.TIRE_RADIUS, False, veh.COLLISION_TYPE_NONE, 0.1, 0.03)

assem_susp = veh.ChAssemblySuspension(tireL, tireR, veh.CSV_TYPE_SIMPLE_PARabolic, 0.04, False, 0, 0)
assem_susp.Initialize()

suspF = veh.ChSuspensionFront(0.2, assem_susp, veh.CSA_TYPE_LINEAR, 0, 0)
suspR = veh.ChSuspensionRear(0.2, assem_susp, veh.CSA_TYPE_LINLEAF, 0, 0.04)
suspF.Initialize()
suspR.Initialize()

chassis = veh.ChChassis(veh.CHA_TYPE_PRIMITIVES, chassis_vis_type, chassis_collision_type)
chassis.Initialize(chrono.Bodyd(initLoc, initRot), veh.CHassisMass, inertia_estimation, veh.MOMENTS_OF_INERTIA)

mastoL = veh.ChSteeringAssemblyFront(0.2, assem_susp, veh.MAST_TYPE_PRIMITIVES, steering_vis_type)
mastoR = veh.ChSteeringAssemblyFront(0.2, assem_susp, veh.MAST_TYPE_PRIMITIVES, steering_vis_type)
mastoL.Initialize()
mastoR.Initialize()

wheelL = veh.ChWheel(0.2, assem_susp, wheel_vis_type, True)
wheelR = veh.ChWheel(0.2, assem_susp, wheel_vis_type, False)
wheelL.Initialize()
wheelR.Initialize()

vehicle = veh.ChVehicle(chassis, suspF, suspR, mastoL, mastoR, wheelL, wheelR)
vehicle.Initialize()


driver_system = chrono.ChSystemNSC()
driver_system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
driver_system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
driver_system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
driver_system.SetMaxPenetrationRecoverySpeed(4.0)
driver_system.Initialize()

driver_system.AddLightDirectional()
driver_system.AddSkyBox()
driver_system.GetChassis().SetFixed(True)


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Scania HMMWV')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle)


driver_irr = veh.ChInteractiveDriverIRR(vis)
driver_irr.SetSteeringDelta(0.05)
driver_irr.SetThrottleDelta(0.02)
driver_irr.SetBrakingDelta(0.1)
driver_irr.Initialize()






print( "VEHICLE MASS: ",  vehicle.GetChassis().GetMass())


render_steps = m.ceil(render_step_size / step_size)


step_number = 0
render_frame = 0

vehicle.EnableRealtime(True)

while vis.Run() :
    time = vehicle.GetSystem().GetChTime()

    
    if (time >= tend):
        break

    
    if (step_number % render_steps == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, tire_step_size)
    vis.Synchronize(time, driver_inputs)

    
    driver.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1