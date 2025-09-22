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





estimation_method = veh.InertiaEstimationMethod_NONE


out_dir = "./"


step_size = 1e-3
tire_step_size = step_size


tend = 1000


render_step_size = 1.0 / 50  










truck = veh.Truck(veh.DataFile("truck/truck.json"), 
                  tire_model, 
                  tire_step_size)
truck.SetChassisCollisionType(chassis_collision_type)
truck.SetChassisFixed(False)
truck.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
truck.SetTireRollingResistance(0.06)
truck.SetTireViscosity(0.3)
truck.Initialize()

truck.SetChassisVisualizationType(chassis_vis_type)
truck.SetSuspensionVisualizationType(suspension_vis_type)
truck.SetSteeringVisualizationType(steering_vis_type)
truck.SetWheelVisualizationType(wheel_vis_type)

truck.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


ground = chrono.ChBody()
ground.SetFixed(True)
truck.GetSystem().Add(ground)


tire_vis = veh.ChTireVisualizationSystemPRIMITIVES()
tire_vis.SetMaxRadius(0.1)
tire_vis.SetLineThickness(0.01)
tire_vis.SetInitPosition(chrono.ChVector3d(0, 0, 0))
truck.GetSystem().Add(tire_vis)





vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('DAF CF 75.340')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(truck.GetVehicle())





truck.GetVehicle().EnableRealtime(True)

while vis.Run() :
    time = truck.GetSystem().GetChTime()

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    driver_inputs = truck.GetDriver().GetInputs()

    
    
    truck.Synchronize(time)
    ground.Synchronize(time)
    vis.Synchronize(time, driver_inputs)

    
    truck.Advance(step_size)
    ground.Advance(step_size)
    vis.Advance(step_size)

    
    
    
    truck.GetSystem().DoStepDynamics(step_size)