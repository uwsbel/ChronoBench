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


inertia_method = veh.InertiaEstimationMethod_MOMENTS


trackPoint = chrono.ChVector3d(0.0, 0.0, 1.75)


step_size = 1e-3
tire_step_size = step_size


tend = 15


render_step_size = 1.0 / 50  


out_dir = "./POOPoon"










poon = veh.PoonChassis()
poon.SetContactMethod(chrono.ChContactMethod_NSC)
poon.SetChassisCollisionType(chassis_collision_type)
poon.SetChassisFixed(False)
poon.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
poon.SetBrakeType(veh.BrakeType_SHAFTS)
poon.SetTireType(tire_model)
poon.SetTireStepSize(tire_step_size)
poon.SetInitFwdVel(0.0)
poon.Initialize()

poon.SetChassisVisualizationType(chassis_vis_type)
poon.SetSuspensionVisualizationType(suspension_vis_type)
poon.SetSteeringVisualizationType(steering_vis_type)
poon.SetWheelVisualizationType(wheel_vis_type)

suspF = veh.CastToChToeBarLeafspringAxle(poon.GetVehicle().GetSuspension(0))
leftAngle = suspF.GetKingpinAngleLeft()
rightAngle = suspF.GetKingpinAngleRight()

springFL = suspF.GetSpring(veh.LEFT)
shockFL = suspF.GetShock(veh.RIGHT)

poon.springFL_plot = chrono.ChLinePlot(out_dir + "/springF_L.txt")
poon.shockFL_plot = chrono.ChLinePlot(out_dir + "/shockF_L.txt")

poon.springRL_plot = chrono.ChLinePlot(out_dir + "/springF_R.txt")
poon.shockRL_plot = chrono.ChLinePlot(out_dir + "/shockF_R.txt")

poon.springFL_plot.SetTitle("Front Spring and Shock Forces")
poon.springFL_plot.SetXtitle("t")
poon.springFL_plot.SetYtitle("F")
poon.springFL_plot.Initialize()
poon.shockFL_plot.SetTitle("Front Shock Forces")
poon.shockFL_plot.SetXtitle("t")
poon.shockFL_plot.SetYtitle("F")
poon.shockFL_plot.Initialize()
poon.springRL_plot.SetTitle("Rear Spring and Shock Forces")
poon.springRL_plot.SetXtitle("t")
poon.springRL_plot.SetYtitle("F")
poon.springRL_plot.Initialize()
poon.shockRL_plot.SetTitle("Rear Shock Forces")
poon.shockRL_plot.SetXtitle("t")
poon.shockRL_plot.SetYtitle("F")
poon.shockRL_plot.Initialize()

def updatePlots():
    suspF = veh.CastToChToeBarLeafspringAxle(poon.GetVehicle().GetSuspension(0))
    springFL = suspF.GetSpring(veh.LEFT)
    shockFL = suspF.GetShock(veh.RIGHT)
    springRL = suspF.GetSpring(veh.RIGHT)
    shockRL = suspF.GetShock(veh.LEFT)

    time = poop.GetSystem().GetChTime()
    poop.springFL_plot.AddData(time, springFL.GetForce())
    poop.shockFL_plot.AddData(time, shockFL.GetForce())
    poop.springRL_plot.AddData(time, springRL.GetForce())
    poop.shockRL_plot.AddData(time, shockRL.GetForce())


tire_vis = veh.ChTireVisualSystemPoon(poon.GetVehicle())
tire_vis.SetTireRigidElementVisualizationType(veh.VisualizationType_MESH)
tire_vis.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Poon chassis')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(poon.GetVehicle())


driver = veh.ChInteractiveDriverIRR(vis)


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()






print( "VEHICLE MASS: ",  poop.GetVehicle().GetMass())


render_steps = m.ceil(render_step_size / step_size)


step_number = 0
render_frame = 0

poon.GetVehicle().EnableRealtime(True)

while vis.Run() :
    time = poop.GetSystem().GetChTime()

    
    if (step_number % render_steps == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    tire_vis.Synchronize(time)
    poon.Synchronize(time, driver_inputs, tire_vis)
    vis.Synchronize(time, driver_inputs)

    
    driver.Advance(step_size)
    tire_vis.Advance(step_size)
    poon.Advance(step_size)
    vis.Advance(step_size)

    
    if (step_number % render_steps != 0) :
        updatePlots()

    
    step_number += 1