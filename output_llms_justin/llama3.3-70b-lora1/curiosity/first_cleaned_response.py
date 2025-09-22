import pychrono as chrono
import pychrono.robot as robot
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math as m








veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(0, 0, 1.6)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_MESH


tire_model = veh.TireModelType_TMEASY


trackPoint = chrono.ChVector3d(0.0, 0.0, 1.75)


step_size = 1e-3
tire_step_size = step_size


tend = 15


render_step_size = 1.0 / 50  


out_dir = "./CURROVER"




print( "Copyright (c) 2017 projectchrono.org\n")






curverover = veh.CURVEROVER()
curverover.SetContactMethod(chrono.ChContactMethod_NSC)
curverover.SetChassisFixed(False)
curverover.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
curverover.SetTireType(tire_model)
curverover.SetTireStepSize(tire_step_size)
curverover.SetInitFwdVel(0.0)
curverover.Initialize()

curverover.SetChassisVisualizationType(chassis_vis_type)
curverover.SetSuspensionVisualizationType(suspension_vis_type)
curverover.SetSteeringVisualizationType(steering_vis_type)
curverover.SetWheelVisualizationType(wheel_vis_type)

suspF = curverover.GetVehicle().GetSuspension(0)
suspR = curverover.GetVehicle().GetSuspension(1)
springFL = suspF.GetSpring(veh.LEFT)
shockFL = suspF.GetShock(veh.LEFT)
springFR = suspF.GetSpring(veh.RIGHT)
shockFR = suspF.GetShock(veh.RIGHT)
springRL = suspR.GetSpring(veh.LEFT)
shockRL = suspR.GetShock(veh.LEFT)
springRR = suspR.GetSpring(veh.RIGHT)
shockRR = suspR.GetShock(veh.RIGHT)

print(curverover)





terrain = veh.RigidTerrain(curverover.GetSystem())
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, 
                         chrono.CSYSNORM, 
                         200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 1.0))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
terrain.Initialize()






vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('curverover')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(curverover.GetVehicle())

driver = veh.ChInteractiveDriverIRR(vis)


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()
















render_steps = m.ceil(render_step_size / step_size)


step_number = 0

curverover.GetVehicle().EnableRealtime(True)

while vis.Run() :
    time = curverover.GetSystem().GetChTime()

    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    curverover.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    curverover.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1