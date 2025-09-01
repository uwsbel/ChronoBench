import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math as m
import os








veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


chassis_CollisionType = veh.CollisionType_NONE
suspension_CollisionType = veh.CollisionType_PRIMITIVES
steering_CollisionType = veh.CollisionType_PRIMITIVES
wheel_CollisionType = veh.CollisionType_NONE
tire_CollisionType = veh.CollisionType_MESH


tire_model = veh.TireModelType_TMEASY


tmeasy_delay = 0.1
tmeasy_fz_scale = 1.0
tmeasy_mu_scale = 1.0


terrainHeight = 0      
terrainLength = 200.0  
terrainWidth = 200.0   


trackPoint = chrono.ChVector3d(0.0, 0.0, 0.0)


step_size = 1e-3
tire_step_size = step_size


tend = 1000


render_step_size = 1.0 / 50  


out_dir = "./VEHICLE_OUTPUT"





print( "Copyright (c) 2017 projectchrono.org\n")






gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisCollisionType(chassis_CollisionType)
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
gator.SetTireType(tire_model)
gator.SetTireStepSize(tire_step_size)
gator.SetDriveline8WD(True)
gator.SetEngineType(veh.EngineModelType_SIMPLE)
gator.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
gator.SetTireCollisionType(tire_CollisionType)
gator.Initialize()

gator.SetChassisVisualizationType(veh.VisualizationType_MESH)
gator.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetWheelVisualizationType(veh.VisualizationType_MESH)
gator.SetTireVisualizationType(veh.VisualizationType_MESH)

gator.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


terrain = veh.RigidTerrain(gator.GetSystem())
if (gator.GetContactMethod() == chrono.ChContactMethod_SMC):
    patch_mat = chrono.ChContactMaterialSMC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
elif (gator.GetContactMethod() == chrono.ChContactMethod_NSC):
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch_mat.SetDampingF(0)
    patch_mat.SetDampingT(0)
patch = terrain.AddPatch(patch_mat, 
                         chrono.CSYSNORM, 
                         terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Gator')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
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






print( "VEHICLE MASS: ",  gator.GetVehicle().GetMass())


render_steps = m.ceil(render_step_size / step_size)


step_number = 0
render_frame = 0

gator.GetVehicle().EnableRealtime(True)

while vis.Run() :
    time = gator.GetSystem().GetChTime()

    
    if (time >= tend):
        break

    
    if (step_number % render_steps == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1
    
    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    gator.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    gator.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1