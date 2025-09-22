import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math as m
import os
import errno


step_size = 2e-3

throttle_value = 0
steering_value = 0
braking_value = 0.0
my_map = veh.GetDataDir() + 'terrain/nurnberg.txt'


hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
hmmwv.SetChassisFixed(False);
hmmwv.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(-55, 1.5, -15), chrono.ChQuaterniond(1, 0, 0, 0)))
hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
hmmwv.SetDriveType(veh.DrivelineTypeWV_RWD)
hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(step_size)
hmmwv.Initialize()

hmmwv.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_NONE)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

hmmwv.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


minfo = chrono.ChContactMaterialData()
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = veh.RigidTerrain(hmmwv.GetSystem(), patch_mat, chrono.ChVector3d(200, 200, 1.0), minfo)
patch.SetColor(chrono.ChColor(0.8, 0.8, 1.0))
patch.Initialize()

terrain = veh.RigidTerrain(hmmwv.GetSystem(), patch_mat, chrono.ChVector3d(100, 12.0, 0.5), minfo)


vis_type = veh.VisualizationType_PRIMITIVES

if (vis_type == veh.VisualizationType_PRIMITIVES):
    terrain.SetVisualizationType(veh.VisualizationType_PRIMITIVES)

elif (vis_type == veh.VisualizationType_MESH):
    
    res_x = 1
    res_y = 1
    terrain.SetMesh(veh.MeshType_TRIPATCH, res_x, res_y)

    
    texture = chrono.GetChronoDataFile('terrain/perturb.png')
    terrain.AddTexture(chrono.GetChronoDataFile('terrain/asphalt.jpg'), res_x, res_y, 1, 0)
    terrain.AddTexture(texture, 5, 15, 3, 1)

elif (vis_type == veh.VisualizationType_HEIGHTCOLOR):
    
    terrain.SetHmin(0)
    terrain.SetHmax(2)

terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
vis.SetChaseCameraState(veh.ChChaseCamera.Track)
vis.SetChaseCameraPosition(chrono.ChVector3d(-10, 0, 2))
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(hmmwv.GetVehicle())


driver = veh.ChInteractiveDriverIRR(vis)


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()


hmmwv.GetVehicle().EnableRealtime(True)

while vis.Run() :
    time = hmmwv.GetSystem().GetChTime()

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    hmmwv.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    hmmwv.Advance(step_size)
    vis.Advance(step_size)