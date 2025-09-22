import pychrono.core as chrono  
import pychrono.irrlicht as irr  
import pychrono.vehicle as veh  
import pychrono.sensor as sens  
import math  


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(0, 0, 0.5)  
initRot = chrono.ChQuaterniond(1, 0, 0, 0)  


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE



terrainHeight = 0  
terrainLength = 100.0  
terrainWidth = 100.0  


trackPoint = chrono.ChVector3d(0.0, 0.0, 1.75)


contact_method = chrono.ChContactMethod_NSC
contact_vis = False


step_size = 1e-3  
tire_step_size = step_size  


render_step_size = 1.0 / 50  


vehicle = veh.FEDA()  
vehicle.SetContactMethod(contact_method)  
vehicle.SetTrackType(veh.FEDA_SCG)  
vehicle.SetTireType(veh.TireType_BlackMichelin)  
vehicle.SetEngineType(veh.EngineModelType_SHAFTS)  
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)  

vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))  
vehicle.Initialize()  

vehicle.SetChassisVisualizationType(vis_type)  
vehicle.SetSuspensionVisualizationType(vis_type)  
vehicle.SetSteeringVisualizationType(vis_type)  
vehicle.SetWheelVisualizationType(vis_type)  
vehicle.SetTireVisualizationType(vis_type)  

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  


patch_mat = chrono.ChContactMaterialNSC()  
patch_mat.SetFriction(0.9)  
patch_mat.SetRestitution(0.01)  
terrain = veh.RigidTerrain(vehicle.GetSystem())  
patch = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
    terrainLength, terrainWidth)

patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))  
terrain.Initialize()  


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()  
vis.SetWindowTitle('FEDA vehicle')  
vis.SetWindowSize(1280, 720)  
vis.SetChaseCamera(trackPoint, 6.0, 0.5)  
vis.Initialize()  
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  
vis.AddLightDirectional()  
vis.AddSkyBox()  
vis.AttachVehicle(vehicle.GetVehicle())  


driver = veh.ChInteractiveDriverIRR(vis)  
driver.SetSteeringDelta(0.02)  
driver.SetThrottleDelta(0.02)  
driver.SetBrakingDelta(0.06)  
driver.Initialize()  


print("Vehicle mass:     ", vehicle.GetVehicle().GetMass())
print("Driveline type:   ", vehicle.GetVehicle().GetDriveline().GetTemplateName())
print("Brake type:       ", vehicle.GetVehicle().GetBrake(1, veh.LEFT).GetTemplateName())
print("Tire type:        ", vehicle.GetTire(1, veh.LEFT).GetTemplateName())
print("Tire step size:   ", tire_step_size)


vehicle.GetVehicle().SetStep(step_size)


time = 0
Step = 0


while vis.Run():
    time = vehicle.GetSystem().GetChTime()  

    
    if (Step % math.ceil(render_step_size / step_size) == 0):
        vis.BeginScene()  
        vis.Render()  
        vis.EndScene()  

    
    driver_inputs = driver.GetInputs()  
    driver.Synchronize(time)  
    terrain.Synchronize(time)  
    vehicle.Synchronize(time, driver_inputs, terrain)  

    
    driver.Advance(step_size)  
    terrain.Advance(step_size)  
    vehicle.Advance(step_size)  

    
    Step = Step + 1