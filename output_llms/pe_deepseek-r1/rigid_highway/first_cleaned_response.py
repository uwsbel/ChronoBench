import os
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr




system = chrono.ChSystemSMC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)





hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.ChassisCollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(
    chrono.ChVector3d(0, 0, 0.5),  
    chrono.Q_from_AngZ(0)           
))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(0.001)
hmmwv.Initialize()


hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)


vehicle = hmmwv.GetVehicle()
tires = [hmmwv.GetTire(i) for i in range(4)]





terrain = veh.RigidTerrain(system)


terrain_material = chrono.ChContactMaterialNSC()
terrain_material.SetFriction(0.9)
terrain_material.SetRestitution(0.01)


mesh_path = os.path.join(chrono.GetChronoDataPath(), "models", "highway")
collision_mesh = os.path.join(mesh_path, "Highway_col.obj")
visual_mesh = os.path.join(mesh_path, "Highway_vis.obj")

patch = terrain.AddPatch(
    terrain_material,
    chrono.ChCoordsysd(),  
    collision_mesh,
    visual_mesh,
    0.01  
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 100, 100)
terrain.Initialize()




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("HMMWV on Custom Terrain")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(6, 6, 2), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()


vis.AddTypicalLights()
hmmwv.GetVehicle().AddVisualSystem(vis)




driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)
driver.Initialize()




step_size = 0.002
frame_rate = 50  
frame_step = 1.0 / frame_rate
time = 0
frame_count = 0

while vis.Run():
    time = system.GetChTime()
    
    
    driver_inputs = driver.GetInputs()
    driver.Synchronize(time)
    
    
    hmmwv.Synchronize(time, driver_inputs, terrain)
    
    
    driver.Advance(step_size)
    hmmwv.Advance(step_size)
    terrain.Advance(step_size)
    system.DoStepDynamics(step_size)
    
    
    if time >= frame_count * frame_step:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        frame_count += 1




vis.GetDevice().closeDevice()