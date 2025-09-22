import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import os
import math




chrono.SetChronoDataPath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "data/"))


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))





terrain_vis_mesh = "Highway_vis.obj"
terrain_col_mesh = "Highway_col.obj"


terrain = veh.RigidTerrain(system)


terrain_file_vis = chrono.GetChronoDataFile(terrain_vis_mesh)
terrain_mesh_vis = chrono.ChTriangleMeshConnected()
terrain_mesh_vis.LoadWavefrontMesh(terrain_file_vis, False, True)
terrain_mesh_vis.Transform(chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33D(1))


terrain_file_col = chrono.GetChronoDataFile(terrain_col_mesh)
terrain_mesh_col = chrono.ChTriangleMeshConnected()
terrain_mesh_col.LoadWavefrontMesh(terrain_file_col, False, True)
terrain_mesh_col.Transform(chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33D(1))


patch_material = chrono.ChMaterialSurfaceNSC()
patch_material.SetFriction(0.9)
patch_material.SetRestitution(0.01)

terrain_patch = terrain.AddPatch(patch_material, 
                                 terrain_mesh_col, 
                                 chrono.ChVectorD(0, 0, 0),
                                 chrono.ChQuaternionD(1, 0, 0, 0))

terrain_patch.SetContactFrictionCoefficient(0.8)
terrain_patch.SetContactRestitutionCoefficient(0.01)
terrain_patch.SetContactMaterialProperties(2e7, 0.3)
terrain_patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain_patch.SetTexture(chrono.GetChronoDataFile("textures/dirt.jpg"), 200, 200)


terrain_patch.AddVisualizationMesh(terrain_file_vis,
                                  chrono.ChColor(0.8, 0.8, 0.5),
                                  chrono.GetChronoDataFile("textures/dirt.jpg"))





vehicle_pos = chrono.ChVectorD(0, 0.5, 0)
vehicle_rot = chrono.ChQuaternionD(1, 0, 0, 0)


hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.ChassisCollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysD(vehicle_pos, vehicle_rot))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(1e-3)
hmmwv.SetMaxMotorVoltageInVolts(12)
hmmwv.Initialize()


hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)









vis = chronoirr.ChIrrApp(system, "HMMWV on Terrain", chronoirr.dimension2du(1280, 720))
vis.AddTypicalLogo()
vis.AddTypicalSky()
vis.AddTypicalLights(chronoirr.vector3df(30, 100, 30), chronoirr.vector3df(-30, 100, 30), 250, 130)
vis.AddTypicalCamera(chronoirr.vector3df(0, 1.5, -6), chronoirr.vector3df(0, 0, 0))


vis.SetSymbolscale(0.5)
vis.SetShowInfos(True)


vis.AssetBindAll()
vis.AssetUpdateAll()
vis.SetTimestep(1e-3)





driver = veh.ChIrrGuiDriver(vis)


driver.SetSteeringDelta(0.05)
driver.SetThrottleDelta(0.05)
driver.SetBrakingDelta(0.1)
driver.SetMaxSteeringAngle(0.3)
driver.Initialize()


controller = veh.ChDriver(hmmwv.GetVehicle())
hmmwv.GetVehicle().RegisterDriver(controller)




frame_skip = 5  
runtime_seconds = 300  
frame_count = 0


chrono_time = 0
wall_clock = chrono.ChRealtimeClock()

while vis.GetDevice().run() and chrono_time < runtime_seconds:
    
    vis.BeginScene()
    vis.Render()
    
    
    driver_inputs = driver.GetInputs()
    
    
    time = system.GetChTime()
    controller.SetThrottle(driver_inputs.m_throttle)
    controller.SetSteering(driver_inputs.m_steering)
    controller.SetBraking(driver_inputs.m_braking)
    
    
    hmmwv.Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time)
    
    
    system.DoStepDynamics(1e-3)
    chrono_time = system.GetChTime()
    
    
    frame_count += 1
    if frame_count % frame_skip == 0:
        vis.Render()
        
        
        speed_kmh = hmmwv.GetVehicle().GetSpeed() * 3.6
        vis.GetDevice().setWindowCaption(f"HMMWV Simulation - Speed: {speed_kmh:.1f} km/h")
        
    
    vis.EndScene()
    
    
    wall_clock.Spin(chrono_time)


vis.GetDevice().drop()