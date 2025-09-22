import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math
import os




print("PyChrono CityBus Simulation")


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
output_dir = "CityBus_output/"
if not os.path.exists(output_dir):
    os.mkdir(output_dir)


system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))




terrain = veh.RigidTerrain(system)
patch_material = chrono.ChMaterialSurfaceSMC()
patch_material.SetFriction(0.9)
patch_material.SetRestitution(0.01)


terrain_mesh = chrono.ChTriangleMeshConnected()
terrain_mesh.LoadWavefrontMesh(chrono.GetChronoDataPath() + "vehicle/terrain/meshes/flat_terrain.obj", False, True)
terrain_mesh.Transform(chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33D(1))
terrain_patch = terrain.AddPatch(patch_material, terrain_mesh, 0.01)


terrain_patch.SetTexture(chrono.GetChronoDataPath() + "vehicle/terrain/textures/tile4.jpg", 200, 200)
terrain.Initialize()





vehicle_file = chrono.GetChronoDataPath() + "vehicle/citybus/vehicle/CityBus_Vehicle.json"
powertrain_file = chrono.GetChronoDataPath() + "vehicle/citybus/powertrain/CityBus_SimpleMapPowertrain.json"
tire_file = chrono.GetChronoDataPath() + "vehicle/citybus/tire/CityBus_TMeasyTire.json"


citybus = veh.WheeledVehicle(system, vehicle_file)
citybus.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
citybus.SetChassisVisualizationType(veh.VisualizationType_MESH)
citybus.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
citybus.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
citybus.SetWheelVisualizationType(veh.VisualizationType_MESH)


powertrain = veh.ReadPowertrainJSON(powertrain_file)
citybus.InitializePowertrain(powertrain)


for axle in range(citybus.GetNumberAxles()):
    for side in range(2):
        wheel = citybus.GetWheel(axle, side)
        tire = veh.ReadTireJSON(tire_file)
        citybus.InitializeTire(tire, wheel, veh.VisualizationType_MESH)





driver = veh.ChIrrGuiDriver(citybus.GetVehicle())
driver.SetSteeringDelta(0.04)
driver.SetThrottleDelta(0.2)
driver.SetBrakingDelta(0.5)
driver.Initialize()





vis = veh.ChWheeledVehicleIrrApp(citybus.GetVehicle(), "CityBus Simulation", irr.dimension2du(1280, 720))
vis.AddTypicalLights()
vis.AddLogo(chrono.GetChronoDataPath() + "logo_pychrono_alpha.png")
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(-10, 5, 0), chrono.ChVectorD(0, 0, 0))
vis.SetChaseCamera(chrono.ChVectorD(5, 2, 0), 6.0, 0.5)
vis.SetTimestep(5e-3)
vis.AttachVehicle(citybus.GetVehicle())
vis.Initialize()




step_size = 5e-3  
realtime_timer = chrono.ChRealtimeStepTimer()
render_step_size = 1.0 / 50  
render_timer = 0.0




while vis.GetDevice().run():
    
    if system.GetChTime() >= render_timer:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_timer += render_step_size
    
    
    driver_inputs = driver.GetInputs()
    
    
    terrain.Synchronize(system.GetChTime())
    citybus.Synchronize(system.GetChTime(), driver_inputs, terrain)
    vis.Synchronize(driver_inputs, system.GetChTime())
    
    
    system.DoStepDynamics(step_size)
    
    
    speed_kmh = citybus.GetVehicle().GetVehicleSpeed() * 3.6
    print(f"Time: {system.GetChTime():.3f}s, Speed: {speed_kmh:.1f} km/h")
    
    
    realtime_timer.Spin(step_size)


del vis
del citybus
del terrain