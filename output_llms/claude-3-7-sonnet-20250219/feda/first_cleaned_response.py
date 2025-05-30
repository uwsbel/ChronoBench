import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import os
import math





chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "data/"))


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
system.SetSolverMaxIterations(150)
system.SetMaxPenetrationRecoverySpeed(4.0)




terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)


terrain_file = chrono.GetChronoDataFile("terrain/height_maps/flat.bmp")
terrain_size = chrono.ChVectorD(200, 1, 200)
terrain_center = chrono.ChVectorD(0, 0, 0)

patch = terrain.AddPatch(patch_mat, 
                        terrain_center,
                        chrono.ChVectorD(0, 1, 0),  
                        terrain_size.x, 
                        terrain_size.z)


patch.SetTexture(chrono.GetChronoDataFile("textures/grass.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 1.0))
terrain.Initialize()





vehicle_file = os.path.join(os.path.dirname(__file__), "data/vehicle/feda.json")
powertrain_file = os.path.join(os.path.dirname(__file__), "data/powertrain/simple_powertrain.json")
tire_file = os.path.join(os.path.dirname(__file__), "data/tire/rigid_tire.json")


init_loc = chrono.ChVectorD(0, 0.5, 0)
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)  


vehicle = veh.WheeledVehicle(system, vehicle_file)
vehicle.Initialize(chrono.ChCoordsysD(init_loc, init_rot))
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)


powertrain = veh.SimplePowertrain(powertrain_file)
vehicle.InitializePowertrain(powertrain)


for axle in vehicle.GetAxles():
    tireL = veh.RigidTire(tire_file)
    tireR = veh.RigidTire(tire_file)
    vehicle.InitializeTire(tireL, axle.m_wheels[0], veh.VisualizationType_MESH)
    vehicle.InitializeTire(tireR, axle.m_wheels[1], veh.VisualizationType_MESH)




app = veh.ChWheeledVehicleIrrApp(vehicle, "FEDA Vehicle Simulation")

app.SetSkyBox()
app.AddTypicalLights()
app.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
app.SetChaseCamera(chrono.ChVectorD(0, 1.5, -6.0), 6.0, 0.5)
app.SetTimestep(1e-3)




driver = veh.ChIrrGuiDriver(app)
driver.SetSteeringDelta(0.05)
driver.SetThrottleDelta(0.05)
driver.SetBrakingDelta(0.1)
driver.Initialize()


vehicle.SetDriver(driver)




app.AssetBindAll()
app.AssetUpdateAll()


step_size = 1e-3  
render_step = 1.0 / 50  
render_next = 0.0




while app.GetDevice().run():
    
    app.BeginScene(True, True, chrono.ChColor(0.1, 0.1, 0.1))
    app.DrawAll()
    app.EndScene()
    
    
    driver_inputs = driver.GetInputs()
    
    
    time = vehicle.GetSystem().GetChTime()
    
    
    vehicle.Advance(step_size)
    terrain.Advance(step_size)
    app.Advance(step_size)
    
    
    vehicle.Synchronize(time, driver_inputs, terrain)
    app.Synchronize(time, driver_inputs)
    
    
    if time >= render_next:
        app.Advance(render_step)
        render_next += render_step