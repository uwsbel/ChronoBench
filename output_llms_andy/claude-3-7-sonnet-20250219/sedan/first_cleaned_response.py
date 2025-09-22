import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import os
import math






step_size = 1e-3


simulation_time = 1000


vehicle_file = "vehicle/BMW_E90.json"
powertrain_file = "powertrain/BMW_E90_SimplePowertrain.json"
tire_file = "tire/BMW_E90_TMeasy.json"


init_pos = chrono.ChVectorD(0, 0, 0.5)
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)


chrono_data_dir = "/path/to/chrono/data/"  






sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))


sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
sys.SetSolverMaxIterations(150)
sys.SetMaxPenetrationRecoverySpeed(4.0)
sys.SetMinBounceSpeed(2.0)
sys.SetSolverForceTolerance(0)





terrain = veh.RigidTerrain(sys)


patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)


terrain_length = 200.0  
terrain_width = 200.0   

patch = terrain.AddPatch(patch_mat, 
                         chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
                         terrain_length, terrain_width)


patch.SetTexture(chrono_data_dir + "terrain/textures/tile4.jpg", 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))


logo = patch.AddPlot(chrono_data_dir + "vehicle/logo/bmw_logo.png", 
                    chrono.ChVectorD(0, 0, 0.02), 
                    10, 10)


terrain.Initialize()






vehicle = veh.WheeledVehicle(sys, vehicle_file)
vehicle.Initialize(chrono.ChCoordsysD(init_pos, init_rot))
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)


powertrain = veh.SimplePowertrain(powertrain_file)
vehicle.InitializePowertrain(powertrain)


for axle in vehicle.GetAxles():
    tireL = veh.TMeasyTire(tire_file)
    tireR = veh.TMeasyTire(tire_file)
    
    vehicle.InitializeTire(tireL, axle.m_wheels[0], veh.VisualizationType_MESH)
    vehicle.InitializeTire(tireR, axle.m_wheels[1], veh.VisualizationType_MESH)


vehicle.SetCollide(veh.CollisionType_ALL)





driver = veh.ChIrrGuiDriver(vehicle.GetVehicle())
driver.SetSteeringDelta(0.05)
driver.SetThrottleDelta(0.05)
driver.SetBrakingDelta(0.1)
driver.Initialize()





vis = irr.ChIrrApp(sys, "BMW E90 Sedan Simulation", irr.dimension2du(1280, 720))
vis.AddTypicalLights()
vis.AddSkyBox()
vis.AddLogo(chrono_data_dir + "logo/chronoengine_logo.png")


vis.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 1.75), 6.0, 0.5)


vis.SetSymbolscale(0.5)
vis.SetShowInfos(True)


vis.Initialize()
vis.AddShadowAll()
vis.SetTimestep(step_size)






time = 0
render_steps = int(0.01/step_size)

while vis.GetDevice().run():
    
    if sys.GetChTime() >= time:
        vis.BeginScene(True, True, irr.SColor(255, 140, 161, 192))
        vis.DrawAll()
        
        
        driver_inputs = driver.GetInputs()
        vehicle_speed = vehicle.GetVehicle().GetVehicleSpeed()
        
        info = "Speed: {:.1f} km/h    Throttle: {:.2f}    Steering: {:.2f}    Braking: {:.2f}".format(
            vehicle_speed * 3.6, driver_inputs.m_throttle, driver_inputs.m_steering, driver_inputs.m_braking)
        vis.GetDevice().getGUIEnvironment().addStaticText(irr.core.stringw(info),
                                                         irr.core.recti(10, 10, 500, 30),
                                                         False)
        
        vis.EndScene()
        time += 0.01
    
    
    driver.Synchronize(time)
    
    
    vehicle.Synchronize(time, driver_inputs, terrain)
    
    
    terrain.Synchronize(time)
    sys.DoStepDynamics(step_size)
    
    
    vis.Synchronize("", driver_inputs)
    
    
    if time > simulation_time:
        break

vis.GetDevice().closeDevice()