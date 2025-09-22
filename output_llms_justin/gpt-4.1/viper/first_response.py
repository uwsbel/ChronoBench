import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math

# 1. Initialize PyChrono system
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(veh.GetDataPath())

system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

# 2. Create rigid terrain (ground)
terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
    length=10.0, width=10.0
)
patch.SetContactFrictionCoefficient(0.9)
patch.SetRestitutionCoefficient(0.01)
patch.SetMaterialSurface(veh.CreateMaterial(veh.ContactMethod_NSC))
patch.SetColor(chrono.ChColor(0.4, 0.6, 0.4))
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 10, 10)
terrain.Initialize()

# 3. Create the Viper rover
init_pos = chrono.ChVectorD(0, 0, 0.5)
init_rot = chrono.Q_from_AngZ(0)
viper = veh.Viper(system)
viper.SetChassisFixed(False)
viper.SetInitPosition(chrono.ChCoordsysD(init_pos, init_rot))
viper.Initialize()

# 4. Create a driver for the Viper
driver = veh.ViperSpeedDriver()
driver.SetThrottle(0.5)  # 50% throttle
driver.SetSteering(0.0)  # Start with zero steering
viper.SetDriver(driver)

# 5. Visualization with Irrlicht
vis = veh.ChWheeledVehicleIrrApp(viper.GetVehicle(), 'PyChrono Viper Rover', irr.dimension2du(1024,768))
vis.AddTypicalLights()
vis.AddTypicalSky()
vis.AddTypicalLogo()
vis.AddTypicalCamera(chrono.ChVectorD(2, 2, 1.5), chrono.ChVectorD(0, 0, 0.5))
vis.SetChaseCamera(chrono.ChVectorD(0, 0, 1.0), 2.0, 0.5)
vis.SetHUDLocation(500, 20)
vis.AssetBindAll()
vis.AssetUpdateAll()

# 6. Simulation loop parameters
step_size = 1e-3
end_time = 10.0
steering_time = 3.0     # seconds over which to ramp steering
max_steering = 0.5      # maximum steering input ([-1,1])

# 7. Simulation loop
time = 0.0
while vis.Run() and time < end_time:
    # Gradually increase steering input over steering_time seconds
    if time < steering_time:
        steering = max_steering * (time / steering_time)
    else:
        steering = max_steering
    driver.SetSteering(steering)
    
    # Update modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    viper.Synchronize(time, driver.GetInputs(), terrain)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    viper.Advance(step_size)
    system.DoStepDynamics(step_size)
    time += step_size

vis.GetDevice().closeDevice()