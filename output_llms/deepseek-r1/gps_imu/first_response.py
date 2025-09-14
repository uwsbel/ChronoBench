import os
import math
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import pychrono.irrlicht as irr

# Initialize Chrono system
chrono.SetChronoDataPath(os.path.join(os.getenv("CHRONO_DATA_DIR"), "vehicle/"))
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

# Create and initialize HMMWV vehicle
vehicle = veh.HMMWV_Full(system)
vehicle.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT))
vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_NONE)

# Create rigid terrain
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1), 100, 100)
patch.SetTexture(veh.GetDataFile("terrain/texture/tile4.jpg"), 200, 200)
terrain.Initialize()

# Create Irrlicht interface
app = irr.ChIrrApp(system, "HMMWV Simulation", irr.dimension2du(800, 600))
app.AddTypicalLights()
app.AddTypicalCamera(irr.vector3df(0, 0, 1))
app.AssetBindAll()
app.AssetUpdateAll()

# Create sensor manager
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVectorD(0, 0, 100), chrono.ChColor(1, 1, 1), 5000)

# Add IMU sensor to chassis
imu_offset = chrono.ChVectorD(0, 0, 0.5)
imu_noise = sens.NoiseNormal(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0.001, 0.001, 0.001))
imu = sens.ChIMUSensor(vehicle.GetChassisBody(), 100, chrono.ChFrameD(imu_offset), manager)
imu.SetNoiseModel(imu_noise)
imu.PushFilter(sens.SensorFilterIIR())

# Add GPS sensor to chassis
gps_offset = chrono.ChVectorD(0, 0, 0.5)
gps_noise = sens.NoiseNormal(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0.1, 0.1, 0.1))
gps = sens.ChGPSSensor(vehicle.GetChassisBody(), 10, chrono.ChFrameD(gps_offset), 
                      chrono.ChVectorD(-100, -100, -100), chrono.ChVectorD(100, 100, 100), manager)
gps.SetNoiseModel(gps_noise)
gps.PushFilter(sens.SensorFilterIIR())

# Create interactive driver
driver = veh.ChInteractiveDriverIRR(app)
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.02)
driver.Initialize()

# Simulation loop
time_step = 0.002
while app.GetDevice().run():
    time = system.GetChTime()
    
    # Update sensor manager
    manager.Update()
    
    # Get latest sensor data
    if imu.GetMostRecentFrame():
        imu_data = imu.GetMostRecentFrame().GetIMUData()
    if gps.GetMostRecentFrame():
        gps_data = gps.GetMostRecentFrame().GetGPSData()
    
    # Process driver inputs
    driver.Synchronize(time)
    vehicle.Synchronize(time, driver.GetInputs(), terrain)
    terrain.Synchronize(time)
    app.Synchronize("", driver.GetInputs())
    
    # Advance simulation
    driver.Advance(time_step)
    terrain.Advance(time_step)
    vehicle.Advance(time_step)
    app.Advance(time_step)
    
    # Output vehicle mass
    if math.fmod(time, 1.0) < time_step:
        print(f"Vehicle mass: {vehicle.GetVehicleMass()} kg")
    
    # Render scene
    app.BeginScene(True, True, irr.SColor(255, 140, 161, 192))
    app.DrawAll()
    app.EndScene()