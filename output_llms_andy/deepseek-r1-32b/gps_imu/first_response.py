importchrono
import numpy as np
from chrono import ChSystem, ChIrrApp, ChIrrGui, ChIrrDriver, ChIrrVehicle, ChIrrTerrain
from chrono.vehicle import HMMWV

# Initialize the Chrono system
system = ChSystem()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravity

# Create the Irrlicht application
app = ChIrrApp(system, "HMMWV Simulation", ChIrrGui())
app.SetWindowSize(1024, 768)
app.SetWindowTitle("HMMWV Simulation")
app.AddTypicalLights()
app.AddTypicalLogo()

# Create the HMMWV vehicle
vehicle = HMMWV(system, "HMMWV", True)  # Enable visualization
vehicle.SetChassisMass(2000)  # Set chassis mass in kg
vehicle.Initialize(chrono.ChVectorD(0, 0, 1), chrono.ChQuaternionD(1, 0, 0, 0), 0, 0, 0)

# Create terrain
terrain = ChIrrTerrain()
terrain.SetSize(100, 100)  # Terrain size in meters
terrain.SetHeight(0)  # Flat terrain
terrain.SetTexture("grass.jpg")  # Optional texture
system.Add(terrain)

# Add IMU and GPS sensors to chassis
chassis = vehicle.GetChassis()
imu = chassis.AddIMUSensor()
gps = chassis.AddGPSSensor()

# Initialize sensor data containers
imu_data = []
gps_data = []

# Create driver inputs
driver = ChIrrDriver()
driver.Initialize(vehicle.GetDriverInputs())

# Simulation parameters
sim_time = 10.0  # Total simulation time in seconds
timestep = 0.01  # Time step in seconds

# Main simulation loop
time = 0.0
while time < sim_time:
    # Update vehicle and driver
    vehicle.Update(timestep)
    driver.Update(timestep)
    
    # Update sensors
    imu_data.append({
        'time': time,
        'acceleration': imu.GetAcceleration(),
        'angular_velocity': imu.GetAngularVelocity(),
        'orientation': imu.GetOrientation()
    })
    
    gps_data.append({
        'time': time,
        'position': gps.GetPosition(),
        'velocity': gps.GetVelocity()
    })
    
    # Advance simulation
    system.DoStepDynamics(timestep)
    
    # Update visualization
    app.BeginScene()
    app.DrawAll()
    app.EndScene()
    
    # Print vehicle mass
    print(f"Time: {time:.2f}s | Vehicle Mass: {vehicle.GetChassis().GetMass():.2f} kg")
    
    time += timestep

# Cleanup
app.End()