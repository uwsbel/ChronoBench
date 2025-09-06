import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import numpy as np
import time

# ==============================================================================
# Simulation parameters
# ==============================================================================

# Simulation step size
step_size = 1e-3

# Time interval for sensor updates
sensor_update_interval = 0.01  # 100Hz

# Simulation end time
t_end = 10.0

# ==============================================================================
# Create the systems and simulation environment
# ==============================================================================

# Create the Chrono physical system
system = chrono.ChSystemNSC()

# Set gravity (default is already 9.81 m/s² downward)
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

# Create the vehicle system
vehicle = veh.HMMWV()
vehicle.Initialize(system, veh.VehicleSide.FRONT)

# Set initial vehicle location and orientation
vehicle.GetChassis().SetPos(chrono.ChVectorD(0, 0, 0.5))
vehicle.GetChassis().SetRot(chrono.ChQuaternionD(1, 0, 0, 0))

# Create the terrain
terrain = veh.RigidTerrain(system)
patch_size = 100
patch = veh.RigidTerrain.Patch(patch_size, patch_size, 0, patch_size/2, 0, patch_size/2)
patch.SetTexture(veh.RigidTerrain.Texture("terrain/textures/tile4.jpg"))
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.AddPatch(patch)
terrain.Initialize()

# Create the driver system
driver = veh.ChDataDriver()
driver.SetSteeringFunction(chrono.ChFunction_Const(0))  # Constant steering (0 = straight)
driver.SetThrottleFunction(chrono.ChFunction_Const(0.3))  # Constant throttle
driver.Initialize()

# ==============================================================================
# Create the visual system
# ==============================================================================

# Create the Irrlicht application
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("HMMWV Simulation with Sensors")
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(8, 0, 2), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()
vis.AddShadowAll()

# Customize contact visualization
vis.SetSymbolScale(0.01)
vis.SetContactDrawingMode(chronoirr.ChVisualSystemIrrlicht.ContactDrawingMode.CONTACT_FORCES)
vis.SetContactForceDrawingMode(chronoirr.ChVisualSystemIrrlicht.ContactForceDrawingMode.SPRINGS)

# ==============================================================================
# Create and initialize sensors
# ==============================================================================

class IMUSensor:
    def __init__(self, chassis):
        self.chassis = chassis
        self.acceleration = chrono.ChVectorD(0, 0, 0)
        self.angular_velocity = chrono.ChVectorD(0, 0, 0)
        self.orientation = chrono.ChQuaternionD(1, 0, 0, 0)
        self.last_time = 0

    def update(self, time):
        if time - self.last_time >= sensor_update_interval:
            # Get current state from chassis
            self.acceleration = self.chassis.GetPos_dt2()
            self.angular_velocity = self.chassis.GetWvel_loc()
            self.orientation = self.chassis.GetRot()

            # Add some noise to simulate real sensor data
            noise = 0.01
            self.acceleration.x += np.random.uniform(-noise, noise)
            self.acceleration.y += np.random.uniform(-noise, noise)
            self.acceleration.z += np.random.uniform(-noise, noise)

            self.angular_velocity.x += np.random.uniform(-noise, noise)
            self.angular_velocity.y += np.random.uniform(-noise, noise)
            self.angular_velocity.z += np.random.uniform(-noise, noise)

            self.last_time = time

            # Print sensor data (for demonstration)
            print(f"IMU - Acceleration: {self.acceleration}")
            print(f"IMU - Angular Velocity: {self.angular_velocity}")
            print(f"IMU - Orientation: {self.orientation}")

class GPSSensor:
    def __init__(self, chassis):
        self.chassis = chassis
        self.position = chrono.ChVectorD(0, 0, 0)
        self.velocity = chrono.ChVectorD(0, 0, 0)
        self.last_time = 0

    def update(self, time):
        if time - self.last_time >= sensor_update_interval:
            # Get current state from chassis
            self.position = self.chassis.GetPos()
            self.velocity = self.chassis.GetPos_dt()

            # Add some noise to simulate real sensor data
            noise = 0.1
            self.position.x += np.random.uniform(-noise, noise)
            self.position.y += np.random.uniform(-noise, noise)
            self.position.z += np.random.uniform(-noise, noise)

            self.velocity.x += np.random.uniform(-noise, noise)
            self.velocity.y += np.random.uniform(-noise, noise)
            self.velocity.z += np.random.uniform(-noise, noise)

            self.last_time = time

            # Print sensor data (for demonstration)
            print(f"GPS - Position: {self.position}")
            print(f"GPS - Velocity: {self.velocity}")

# Create sensor instances
imu = IMUSensor(vehicle.GetChassis())
gps = GPSSensor(vehicle.GetChassis())

# ==============================================================================
# Main simulation loop
# ==============================================================================

# Simulation time
time = 0
frame_count = 0
last_output_time = 0

while time < t_end:
    # Get current simulation time
    time = system.GetChTime()

    # Update driver inputs
    driver.SetTime(time)
    driver.Synchronize(time)

    # Update vehicle
    vehicle.Synchronize(time, driver.GetSteering(), driver.GetThrottle(), driver.GetBraking())

    # Update sensors
    imu.update(time)
    gps.update(time)

    # Update visualization
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Output vehicle mass periodically
    if time - last_output_time >= 1.0:
        chassis_mass = vehicle.GetChassis().GetMass()
        print(f"Time: {time:.2f}s - Vehicle mass: {chassis_mass:.2f} kg")
        last_output_time = time

    # Advance simulation by one step
    system.DoStepDynamics(step_size)

    # Increment frame counter
    frame_count += 1

# ==============================================================================
# Simulation complete
# ==============================================================================

print("Simulation complete")
print(f"Total simulation time: {time:.2f} seconds")
print(f"Total frames rendered: {frame_count}")