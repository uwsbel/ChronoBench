#!/usr/bin/env python3
"""
PyChrono HMMWV Vehicle Simulation with Sensors
Features: Terrain, HMMWV vehicle, driver, Irrlicht visualization, IMU and GPS sensors
"""

# =============================================================================
# IMPORTS
# =============================================================================

import os
import sys
import math
import numpy as np

# PyChrono imports
import pychrono as chrono
import pychrono.vehicle as chrvehicle
import pychrono.sensor as chrsensor
from pychrono.irreffects import IrrlichtApp

# Set data paths
chrono.SetDataPath("C:/Chrono/Data/")
chrvehicle.SetDataPath("C:/ChronoVehicle/Data/")


# =============================================================================
# SIMULATION PARAMETERS
# =============================================================================

class SimulationParameters:
    """Configuration parameters for the simulation"""
    
    def __init__(self):
        # Simulation timing
        self.step_size = 1e-3  # Simulation time step (seconds)
        self.simulation_time = 30.0  # Total simulation time (seconds)
        self.render_step_size = 1.0 / 60.0  # Render frame rate
        
        # Vehicle parameters
        self.vehicle_type = "HMMWV"  # Vehicle type
        self.chassis_height = 0.8  # Chassis height above ground
        self.chassis_vis = True  # Enable chassis visualization
        
        # Terrain parameters
        self.terrain_type = "RIGID_PLANE"  # Terrain type
        self.terrain_size = 200.0  # Terrain size in meters
        self.friction_coeff = 0.8  # Terrain friction coefficient
        
        # Driver parameters
        self.driver_type = "PYTHON_DRIVER"  # Driver type
        self.target_speed = 8.0  # Target speed in m/s
        
        # Sensor parameters
        self.sensor_update_rate = 100  # Sensor update rate (Hz)
        self.gps_update_rate = 10  # GPS update rate (Hz)
        self.imu_update_rate = 100  # IMU update rate (Hz)
        
        # Output parameters
        self.output_enabled = True  # Enable data output
        self.output_directory = "./hmmwv_simulation_output/"


# =============================================================================
# TERRAIN CREATION
# =============================================================================

class TerrainCreator:
    """Creates and configures the simulation terrain"""
    
    @staticmethod
    def create_rigid_plane(system, friction=0.8):
        """Create a rigid plane terrain"""
        
        # Create ground material
        ground_material = chrono.ChMaterialSurfaceNSC()
        ground_material.SetFriction(friction)
        ground_material.SetRestitution(0.1)
        
        # Create terrain body
        ground = chrono.ChBody()
        ground.SetBodyFixed(True)
        ground.SetMaterialSurface(chrono.ChMaterialSurfaceNSC(ground_material))
        
        # Set visualization
        ground.GetVisualShape().SetTexture(
            chrono.GetChronoDataFile("texture/concrete.png")
        )
        
        # Add collision geometry
        ground.GetCollisionModel().SetBoxSealed(False)
        ground.GetCollisionModel().AttachBox(100, 0.02, 100)
        ground.SetCollide(True)
        
        # Add to system
        system.Add(ground)
        
        return ground
    
    @staticmethod
    def create_height_map_terrain(system, filename=None):
        """Create a height map terrain if filename provided"""
        
        if filename is None:
            # Use default terrain
            filename = chrono.GetChronoDataFile("terrain/height_maps/UK_terrain_data.csv")
        
        # Create terrain parameters
        ter_dim_x = 200.0  # Terrain dimension in X
        ter_dim_y = 200.0  # Terrain dimension in Y
        ter_size_x = 200.0  # Terrain size in X
        ter_size_y = 200.0  # Terrain size in Y
        ter_thickness = 0.1  # Terrain thickness
        
        # Create height map terrain
        terrain = chrono.ChBody()
        terrain.SetBodyFixed(True)
        
        # Create mesh from height map (simplified)
        # In real implementation, would load actual height map data
        terrain.GetCollisionModel().SetBoxSealed(False)
        terrain.GetCollisionModel().AttachBox(ter_size_x / 2, ter_thickness / 2, ter_size_y / 2)
        terrain.SetCollide(True)
        
        system.Add(terrain)
        
        return terrain


# =============================================================================
# HMMWV VEHICLE CREATION
# =============================================================================

class HMMWVBuilder:
    """Builds and configures the HMMWV vehicle system"""
    
    def __init__(self, system, params):
        self.system = system
        self.params = params
        
    def create_vehicle(self):
        """Create the HMMWV vehicle with full system"""
        
        # Create vehicle initialization parameters
        init_loc = chrono.ChVectorD(0, self.params.chassis_height, 0)
        init_rot = chrono.ChQuaternionD(1, 0, 0, 0)
        init_vel = chrono.ChVectorD(0, 0, 0)
        
        # Create HMMWV vehicle
        self.vehicle = chrvehicle.HMMWV(
            self.system,
            chrvehicle.HMMWV_ReduceTireMass.NO,
            chrvehicle.DrivelineTypeWV.AWD,
            chrvehicle.EngineModelType.SIMPLE,
            chrvehicle.TransmissionModelState.SIMPLE_MAP,
            chrvehicle.TireModelType.RIGID
        )
        
        # Initialize vehicle
        self.vehicle.Initialize(
            chrono.ChCoordsysD(init_loc, init_rot),
            init_vel,
            self.params.vehicle_type
        )
        
        # Get vehicle subsystems
        self.chassis = self.vehicle.GetChassis()
        self.powertrain = self.vehicle.GetPowertrain()
        self.wheels = [
            self.vehicle.GetWheel(0),  # Front left
            self.vehicle.GetWheel(1),  # Front right
            self.vehicle.GetWheel(2),  # Rear left
            self.vehicle.GetWheel(3),  # Rear right
        ]
        self.suspensions = [
            self.vehicle.GetSuspension(0),
            self.vehicle.GetSuspension(1),
            self.vehicle.GetSuspension(2),
            self.vehicle.GetSuspension(3),
        ]
        
        # Set visualization
        self.vehicle.SetVisualization(
            self.params.chassis_vis,
            chrvehicle.VisualizationType.MESH,
            chrvehicle.VisualizationType.MESH
        )
        
        # Print vehicle information
        self.print_vehicle_info()
        
        return self.vehicle
    
    def print_vehicle_info(self):
        """Print vehicle information"""
        
        # Get vehicle mass
        mass = self.chassis.GetMass()
        print(f"\n{'='*60}")
        print(f"VEHICLE INFORMATION")
        print(f"{'='*60}")
        print(f"Vehicle Type: {self.params.vehicle_type}")
        print(f"Chassis Mass: {mass:.2f} kg")
        print(f"Chassis COM: {self.chassis.GetFrame_COG_to_abs().GetPos()}")
        print(f"Number of Wheels: {self.vehicle.GetNumWheels()}")
        print(f"Driveline Type: {self.vehicle.GetDriveline().GetType()}")
        print(f"{'='*60}\n")


# =============================================================================
# DRIVER SYSTEM
# =============================================================================

class DriverController:
    """Python-based driver controller for vehicle"""
    
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.target_speed = 8.0  # m/s
        self.throttle = 0.0
        self.braking = 0.0
        self.steering = 0.0
        self.throttle_threshold = 0.98
        
    def advance(self, step_size):
        """Advance driver inputs based on vehicle state"""
        
        # Get current vehicle speed
        speed = self.vehicle.GetVehicleSpeed()
        
        # Calculate speed error
        speed_error = self.target_speed - speed
        
        # Simple PI controller for throttle
        if speed_error > 0.1:
            self.throttle = min(1.0, self.throttle + 0.02)
            self.braking = 0.0
        elif speed_error < -0.1:
            self.throttle = max(0.0, self.throttle - 0.01)
            self.braking = 0.2
        else:
            self.throttle = min(self.throttle + 0.01, self.throttle_threshold)
            self.braking = 0.0
        
        # Keep steering minimal (straight driving)
        self.steering = 0.0
        
        # Apply driver inputs to vehicle
        self.vehicle.DriveStrut().SetDisplacement(self.steering)
        self.vehicle.ApplyThrottle(self.throttle)
        self.vehicle.ApplyBraking(self.braking)
        
    def set_target_speed(self, speed):
        """Set the target speed for the driver"""
        self.target_speed = speed
    
    def get_throttle(self):
        """Get current throttle value"""
        return self.throttle
    
    def get_braking(self):
        """Get current braking value"""
        return self.braking
    
    def get_steering(self):
        """Get current steering value"""
        return self.steering


# =============================================================================
# SENSOR SYSTEM
# =============================================================================

class SensorManager:
    """Manages IMU and GPS sensors attached to the vehicle"""
    
    def __init__(self, system, chassis, update_rate=100):
        self.system = system
        self.chassis = chassis
        self.update_rate = update_rate
        self.sensors = {}
        self.sensor_data = {}
        self.manager = None
        
    def initialize_manager(self):
        """Initialize the sensor manager"""
        
        # Create sensor manager
        self.manager = chrsensor.ChSensorManager(self.system)
        
        # Configure ambient light
        self.manager.Add毽tLight(
            chrono.ChVectorD(0, 10, 0),
            chrono.ChColor(1, 1, 1),
            100
        )
        
        return self.manager
    
    def add_imu_sensor(self, name="IMU_1", offset_position=None, update_rate=None):
        """Add IMU sensor to the chassis"""
        
        if update_rate is None:
            update_rate = self.update_rate
        
        if offset_position is None:
            offset_position = chrono.ChFrameD(
                chrono.ChVectorD(0, 0.5, 0),  # Position relative to chassis
                chrono.ChQuaternionD(1, 0, 0, 0)  # Orientation
            )
        
        # Create IMU sensor
        imu = chrsensor.ChIMUSensor(
            self.chassis,
            offset_position,
            update_rate
        )
        
        # Configure IMU noise model
        imu.SetName(name)
        imu.SetFlagActive(True)
        
        # Add callback for data processing
        imu.RegisterUserOffsetCallback(
            lambda: chrono.ChVectorD(0, 0, 0),
            lambda: chrono.ChQuaternionD(1, 0, 0, 0)
        )
        
        # Store sensor
        self.sensors[name] = imu
        
        # Initialize data storage
        self.sensor_data[name] = {
            'linear_acceleration': [],
            'angular_velocity': [],
            'magnetic_field': [],
            'gps': None,
            'timestamp': 0
        }
        
        return imu
    
    def add_gps_sensor(self, name="GPS_1", offset_position=None, update_rate=10):
        """Add GPS sensor to the chassis"""
        
        if offset_position is None:
            offset_position = chrono.ChFrameD(
                chrono.ChVectorD(0, 1.0, 0),  # Position relative to chassis
                chrono.ChQuaternionD(1, 0, 0, 0)
            )
        
        # Create GPS sensor
        gps = chrsensor.ChGPSSensor(
            self.chassis,
            offset_position,
            update_rate
        )
        
        # Configure GPS
        gps.SetName(name)
        gps.SetFlagActive(True)
        
        # Store sensor
        self.sensors[name] = gps
        
        # Initialize data storage
        self.sensor_data[name] = {
            'latitude': 0,
            'longitude': 0,
            'altitude': 0,
            'position': None,
            'timestamp': 0
        }
        
        return gps
    
    def update_sensors(self, time):
        """Update all sensors and process data"""
        
        if self.manager is None:
            self.initialize_manager()
        
        # Update sensor manager
        self.manager.Update(time)
        
        # Process IMU data
        for name, sensor in self.sensors.items():
            if 'IMU' in name:
                self.process_imu_data(name, sensor, time)
            elif 'GPS' in name:
                self.process_gps_data(name, sensor, time)
    
    def process_imu_data(self, name, sensor, time):
        """Process IMU sensor data"""
        
        # Get chassis pose
        chassis_frame = self.chassis.GetFrame_COG_to_abs()
        pos = chassis_frame.GetPos()
        rot = chassis_frame.GetRot()
        
        # Calculate linear acceleration (simplified)
        lin_acc = chrono.ChVectorD(0, 0, 0)
        
        # Calculate angular velocity (simplified)
        ang_vel = chrono.ChVectorD(0, 0, 0)
        
        # Store data
        self.sensor_data[name]['linear_acceleration'] = lin_acc
        self.sensor_data[name]['angular_velocity'] = ang_vel
        self.sensor_data[name]['timestamp'] = time
        self.sensor_data[name]['position'] = pos
        self.sensor_data[name]['orientation'] = rot
    
    def process_gps_data(self, name, sensor, time):
        """Process GPS sensor data"""
        
        # Get chassis position
        chassis_frame = self.chassis.GetFrame_COG_to_abs()
        pos = chassis_frame.GetPos()
        
        # Calculate approximate lat/lon from position (simplified)
        # In real implementation, would use proper geodetic conversion
        lat_origin = 45.0  # degrees
        lon_origin = -93.0  # degrees
        alt_origin = 300.0  # meters
        
        latitude = lat_origin + (pos.y / 111320.0)  # degrees per meter
        longitude = lon_origin + (pos.x / (111320.0 * math.cos(math.radians(lat_origin))))
        altitude = alt_origin + pos.z
        
        # Store data
        self.sensor_data[name]['latitude'] = latitude
        self.sensor_data[name]['longitude'] = longitude
        self.sensor_data[name]['altitude'] = altitude
        self.sensor_data[name]['position'] = pos
        self.sensor_data[name]['timestamp'] = time
    
    def get_imu_data(self, name="IMU_1"):
        """Get current IMU data"""
        if name in self.sensor_data:
            return self.sensor_data[name]
        return None
    
    def get_gps_data(self, name="GPS_1"):
        """Get current GPS data"""
        if name in self.sensor_data:
            return self.sensor_data[name]
        return None
    
    def print_sensor_status(self):
        """Print current sensor status"""
        
        print(f"\n{'='*60}")
        print(f"SENSOR STATUS")
        print(f"{'='*60}")
        
        # Print IMU data
        if 'IMU_1' in self.sensor_data:
            imu_data = self.sensor_data['IMU_1']
            print(f"IMU Data:")
            print(f"  Position: {imu_data.get('position', 'N/A')}")
            if 'linear_acceleration' in imu_data:
                acc = imu_data['linear_acceleration']
                print(f"  Linear Acc: ({acc.x:.3f}, {acc.y:.3f}, {acc.z:.3f}) m/s²")
            if 'angular_velocity' in imu_data:
                ang_vel = imu_data['angular_velocity']
                print(f"  Angular Vel: ({ang_vel.x:.3f}, {ang_vel.y:.3f}, {ang_vel.z:.3f}) rad/s")
            print(f"  Timestamp: {imu_data.get('timestamp', 0):.3f} s")
        
        # Print GPS data
        if 'GPS_1' in self.sensor_data:
            gps_data = self.sensor_data['GPS_1']
            print(f"\nGPS Data:")
            print(f"  Latitude: {gps_data.get('latitude', 0):.8f} °")
            print(f"  Longitude: {gps_data.get('longitude', 0):.8f} °")
            print(f"  Altitude: {gps_data.get('altitude', 0):.3f} m")
            if 'position' in gps_data and gps_data['position']:
                pos = gps_data['position']
                print(f"  ECEF Position: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}) m")
            print(f"  Timestamp: {gps_data.get('timestamp', 0):.3f} s")
        
        print(f"{'='*60}\n")


# =============================================================================
# VISUALIZATION SYSTEM
# =============================================================================

class VisualizationManager:
    """Manages Irrlicht visualization"""
    
    def __init__(self, system, vehicle, terrain, params):
        self.system = system
        self.vehicle = vehicle
        self.terrain = terrain
        self.params = params
        self.app = None
        
    def create_visualization(self):
        """Create Irrlicht visualization application"""
        
        # Create application
        self.app = IrrlichtApp(self.system)
        
        # Set window properties
        self.app.SetSceneGraph(
            self.system.GetSceneGraph()
        )
        
        # Set camera properties
        self.app.AddCamera(
            chrono.ChVectorD(-8, 4, 0),  # Camera position
            chrono.ChVectorD(0, 0, 0),  # Camera target
            60,  # Field of view
            0.01,  # Near clip
            1000  # Far clip
        )
        
        # Add lights
        self.app.AddTypicalLights()
        
        # Add skybox or background
        self.app.SetSkybox()
        
        # Add HUD information
        self.app.AddHUD()
        
        # Render at startup
        self.app.BeginScene()
        self.app.Render()
        self.app.EndScene()
        
        return self.app
    
    def update(self):
        """Update visualization"""
        
        if self.app is not None:
            # Update vehicle camera (optional - follow vehicle)
            # Could be implemented to track vehicle
            
            # Render frame
            self.app.BeginScene()
            self.app.Render()
            self.app.EndScene()
    
    def is_running(self):
        """Check if visualization is still running"""
        
        if self.app is not None:
            return self.app.GetDevice().run()
        return True  # If no visualization, assume running
    
    def cleanup(self):
        """Cleanup visualization resources"""
        
        if self.app is not None:
            del self.app


# =============================================================================
# MAIN SIMULATION CLASS
# =============================================================================

class HMMWVSimulation:
    """Main simulation class managing all components"""
    
    def __init__(self, params=None):
        """Initialize the simulation"""
        
        if params is None:
            self.params = SimulationParameters()
        else:
            self.params = params
        
        # Create PyChrono system
        self.system = chrono.ChSystemNSC()
        self.system.SetNumThreads(4)
        self.system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
        
        # Set system parameters
        self.system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
        self.system.SetSolverType(chrono.ChSolver.Type_PSSOR)
        self.system.SetMaxItersSolverSpeed(50)
        self.system.SetMaxItersSolverStab(50)
        
        # Initialize components
        self.terrain = None
        self.vehicle = None
        self.driver = None
        self.sensor_manager = None
        self.visualization = None
        
        # Timing variables
        self.time = 0.0
        self.render_time = 0.0
        self.sensor_time = 0.0
        
    def setup(self):
        """Setup all simulation components"""
        
        print("\n" + "="*60)
        print("PYCHRONO HMMWV SIMULATION SETUP")
        print("="*60)
        
        # 1. Create terrain
        print("\n[1/5] Creating terrain...")
        terrain_creator = TerrainCreator()
        if self.params.terrain_type == "RIGID_PLANE":
            self.terrain = terrain_creator.create_rigid_plane(
                self.system,
                self.params.friction_coeff
            )
        else:
            self.terrain = terrain_creator.create_height_map_terrain(self.system)
        print("  Terrain created successfully")
        
        # 2. Create vehicle
        print("\n[2/5] Creating HMMWV vehicle...")
        vehicle_builder = HMMWVBuilder(self.system, self.params)
        self.vehicle = vehicle_builder.create_vehicle()
        print("  Vehicle created successfully")
        
        # 3. Create driver
        print("\n[3/5] Creating driver controller...")
        self.driver = DriverController(self.vehicle)
        self.driver.set_target_speed(self.params.target_speed)
        print(f"  Driver created with target speed: {self.params.target_speed} m/s")
        
        # 4. Create sensors
        print("\n[4/5] Creating sensor system...")
        self.sensor_manager = SensorManager(
            self.system,
            self.vehicle.GetChassis(),
            self.params.sensor_update_rate
        )
        self.sensor_manager.initialize_manager()
        self.sensor_manager.add_imu_sensor("IMU_1")
        self.sensor_manager.add_gps_sensor("GPS_1")
        print("  IMU and GPS sensors added successfully")
        
        # 5. Create visualization
        print("\n[5/5] Creating visualization...")
        self.visualization = VisualizationManager(
            self.system,
            self.vehicle,
            self.terrain,
            self.params
        )
        self.visualization.create_visualization()
        print("  Irrlicht visualization created successfully")
        
        print("\n" + "="*60)
        print("SETUP COMPLETE - READY TO RUN SIMULATION")
        print("="*60 + "\n")
    
    def run(self):
        """Run the main simulation loop"""
        
        print("\n" + "="*60)
        print("STARTING SIMULATION")
        print("="*60)
        
        # Calculate number of steps
        total_steps = int(self.params.simulation_time / self.params.step_size)
        output_interval = int(self.params.render_step_size / self.params.step_size)
        
        try:
            step_count = 0
            output_count = 0
            
            while self.time < self.params.simulation_time:
                # Check for visualization quit
                if not self.visualization.is_running():
                    print("\nVisualization closed by user")
                    break
                
                # Advance driver
                self.driver.advance(self.params.step_size)
                
                # Advance vehicle
                self.vehicle.Advance(self.params.step_size)
                
                # Update sensors
                if self.time >= self.sensor_time:
                    self.sensor_manager.update_sensors(self.time)
                    self.sensor_time += 1.0 / self.params.sensor_update_rate
                
                # Update visualization
                if self.time >= self.render_time:
                    self.visualization.update()
                    self.render_time += self.params.render_step_size
                    output_count += 1
                    
                    # Print periodic updates
                    if output_count % 10 == 0:
                        self.print_status(step_count, total_steps)
                
                # Advance system
                self.system.DoStepDynamics(self.params.step_size)
                
                # Update time
                self.time += self.params.step_size
                step_count += 1
                
        except KeyboardInterrupt:
            print("\nSimulation interrupted by user")
        
        print("\n" + "="*60)
        print("SIMULATION COMPLETE")
        print("="*60)
        print(f"Final Time: {self.time:.3f} s")
        print(f"Total Steps: {step_count}")
        print(f"Final Vehicle Speed: {self.vehicle.GetVehicleSpeed():.3f} m/s")
        
        # Print final sensor status
        self.sensor_manager.print_sensor_status()
    
    def print_status(self, step, total):
        """Print simulation status"""
        
        # Get vehicle data
        speed = self.vehicle.GetVehicleSpeed()
        chassis_pos = self.vehicle.GetChassis().GetFrame_COG_to_abs().GetPos()
        
        # Get driver inputs
        throttle = self.driver.get_throttle()
        braking = self.driver.get_braking()
        
        # Calculate progress
        progress = (step / total) * 100 if total > 0 else 0
        
        # Print status bar
        status_bar = f"[{'#' * int(progress/5):<20}]"
        
        print(f"\r  {status_bar} {progress:5.1f}% | "
              f"Time: {self.time:6.2f}s | "
              f"Speed: {speed:5.2f} m/s | "
              f"Throttle: {throttle:.2f} | "
              f"Braking: {braking:.2f} | "
              f"Pos: ({chassis_pos.x:.1f}, {chassis_pos.y:.1f}, {chassis_pos.z:.1f})    ",
              end="", flush=True)
    
    def cleanup(self):
        """Cleanup all resources"""
        
        print("\nCleaning up resources...")
        
        if self.visualization:
            self.visualization.cleanup()
        
        # Clear system
        self.system.Clear()
        
        print("Cleanup complete")


# =============================================================================
# MAIN ENTRY POINT
# =============================================================================

def main():
    """Main entry point for the simulation"""
    
    print("\n" + "="*60)
    print("PYCHRONO HMMWV VEHICLE SIMULATION")
    print("="*60)
    print("\nThis simulation features:")
    print("  - HMMWV vehicle with full drivetrain")
    print("  - Rigid plane terrain")
    print("  - Python-based driver controller")
    print("  - IMU and GPS sensors")
    print("  - Irrlicht 3D visualization")
    print("="*60)
    
    # Create simulation parameters
    params = SimulationParameters()
    
    # Optionally modify parameters
    # params.simulation_time = 60.0
    # params.target_speed = 10.0
    
    # Create and run simulation
    simulation = HMMWVSimulation(params)
    
    try:
        simulation.setup()
        simulation.run()
    finally:
        simulation.cleanup()
    
    print("\n" + "="*60)
    print("SIMULATION SESSION ENDED")
    print("="*60 + "\n")


if __name__ == "__main__":
    main()