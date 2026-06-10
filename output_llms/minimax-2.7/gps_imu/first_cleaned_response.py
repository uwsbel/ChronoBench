import os
import sys
import math
import numpy as np


import pychrono as chrono
import pychrono.vehicle as chrvehicle
import pychrono.sensor as chrsensor
from pychrono.irreffects import IrrlichtApp


chrono.SetDataPath("C:/Chrono/Data/")
chrvehicle.SetDataPath("C:/ChronoVehicle/Data/")






class SimulationParameters:
    
    
    def __init__(self):
        
        self.step_size = 1e-3  
        self.simulation_time = 30.0  
        self.render_step_size = 1.0 / 60.0  
        
        
        self.vehicle_type = "HMMWV"  
        self.chassis_height = 0.8  
        self.chassis_vis = True  
        
        
        self.terrain_type = "RIGID_PLANE"  
        self.terrain_size = 200.0  
        self.friction_coeff = 0.8  
        
        
        self.driver_type = "PYTHON_DRIVER"  
        self.target_speed = 8.0  
        
        
        self.sensor_update_rate = 100  
        self.gps_update_rate = 10  
        self.imu_update_rate = 100  
        
        
        self.output_enabled = True  
        self.output_directory = "./hmmwv_simulation_output/"






class TerrainCreator:
    
    
    @staticmethod
    def create_rigid_plane(system, friction=0.8):
        
        
        
        ground_material = chrono.ChMaterialSurfaceNSC()
        ground_material.SetFriction(friction)
        ground_material.SetRestitution(0.1)
        
        
        ground = chrono.ChBody()
        ground.SetBodyFixed(True)
        ground.SetMaterialSurface(chrono.ChMaterialSurfaceNSC(ground_material))
        
        
        ground.GetVisualShape().SetTexture(
            chrono.GetChronoDataFile("texture/concrete.png")
        )
        
        
        ground.GetCollisionModel().SetBoxSealed(False)
        ground.GetCollisionModel().AttachBox(100, 0.02, 100)
        ground.SetCollide(True)
        
        
        system.Add(ground)
        
        return ground
    
    @staticmethod
    def create_height_map_terrain(system, filename=None):
        
        
        if filename is None:
            
            filename = chrono.GetChronoDataFile("terrain/height_maps/UK_terrain_data.csv")
        
        
        ter_dim_x = 200.0  
        ter_dim_y = 200.0  
        ter_size_x = 200.0  
        ter_size_y = 200.0  
        ter_thickness = 0.1  
        
        
        terrain = chrono.ChBody()
        terrain.SetBodyFixed(True)
        
        
        
        terrain.GetCollisionModel().SetBoxSealed(False)
        terrain.GetCollisionModel().AttachBox(ter_size_x / 2, ter_thickness / 2, ter_size_y / 2)
        terrain.SetCollide(True)
        
        system.Add(terrain)
        
        return terrain






class HMMWVBuilder:
    
    
    def __init__(self, system, params):
        self.system = system
        self.params = params
        
    def create_vehicle(self):
        
        
        
        init_loc = chrono.ChVectorD(0, self.params.chassis_height, 0)
        init_rot = chrono.ChQuaternionD(1, 0, 0, 0)
        init_vel = chrono.ChVectorD(0, 0, 0)
        
        
        self.vehicle = chrvehicle.HMMWV(
            self.system,
            chrvehicle.HMMWV_ReduceTireMass.NO,
            chrvehicle.DrivelineTypeWV.AWD,
            chrvehicle.EngineModelType.SIMPLE,
            chrvehicle.TransmissionModelState.SIMPLE_MAP,
            chrvehicle.TireModelType.RIGID
        )
        
        
        self.vehicle.Initialize(
            chrono.ChCoordsysD(init_loc, init_rot),
            init_vel,
            self.params.vehicle_type
        )
        
        
        self.chassis = self.vehicle.GetChassis()
        self.powertrain = self.vehicle.GetPowertrain()
        self.wheels = [
            self.vehicle.GetWheel(0),  
            self.vehicle.GetWheel(1),  
            self.vehicle.GetWheel(2),  
            self.vehicle.GetWheel(3),  
        ]
        self.suspensions = [
            self.vehicle.GetSuspension(0),
            self.vehicle.GetSuspension(1),
            self.vehicle.GetSuspension(2),
            self.vehicle.GetSuspension(3),
        ]
        
        
        self.vehicle.SetVisualization(
            self.params.chassis_vis,
            chrvehicle.VisualizationType.MESH,
            chrvehicle.VisualizationType.MESH
        )
        
        
        self.print_vehicle_info()
        
        return self.vehicle
    
    def print_vehicle_info(self):
        
        
        
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






class DriverController:
    
    
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.target_speed = 8.0  
        self.throttle = 0.0
        self.braking = 0.0
        self.steering = 0.0
        self.throttle_threshold = 0.98
        
    def advance(self, step_size):
        
        
        
        speed = self.vehicle.GetVehicleSpeed()
        
        
        speed_error = self.target_speed - speed
        
        
        if speed_error > 0.1:
            self.throttle = min(1.0, self.throttle + 0.02)
            self.braking = 0.0
        elif speed_error < -0.1:
            self.throttle = max(0.0, self.throttle - 0.01)
            self.braking = 0.2
        else:
            self.throttle = min(self.throttle + 0.01, self.throttle_threshold)
            self.braking = 0.0
        
        
        self.steering = 0.0
        
        
        self.vehicle.DriveStrut().SetDisplacement(self.steering)
        self.vehicle.ApplyThrottle(self.throttle)
        self.vehicle.ApplyBraking(self.braking)
        
    def set_target_speed(self, speed):
        
        self.target_speed = speed
    
    def get_throttle(self):
        
        return self.throttle
    
    def get_braking(self):
        
        return self.braking
    
    def get_steering(self):
        
        return self.steering






class SensorManager:
    
    
    def __init__(self, system, chassis, update_rate=100):
        self.system = system
        self.chassis = chassis
        self.update_rate = update_rate
        self.sensors = {}
        self.sensor_data = {}
        self.manager = None
        
    def initialize_manager(self):
        
        
        
        self.manager = chrsensor.ChSensorManager(self.system)
        
        
        self.manager.Add毽tLight(
            chrono.ChVectorD(0, 10, 0),
            chrono.ChColor(1, 1, 1),
            100
        )
        
        return self.manager
    
    def add_imu_sensor(self, name="IMU_1", offset_position=None, update_rate=None):
        
        
        if update_rate is None:
            update_rate = self.update_rate
        
        if offset_position is None:
            offset_position = chrono.ChFrameD(
                chrono.ChVectorD(0, 0.5, 0),  
                chrono.ChQuaternionD(1, 0, 0, 0)  
            )
        
        
        imu = chrsensor.ChIMUSensor(
            self.chassis,
            offset_position,
            update_rate
        )
        
        
        imu.SetName(name)
        imu.SetFlagActive(True)
        
        
        imu.RegisterUserOffsetCallback(
            lambda: chrono.ChVectorD(0, 0, 0),
            lambda: chrono.ChQuaternionD(1, 0, 0, 0)
        )
        
        
        self.sensors[name] = imu
        
        
        self.sensor_data[name] = {
            'linear_acceleration': [],
            'angular_velocity': [],
            'magnetic_field': [],
            'gps': None,
            'timestamp': 0
        }
        
        return imu
    
    def add_gps_sensor(self, name="GPS_1", offset_position=None, update_rate=10):
        
        
        if offset_position is None:
            offset_position = chrono.ChFrameD(
                chrono.ChVectorD(0, 1.0, 0),  
                chrono.ChQuaternionD(1, 0, 0, 0)
            )
        
        
        gps = chrsensor.ChGPSSensor(
            self.chassis,
            offset_position,
            update_rate
        )
        
        
        gps.SetName(name)
        gps.SetFlagActive(True)
        
        
        self.sensors[name] = gps
        
        
        self.sensor_data[name] = {
            'latitude': 0,
            'longitude': 0,
            'altitude': 0,
            'position': None,
            'timestamp': 0
        }
        
        return gps
    
    def update_sensors(self, time):
        
        
        if self.manager is None:
            self.initialize_manager()
        
        
        self.manager.Update(time)
        
        
        for name, sensor in self.sensors.items():
            if 'IMU' in name:
                self.process_imu_data(name, sensor, time)
            elif 'GPS' in name:
                self.process_gps_data(name, sensor, time)
    
    def process_imu_data(self, name, sensor, time):
        
        
        
        chassis_frame = self.chassis.GetFrame_COG_to_abs()
        pos = chassis_frame.GetPos()
        rot = chassis_frame.GetRot()
        
        
        lin_acc = chrono.ChVectorD(0, 0, 0)
        
        
        ang_vel = chrono.ChVectorD(0, 0, 0)
        
        
        self.sensor_data[name]['linear_acceleration'] = lin_acc
        self.sensor_data[name]['angular_velocity'] = ang_vel
        self.sensor_data[name]['timestamp'] = time
        self.sensor_data[name]['position'] = pos
        self.sensor_data[name]['orientation'] = rot
    
    def process_gps_data(self, name, sensor, time):
        
        
        
        chassis_frame = self.chassis.GetFrame_COG_to_abs()
        pos = chassis_frame.GetPos()
        
        
        
        lat_origin = 45.0  
        lon_origin = -93.0  
        alt_origin = 300.0  
        
        latitude = lat_origin + (pos.y / 111320.0)  
        longitude = lon_origin + (pos.x / (111320.0 * math.cos(math.radians(lat_origin))))
        altitude = alt_origin + pos.z
        
        
        self.sensor_data[name]['latitude'] = latitude
        self.sensor_data[name]['longitude'] = longitude
        self.sensor_data[name]['altitude'] = altitude
        self.sensor_data[name]['position'] = pos
        self.sensor_data[name]['timestamp'] = time
    
    def get_imu_data(self, name="IMU_1"):
        
        if name in self.sensor_data:
            return self.sensor_data[name]
        return None
    
    def get_gps_data(self, name="GPS_1"):
        
        if name in self.sensor_data:
            return self.sensor_data[name]
        return None
    
    def print_sensor_status(self):
        
        
        print(f"\n{'='*60}")
        print(f"SENSOR STATUS")
        print(f"{'='*60}")
        
        
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






class VisualizationManager:
    
    
    def __init__(self, system, vehicle, terrain, params):
        self.system = system
        self.vehicle = vehicle
        self.terrain = terrain
        self.params = params
        self.app = None
        
    def create_visualization(self):
        
        
        
        self.app = IrrlichtApp(self.system)
        
        
        self.app.SetSceneGraph(
            self.system.GetSceneGraph()
        )
        
        
        self.app.AddCamera(
            chrono.ChVectorD(-8, 4, 0),  
            chrono.ChVectorD(0, 0, 0),  
            60,  
            0.01,  
            1000  
        )
        
        
        self.app.AddTypicalLights()
        
        
        self.app.SetSkybox()
        
        
        self.app.AddHUD()
        
        
        self.app.BeginScene()
        self.app.Render()
        self.app.EndScene()
        
        return self.app
    
    def update(self):
        
        
        if self.app is not None:
            
            
            
            
            self.app.BeginScene()
            self.app.Render()
            self.app.EndScene()
    
    def is_running(self):
        
        
        if self.app is not None:
            return self.app.GetDevice().run()
        return True  
    
    def cleanup(self):
        
        
        if self.app is not None:
            del self.app






class HMMWVSimulation:
    
    
    def __init__(self, params=None):
        
        
        if params is None:
            self.params = SimulationParameters()
        else:
            self.params = params
        
        
        self.system = chrono.ChSystemNSC()
        self.system.SetNumThreads(4)
        self.system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
        
        
        self.system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
        self.system.SetSolverType(chrono.ChSolver.Type_PSSOR)
        self.system.SetMaxItersSolverSpeed(50)
        self.system.SetMaxItersSolverStab(50)
        
        
        self.terrain = None
        self.vehicle = None
        self.driver = None
        self.sensor_manager = None
        self.visualization = None
        
        
        self.time = 0.0
        self.render_time = 0.0
        self.sensor_time = 0.0
        
    def setup(self):
        
        
        print("\n" + "="*60)
        print("PYCHRONO HMMWV SIMULATION SETUP")
        print("="*60)
        
        
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
        
        
        print("\n[2/5] Creating HMMWV vehicle...")
        vehicle_builder = HMMWVBuilder(self.system, self.params)
        self.vehicle = vehicle_builder.create_vehicle()
        print("  Vehicle created successfully")
        
        
        print("\n[3/5] Creating driver controller...")
        self.driver = DriverController(self.vehicle)
        self.driver.set_target_speed(self.params.target_speed)
        print(f"  Driver created with target speed: {self.params.target_speed} m/s")
        
        
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
        
        
        print("\n" + "="*60)
        print("STARTING SIMULATION")
        print("="*60)
        
        
        total_steps = int(self.params.simulation_time / self.params.step_size)
        output_interval = int(self.params.render_step_size / self.params.step_size)
        
        try:
            step_count = 0
            output_count = 0
            
            while self.time < self.params.simulation_time:
                
                if not self.visualization.is_running():
                    print("\nVisualization closed by user")
                    break
                
                
                self.driver.advance(self.params.step_size)
                
                
                self.vehicle.Advance(self.params.step_size)
                
                
                if self.time >= self.sensor_time:
                    self.sensor_manager.update_sensors(self.time)
                    self.sensor_time += 1.0 / self.params.sensor_update_rate
                
                
                if self.time >= self.render_time:
                    self.visualization.update()
                    self.render_time += self.params.render_step_size
                    output_count += 1
                    
                    
                    if output_count % 10 == 0:
                        self.print_status(step_count, total_steps)
                
                
                self.system.DoStepDynamics(self.params.step_size)
                
                
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
        
        
        self.sensor_manager.print_sensor_status()
    
    def print_status(self, step, total):
        
        
        
        speed = self.vehicle.GetVehicleSpeed()
        chassis_pos = self.vehicle.GetChassis().GetFrame_COG_to_abs().GetPos()
        
        
        throttle = self.driver.get_throttle()
        braking = self.driver.get_braking()
        
        
        progress = (step / total) * 100 if total > 0 else 0
        
        
        status_bar = f"[{'
        
        print(f"\r  {status_bar} {progress:5.1f}% | "
              f"Time: {self.time:6.2f}s | "
              f"Speed: {speed:5.2f} m/s | "
              f"Throttle: {throttle:.2f} | "
              f"Braking: {braking:.2f} | "
              f"Pos: ({chassis_pos.x:.1f}, {chassis_pos.y:.1f}, {chassis_pos.z:.1f})    ",
              end="", flush=True)
    
    def cleanup(self):
        
        
        print("\nCleaning up resources...")
        
        if self.visualization:
            self.visualization.cleanup()
        
        
        self.system.Clear()
        
        print("Cleanup complete")






def main():
    
    
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
    
    
    params = SimulationParameters()
    
    
    
    
    
    
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