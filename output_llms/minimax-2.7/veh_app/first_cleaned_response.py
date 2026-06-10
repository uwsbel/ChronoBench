import numpy as np
import math
import os
import sys


import pychrono as chrono
import pychrono.vehicle as chrvehicle
import pychrono.sensor as chrsensor


chrono.SetDataPath("../../../data/")


class GatorVehicleSimulation:
    
    
    def __init__(self):
        
        
        
        self.step_size = 1e-3
        self.end_time = 10.0
        self.render_step = 1.0/60.0
        
        
        self.system = chrono.ChSystemNSC()
        self.system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
        self.system.SetSolverType(chrono.ChSolver.Type_SOR)
        self.system.SetMaxItersSolverSpeed(50)
        self.system.SetMaxItersSolverStab(50)
        self.system.SetTolerance(1e-6)
        self.system.SetStepSize(self.step_size)
        
        
        self._create_terrain()
        
        
        self._create_vehicle()
        
        
        self._create_driver()
        
        
        self._create_sensors()
        
        
        self._create_visualization()
        
        
        self.render_simulation = True
        self.simulation_time = 0.0
        
    def _create_terrain(self):
        
        
        
        terrain_height = 0.0  
        terrain_size = 50.0   
        
        
        ground_material = chrono.ChMaterialSurfaceNSC()
        ground_material.SetFriction(0.8)
        ground_material.SetRestitution(0.1)
        
        
        self.ground = chrono.ChBody()
        self.ground.SetBodyFixed(True)
        self.ground.SetMaterialSurface(ground_material)
        
        
        ground_shape = chrono.ChBoxShape(terrain_size, 0.5, terrain_size)
        ground_shape.SetTexture(chrono.GetChronoDataFile("textures/tile4.png"))
        self.ground.AddVisualShape(ground_shape, chrono.ChFrameD(
            chrono.ChVectorD(0, -0.5, 0)))
        
        
        num_obstacles = 10
        for i in range(num_obstacles):
            obs = chrono.ChBody()
            obs.SetMaterialSurface(ground_material)
            angle = 2 * math.pi * i / num_obstacles
            radius = 8.0
            obs.SetPos(chrono.ChVectorD(
                radius * math.cos(angle), 0.5, radius * math.sin(angle)))
            obs.SetBodyFixed(False)
            obs.SetMass(10)
            
            
            cyl_shape = chrono.ChCylinderShape(0.3, 1.0)
            obs.AddCollisionShape(cyl_shape)
            obs.SetCollide(True)
            
            
            vis_cyl = chrono.ChVisualizationGeometry()
            vis_cyl = chrono.ChCylinderShape(0.3, 1.0)
            obs.AddVisualShape(vis_cyl)
            
            self.system.AddBody(obs)
        
        self.system.AddBody(self.ground)
        print("Terrain created with obstacles")
        
    def _create_vehicle(self):
        
        
        
        self.vehicle = chrvehicle.Gator(self.system, 
                                        chrvehicle.DrivelineTypeWV_AWD,
                                        chrvehicle.EngineModelType_SIMPLE,
                                        chrvehicle.TransmissionModelType_SIMPLE)
        
        
        self.vehicle.SetTireContactModel(chrvehicle.TireContactModel_RIGID)
        
        
        init_loc = chrono.ChVectorD(-15, 1.5, 5)
        init_rot = chrono.ChQuaternionD(1, 0, 0, 0)
        self.vehicle.Initialize(chrono.ChCoordsysD(init_loc, init_rot))
        
        
        self.vehicle.SetChassisVisualizationType(chrvehicle.VisualizationType_PRIMITIVES)
        self.vehicle.SetWheelVisualizationType(chrvehicle.VisualizationType_MESH)
        self.vehicle.SetSuspensionVisualizationType(chrvehicle.VisualizationType_PRIMITIVES)
        self.vehicle.SetSteeringVisualizationType(chrvehicle.VisualizationType_PRIMITIVES)
        
        
        self.chassis = self.vehicle.GetChassis()
        self.driver = None  
        
        print("Gator vehicle initialized")
        
    def _create_driver(self):
        
        
        
        self.driver = chrvehicle.ChInteractiveDriverIRR(self.vehicle)
        
        
        self.driver.Initialize()
        
        
        self.driver.SetThrottleDelta(0.02)
        self.driver.SetSteeringDelta(0.03)
        self.driver.SetBrakingDelta(0.02)
        
        print("Interactive driver system created")
        
    def _create_sensors(self):
        
        
        
        self.sensor_manager = chrsensor.ChSensorManager(self.system)
        
        
        self._add_point_lights()
        
        
        self._add_chassis_camera()
        
        print("Sensor system initialized")
        
    def _add_point_lights(self):
        
        
        
        light1 = chrsensor.ChLightPoint(
            chrono.ChVectorD(0, 5, -10),
            chrono.ChColor(1.0, 1.0, 0.9),
            100.0,  
            20.0    
        )
        self.sensor_manager.AddLight(light1)
        
        
        light2 = chrsensor.ChLightPoint(
            chrono.ChVectorD(10, 5, 0),
            chrono.ChColor(1.0, 0.9, 0.8),
            80.0,
            15.0
        )
        self.sensor_manager.AddLight(light2)
        
        
        light3 = chrsensor.ChLightPoint(
            chrono.ChVectorD(-5, 8, 5),
            chrono.ChColor(0.6, 0.7, 0.9),
            50.0,
            30.0
        )
        self.sensor_manager.AddLight(light3)
        
        print(f"Added {3} point lights to the scene")
        
    def _add_chassis_camera(self):
        
        
        
        chassis_frame = self.chassis.GetFrame()
        
        
        camera_offset = chrono.ChFrameD(
            chrono.ChVectorD(0, 1.5, 2.0),  
            chrono.ChQuaternionD(chrono.Q_ROTATE_Y_TO_RAD, 0)  
        )
        
        
        camera_params = chrsensor.ChCameraParameters()
        camera_params.resolution = chrsensor.ChVector2i(640, 480)
        camera_params.frustum_angle = 60.0 * (math.pi / 180.0)  
        camera_params.frustum_znear = 0.1
        camera_params.frustum_zfar = 100.0
        camera_params.sample_rate = 30.0
        
        
        self.chassis_camera = chrsensor.ChSensorCamera(
            "ChassisCamera",
            camera_offset,
            camera_params
        )
        
        
        self.chassis_camera.SetName("Vehicle Rear Camera")
        
        
        self.sensor_manager.AddSensor(self.chassis_camera)
        
        
        self._create_render_target()
        
        print("Chassis camera sensor added")
        
    def _create_render_target(self):
        
        
        
        self.render_target = chrsensor.ChRenderTarget()
        self.render_target.SetWidth(640)
        self.render_target.SetHeight(480)
        self.render_target.SetPostProcess(True)
        self.render_target.SetSaveCameraData(True)
        self.render_target.SetUseSingleBuffer(True)
        
        
        self.chassis_camera.AddRenderTarget(self.render_target)
        
        print("Render target created for camera")
        
    def _create_visualization(self):
        
        
        
        self.application = None
        
        
        try:
            self.application = chrono.ChVisualSystemIrrlicht()
            self.application.AttachSystem(self.system)
            self.application.AddCamera(chrono.ChVectorD(0, 10, -20))
            self.application.AddTypicalLights()
            self.application.AddLightWithShadow(
                chrono.ChVectorD(10, 20, -10),
                chrono.ChVectorD(0, 0, 0),
                50, 10, 50, 60, 20)
            self.application.SetWindowSize(1280, 720)
            self.application.SetWindowTitle("PyChrono Gator Vehicle Simulation")
            self.application.AddLogo()
            self.application.Initialize()
            
            print("Irrlicht visualization initialized")
            
        except Exception as e:
            print(f"Irrlicht visualization not available: {e}")
            print("Running in headless mode")
            self.render_simulation = False
            
    def update_simulation(self, step_time):
        
        
        
        self.driver.Update()
        
        
        self.vehicle.Update(step_time)
        
        
        self.sensor_manager.Update(step_time)
        
        
        self.system.DoStepDynamics(self.step_size)
        
        
        self.simulation_time += self.step_size
        
    def update_visualization(self):
        
        
        if self.application and self.render_simulation:
            self.application.Synchronize("", [])
            self.application.Render()
            return self.application.Run()
        return True
        
    def run(self):
        
        
        print("\n" + "="*60)
        print("Starting PyChrono Gator Vehicle Simulation")
        print("="*60)
        print(f"Step size: {self.step_size}")
        print(f"End time: {self.end_time}")
        print("="*60 + "\n")
        
        
        vehicle_state = self.vehicle.GetVehicleSpeed()
        print(f"Initial vehicle speed: {vehicle_state} m/s")
        
        
        step_number = 0
        last_render_time = 0.0
        last_camera_capture = 0.0
        
        try:
            while self.simulation_time < self.end_time:
                
                
                if not self.update_visualization():
                    print("Visualization closed by user")
                    break
                
                
                self.update_simulation(self.step_size)
                step_number += 1
                
                
                if step_number % 500 == 0:
                    speed = self.vehicle.GetVehicleSpeed()
                    pos = self.chassis.GetPos()
                    print(f"Time: {self.simulation_time:.2f}s | "
                          f"Speed: {speed:.2f} m/s | "
                          f"Position: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})")
                
                
                if self.simulation_time - last_camera_capture > 1.0:
                    self._capture_camera_image(step_number)
                    last_camera_capture = self.simulation_time
                    
        except KeyboardInterrupt:
            print("\nSimulation interrupted by user")
            
        finally:
            self._print_simulation_summary(step_number)
            
    def _capture_camera_image(self, step_num):
        
        
        try:
            
            camera_data = self.chassis_camera.GetMostRecentData()
            if camera_data:
                
                
                pass
                
        except Exception as e:
            print(f"Camera capture error: {e}")
            
    def _print_simulation_summary(self, total_steps):
        
        
        print("\n" + "="*60)
        print("Simulation Complete")
        print("="*60)
        print(f"Total simulation time: {self.simulation_time:.2f} seconds")
        print(f"Total physics steps: {total_steps}")
        print(f"Final vehicle speed: {self.vehicle.GetVehicleSpeed():.2f} m/s")
        print(f"Final chassis position: {self.chassis.GetPos()}")
        print("="*60)


def main():
    
    
    print("PyChrono Gator Vehicle Simulation")
    print("Initializing components...")
    
    
    simulation = GatorVehicleSimulation()
    simulation.run()
    
    print("\nSimulation finished successfully!")


if __name__ == "__main__":
    main()