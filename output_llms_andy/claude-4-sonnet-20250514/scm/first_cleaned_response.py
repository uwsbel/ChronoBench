import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.sensor as sens
import math
import numpy as np


chrono.SetChronoDataPath("C:/path/to/chrono/data/")  
veh.SetDataPath("C:/path/to/chrono/data/vehicle/")   


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))


system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


class HMMWVSimulation:
    def __init__(self):
        
        self.initLoc = chrono.ChVectorD(0, 0, 1.0)
        self.initRot = chrono.ChQuaternionD(1, 0, 0, 0)
        
        
        self.step_size = 1e-3
        self.tend = 1000
        self.render_step_size = 1.0/50  
        self.output_step_size = 1.0/10  
        
        
        self.render_steps = math.ceil(self.render_step_size / self.step_size)
        self.output_steps = math.ceil(self.output_step_size / self.step_size)
        
        
        self.create_vehicle()
        
        
        self.create_terrain()
        
        
        self.create_driver()
        
        
        self.create_visualization()
    
    def create_vehicle(self):
        
        
        self.vehicle = veh.HMMWV_Full()
        self.vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
        self.vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
        self.vehicle.SetChassisFixed(False)
        self.vehicle.SetInitPosition(chrono.ChCoordsysD(self.initLoc, self.initRot))
        
        
        self.vehicle.SetPowertrainType(veh.PowertrainModelType_SHAFTS)
        
        
        self.vehicle.SetDriveType(veh.DrivelineTypeWV_AWD)
        
        
        tire_type = veh.TireModelType_RIGID
        self.vehicle.SetTireType(tire_type)
        
        
        self.vehicle.Initialize()
        
        
        self.vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)
        
        
        self.vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
        
        
        self.vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
        
        
        self.vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
        
        
        self.vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
        
        
        for axle in self.vehicle.GetVehicle().GetAxles():
            for wheel in axle.GetWheels():
                wheel.GetTire().SetVisualizationType(veh.VisualizationType_MESH)
    
    def create_terrain(self):
        
        
        self.terrain = veh.SCMTerrain(system)
        
        
        self.terrain.SetSoilParameters(
            2e6,     
            0,       
            1.1,     
            0,       
            30,      
            0.01,    
            2e8,     
            3e4      
        )
        
        
        self.terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)
        
        
        self.terrain.EnableMovingPatch(self.vehicle.GetVehicle().GetChassisBody(), 
                                     10, 6)  
        
        
        self.terrain.SetMeshWireframe(False)
        
        
        terrain_length = 100.0
        terrain_width = 100.0
        self.terrain.Initialize(terrain_length, terrain_width, 0.02)
        
        
        self.terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 200, 200)
        self.terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)
        
    def create_driver(self):
        
        
        self.driver = veh.ChInteractiveDriverIRR()
        
        
        steering_time = 1.0  
        throttle_time = 1.0  
        braking_time = 0.3   
        
        self.driver.SetSteeringDelta(self.render_step_size / steering_time)
        self.driver.SetThrottleDelta(self.render_step_size / throttle_time)
        self.driver.SetBrakingDelta(self.render_step_size / braking_time)
        
        
        self.driver.Initialize()
    
    def create_visualization(self):
        
        
        self.vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        self.vis.SetWindowTitle('HMMWV on SCM Terrain')
        self.vis.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 1.75), 6.0, 0.5)
        self.vis.Initialize()
        self.vis.AddLightDirectional()
        self.vis.AddSkyBox()
        self.vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
        self.vis.AttachVehicle(self.vehicle.GetVehicle())
        
        
        self.vis.GetDevice().getSceneManager().addLightSceneNode(
            None, 
            chrono.ChVectorD(100, 100, 100),
            chrono.ChColor(0.7, 0.7, 0.7),
            500
        )
        
        
        self.vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
        
    def run_simulation(self):
        
        
        step_number = 0
        render_frame = 0
        
        
        while self.vis.Run():
            time = system.GetChTime()
            
            
            if time >= self.tend:
                break
                
            
            if step_number % self.render_steps == 0:
                
                driver_inputs = self.driver.GetInputs()
                
                
                if render_frame % 100 == 0:  
                    mass = self.vehicle.GetVehicle().GetMass()
                    print(f"Time: {time:.2f}s, Vehicle mass: {mass:.2f} kg")
                
                self.vis.BeginScene()
                self.vis.Render()
                
                
                self.terrain.GetMesh().SetWireframe(False)
                
                self.vis.EndScene()
                render_frame += 1
            
            
            driver_inputs = self.driver.GetInputs()
            
            
            self.driver.Synchronize(time)
            self.terrain.Synchronize(time)
            self.vehicle.Synchronize(time, driver_inputs, self.terrain)
            
            
            self.driver.Advance(self.step_size)
            self.terrain.Advance(self.step_size)
            self.vehicle.Advance(self.step_size)
            system.DoStepDynamics(self.step_size)
            
            
            step_number += 1
            
            
            if step_number % self.output_steps == 0:
                
                pos = self.vehicle.GetVehicle().GetPos()
                vel = self.vehicle.GetVehicle().GetPointVelocity(chrono.ChVectorD(0,0,0))
                
                
                print(f"Time: {time:.2f}, Pos: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}), "
                      f"Speed: {vel.Length():.2f} m/s")


def main():
    print("HMMWV SCM Deformable Terrain Simulation")
    print("========================================")
    print("Vehicle: HMMWV Full Vehicle")
    print("Terrain: SCM Deformable")
    print("Tire Model: Rigid")
    print("Visualization: Irrlicht with mesh rendering")
    print("")
    print("Controls:")
    print("  W/S - Throttle/Brake")
    print("  A/D - Steering")
    print("  Space - Handbrake")
    print("")
    
    
    simulation = HMMWVSimulation()
    simulation.run_simulation()
    
    print("Simulation completed!")

if __name__ == "__main__":
    main()