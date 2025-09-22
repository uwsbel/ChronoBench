import math
import numpy as np
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


system = chrono.ChSystemSMC()
chrono.SetChronoDataPath("chrono_data/")


rover = veh.Rover()
rover.SetContactMethod(chrono.ChContactMethod.NSC)
rover.SetChassisCollisionType(chrono.ChCollisionSystem.Type_BULLET)
rover.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
rover.Initialize()


terrain = veh.RigidTerrain(system)
terrain_material = chrono.ChMaterialSurfaceNSC()
terrain_material.SetFriction(0.5)
terrain_material.SetRestitution(0.1)
terrain_patch = terrain.AddPatch(terrain_material, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 100, 100)
terrain.Initialize()


class RoverDriver:
    def __init__(self, rover):
        self.rover = rover
        self.time = 0
        self.speed = 1.0  
        self.steering = 0.0
    
    def Update(self, time):
        self.time = time
        
        self.steering = 0.5 * math.sin(time / 2)
        self.rover.GetDriver().SetSteering(self.steering)
        self.rover.GetDriver().SetThrottle(self.speed)

driver = RoverDriver(rover)
rover.SetDriver(driver)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Curiosity Rover Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()


camera_pos = chrono.ChVector3d(10, 10, 10)
camera_rot = chrono.ChQuaterniond(0.7071, 0, 0.7071, 0)
vis.AddCamera(camera_pos, camera_rot)
vis.SetRenderShadows(True)
vis.SetShadowIntensity(0.5)


simulation_time = 10.0
step_size = 0.01
render_steps = 10


time = 0.0
while vis.Run() and time < simulation_time:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    system.DoStepDynamics(step_size)
    driver.Update(time)
    
    time += step_size
    
vis.Close()


chrono.GetLog().Write("Simulation completed successfully\n")