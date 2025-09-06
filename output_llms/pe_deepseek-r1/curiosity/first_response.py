import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import math

# 1. Initialize system and paths
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
system = chrono.ChSystemSMC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

# 2. Create rigid terrain
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialSMC()
patch_mat.SetFriction(0.9)  # High friction for rocky terrain
patch_mat.SetRestitution(0.1)
patch = terrain.AddPatch(patch_mat, 
                        chrono.CSYSNORM, 
                        100, 50)  # 100x50 meter terrain
patch.SetTexture(veh.GetDataFile("terrain/textures/rock.jpg"), 100, 50)
terrain.Initialize()

# 3. Create Curiosity rover
rover = veh.Curiosity(system)
rover.SetContactMethod(chrono.ChContactMethod_SMC)
rover.SetChassisFixed(False)
rover.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.5), chrono.QUNIT))
rover.Initialize()

# Set subsystems visible
rover.SetWheelVisualizationType(veh.VisualizationType_MESH)
rover.SetChassisVisualizationType(veh.VisualizationType_MESH)
rover.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)

# 4. Create motor control driver
class RoverDriver(chrono.ChDriver):
    def __init__(self, rover):
        super().__init__(rover.GetVehicle())
        self.rover = rover
        self.steering = 0
        self.speed = 0.5  # m/s
        self.time = 0
        
    def Synchronize(self, time):
        self.time = time
        # Sine-wave steering pattern
        self.steering = 0.3 * math.sin(time * 0.5)
        
    def Advance(self, step):
        # Apply controls to rover wheels
        self.SetSteering(self.steering)
        self.SetThrottle(0.5)  # Constant throttle
        
driver = RoverDriver(rover)
rover.GetVehicle().SetDriver(driver)

# 5. Initialize visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Curiosity Rover Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(3, 3, 1.5), chrono.ChVector3d(0, 0, 0.5))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(5, 5, 5), 
                      chrono.ChVector3d(0, 0, 0), 
                      50, 3, 10, 40, 512)

# 6. Simulation loop
time_step = 0.001
frame_step = 1.0 / 60  # Render at 60 FPS
time = 0
frame_time = 0

while vis.Run():
    time += time_step
    frame_time += time_step
    
    # Update driver inputs
    driver.Synchronize(time)
    
    # Advance simulation
    rover.GetVehicle().Advance(time_step)
    system.DoStepDynamics(time_step)
    driver.Advance(time_step)
    
    # Render frame
    if frame_time >= frame_step:
        vis.BeginScene()
        vis.Render()
        vis.RenderFrame(chrono.ChVector3d(0,0,0), chrono.QUNIT, 1.0)  # Reference frame
        vis.EndScene()
        frame_time = 0

        # Display rover stats
        pos = rover.GetChassis().GetPos()
        vel = rover.GetChassis().GetPosDt()
        vis.GetGUIEnvironment().addStaticText(
            f"Position: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}) m\n"
            f"Velocity: ({vel.x:.2f}, {vel.y:.2f}, {vel.z:.2f}) m/s\n"
            f"Steering: {driver.steering:.2f} rad",
            chronoirr.recti(10, 10, 300, 80)
        )