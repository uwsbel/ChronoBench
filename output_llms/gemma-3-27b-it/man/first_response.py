import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import math

# --- Simulation setup ---
chrono.SetChronoDataPath("./chrono_data/")  # Set the path to the Chrono data directory

# Create a Chrono physical system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravity

# --- Vehicle model ---
# Load the MAN 10t truck vehicle model
vehicle = veh.ChVehicleMAN10T(system)

# Set initial vehicle position and orientation
vehicle.SetPos(chrono.ChVectorD(0, 1, 0))
vehicle.SetTransform(chrono.ChFrameD(chrono.ChVectorD(0, 1, 0), chrono.ChQuaternionD(1, 0, 0, 0)))

# Configure visualization settings
vehicle.SetCollidable(True)
vehicle.SetVisualize(True)
vehicle.SetVehicleDebugCollision(False)  # Set to True for debugging collision shapes

# --- Terrain ---
# Create a rigid terrain
terrain = chrono.ChBodyEasy()
terrain.SetBodyFixed(True)
terrain.SetMass(0)
terrain.SetPos(chrono.ChVectorD(0, -0.1, 0))
terrain.SetShapeBoxes(0.2, 0.05, 2)  # Create a flat terrain
terrain.CreateMaterialAndTexture(chrono.ChMaterialSurfaceNSC(), "terrain.png")
system.Add(terrain)

# --- Tire model ---
# Use the TMEASY tire model
vehicle.SetTireFormula(veh.ChTireTMEasy())

# --- Driver controls ---
# Create a simple driver controller
class MyDriver:
    def __init__(self):
        self.throttle = 0.0
        self.steering = 0.0
        self.braking = 0.0

    def Drive(self, vehicle, time_step):
        # Simple keyboard controls (example)
        if chrono.ChTime() < 5:
            self.throttle = 0.5
            self.steering = 0.0
            self.braking = 0.0
        elif chrono.ChTime() >= 5 and chrono.ChTime() < 10:
            self.throttle = 0.5
            self.steering = 0.2
            self.braking = 0.0
        else:
            self.throttle = 0.0
            self.steering = 0.0
            self.braking = 0.5

        vehicle.SetThrottle(self.throttle)
        vehicle.SetSteering(self.steering)
        vehicle.SetBraking(self.braking)

driver = MyDriver()

# --- Visualization ---
# Create an Irrlicht application
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('MAN 10t Truck Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 2, -5), chrono.ChVectorD(0, 1, 0))
vis.AddTypicalLights()
vis.AddSkyBox()

# Customize terrain texture and logo
vis.GetSceneManager().addTexture("terrain.png", "terrain.png")
vis.GetSceneManager().addLogo("chrono_logo.png")

# Set chase camera mode
vis.SetChaseCameraMode()

# --- Simulation loop ---
time_step = 0.01
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    driver.Drive(vehicle, time_step)
    system.DoStepDynamics(time_step)