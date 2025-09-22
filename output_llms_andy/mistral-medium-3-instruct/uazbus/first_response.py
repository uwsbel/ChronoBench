import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.driver as driver

# =============================================================================
# Simulation parameters
# =============================================================================

# Simulation step size
step_size = 1e-3

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Point on chassis tracked by the camera
trackPoint = chrono.ChVectorD(0.0, 0.0, 1.0)

# =============================================================================
# Create the systems
# =============================================================================

# 1. Create a Chrono physical system
sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

# 2. Create the vehicle system
vehicle = veh.UAZBUS(sys)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT))
vehicle.Initialize()

# 3. Create the terrain
terrain = veh.RigidTerrain(sys)
terrain.SetContactFrictionCoefficient(0.9)
terrain.SetContactRestitutionCoefficient(0.1)
terrain.SetContactMaterialProperties(2e7, 0.3)
patch = terrain.AddPatch(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(100, 100, 0),
                         0, 0.5, True)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
terrain.Initialize()

# 4. Create the driver system
driver = driver.ChPathFollowerDriver(vehicle, "path.txt", "steering_controller.json", "speed_controller.json", 0.0)
driver.Initialize()

# =============================================================================
# Visualization system
# =============================================================================

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("UAZBUS Simulation")
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddTypicalLights()
vis.AddSkyBox()
vis.AddLogo()
vis.AttachVehicle(vehicle.GetChassisBody())

# =============================================================================
# Simulation loop
# =============================================================================

# Number of simulation steps between two 3D view render frames
render_steps = int(render_step_size / step_size)

# Initialize simulation frame counter
step_number = 0

while vis.Run():
    time = vehicle.GetChTime()

    # Driver inputs
    driver_input = driver.GetInputs()

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_input, terrain)
    vis.Synchronize(time, driver_input)

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Render scene
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()