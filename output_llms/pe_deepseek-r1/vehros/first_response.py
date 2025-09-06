import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.ros as chros
import pychrono.irrlicht as chronoirr
import math

# ======================
# Simulation Parameters
# ======================
step_size = 0.01  # Simulation timestep (s)
end_time = 20.0   # Simulation duration (s)
terrain_length = 100.0  # Terrain length (m)
terrain_width = 8.0     # Terrain width (m)
friction_coeff = 0.8    # Terrain friction coefficient
restitution_coeff = 0.1 # Terrain restitution

# ======================
# System Initialization
# ======================
system = chrono.ChSystemSMC()
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
system.SetSolverMaxIterations(100)

# ======================
# Vehicle Setup
# ======================
# Create HMMWV vehicle
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysD(chrono.ChVector3d(0, 1, 0), chrono.QUNIT))
hmmwv.SetEngineType(veh.EngineModelType_SIMPLE_MAP)
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.Initialize()

# Enable vehicle collision
hmmwv.SetChassisCollision(True)

# Get vehicle subsystems
vehicle = hmmwv.GetVehicle()
tires = [hmmwv.GetTire(i) for i in range(4)]

# ======================
# Terrain Setup
# ======================
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialSMC()
patch_mat.SetFriction(friction_coeff)
patch_mat.SetRestitution(restitution_coeff)
patch = terrain.AddPatch(patch_mat, 
                         chrono.CSYSNORM, 
                         terrain_length, 
                         terrain_width)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
terrain.Initialize()

# ======================
# Driver System Setup
# ======================
driver = veh.ChInteractiveDriverIRR(vehicle)
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)
driver.Initialize()

# ======================
# ROS Integration
# ======================
ros_manager = chros.ChROSMgr()
ros_manager.RegisterHandler(chros.ChROSClockHandler())  # Clock sync
ros_manager.RegisterHandler(chros.ChROSDriverInputsHandler(step_size, driver))  # Driver inputs
ros_manager.RegisterHandler(chros.ChROSVehicleStateHandler(step_size, vehicle))  # Vehicle state

# Initialize ROS
if not ros_manager.Initialize():
    print("ROS initialization failed!")
    exit(1)

# ======================
# Visualization Setup
# ======================
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('HMMWV Simulation with ROS Integration')
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(-8, 0, 1.5), chrono.ChVector3d(0, 0, 1.5))
vis.AddTypicalLights()

# Add vehicle visualization
hmmwv.GetVehicle().SetVisualSystem(vis)

# ======================
# Simulation Loop
# ======================
time = 0.0
while time < end_time:
    # Update ROS
    if not ros_manager.Update(time, step_size):
        print("ROS update failed!")
        break

    # Get driver inputs
    driver_inputs = driver.GetInputs()
    driver.Synchronize(time)
    
    # Update vehicle systems
    vehicle.Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time)
    
    # Advance simulation
    driver.Advance(step_size)
    vehicle.Advance(step_size)
    terrain.Advance(step_size)
    system.DoStepDynamics(step_size)
    
    # Update visualization
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    # Increment simulation time
    time += step_size

print("Simulation completed.")