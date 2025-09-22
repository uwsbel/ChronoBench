import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.vehicle.g.VehicleJson as vehjson
import pychrono.irrlicht as chronoirr

# 1. Initialize the PyChrono environment and core components.
chrono.SetChronoDataPath(chrono.GetChronoDataPath())  # make sure data path is set

# Create the physical system (ChSystem) and set gravity
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

# 2. Add the required physical systems and objects as specified.

# Create a rigid terrain
terrain = veh.RigidTerrain(system)

# Create the patch (flat terrain element)
patch_length = 300.0
patch_width = 300.0
patch_height = 0.0
patch_center = chrono.ChVectorD(0, 0, patch_height)

patch = terrain.AddPatch(patch_center, chrono.QUNIT, patch_length, patch_width)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
patch.SetFriction(0.9)
patch.SetRestitution(0.01)
patch.SetMaterialSurface(0, 0.9, 0.01)  # (optional) friction, restitution

terrain.Initialize()

# Create the Kraz vehicle
# Vehicle initial position and orientation
initLoc = chrono.ChVectorD(0, 0, 1.0)
initRot = chrono.Q_from_AngY(chrono.CH_C_PI_4)  # 45 degrees yaw, example

vehicle = veh.Kraz(vehicle_model_file=None, 
                   fixed=False, 
                   contactMethod=chrono.ChContactMethod_NSC)

# Note: Kraz constructor does not have vehicle_model_file input in the python binding by default,
# so just use default initialization
vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
vehicle.SetSystem(system)
vehicle.Initialize()

# 3. Set necessary parameters: (some defaults are already set above)

# Initialize the driver system for vehicle control
# Use the ChIrrGuiDriver to control via Irrlicht GUI inputs (keyboard)
driver = veh.ChIrrGuiDriver(vehicle.GetVehicle())

# Set steering, throttle, and braking input delay times, and max forces
driver.SetThrottleDelta(0.05)
driver.SetSteeringDelta(0.04)
driver.SetBrakingDelta(0.1)

# Visualization system (Irrlicht)
vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Kraz Vehicle Simulation')
vis.Initialize()
vis.AddTypicalLights()
vis.AddLogo()
vis.AttachVehicle(vehicle.GetVehicle())

# Set camera position and target
cam_pos = chrono.ChVectorD(-10, -10, 5)
cam_target = vehicle.GetVehicle().GetChassisBody().GetPos()
vis.SetCameraPosition(cam_pos, cam_target)

# Simulation parameters
timestep = 0.01

# Simulation loop
while vis.Run():
    # Get driver inputs (from GUI keyboard)
    driver_inputs = driver.GetInputs()

    # Synchronize all modules
    time = system.GetChTime()
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, True, False)

    # Advance simulation by one timestep
    driver.Advance(timestep)
    terrain.Advance(timestep)
    vehicle.Advance(timestep)
    vis.Advance(timestep)

    # Step the physical system
    system.DoStepDynamics(timestep)