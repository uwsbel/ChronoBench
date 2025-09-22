import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# -----------------
# Create Chrono system
# -----------------
mesh_system = chrono.ChSystemSMC()

# Create the Irrlicht visualization system
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.71), 6.0, 0.5)
vis.Initialize()
vis.AttachSystem(mesh_system)

# Create the vehicle and set parameters
vehicle = veh.HMMWV_Full()
vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(-8, 0, 0.6), chrono.QUNIT))
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetTireType(veh.TireModelType_RIGID)
vehicle.Initialize()

# Create the SCM terrain
terrain = veh.SCMTerrain(vehicle.GetSystem())
terrain.SetSoilParameters(2e6,   # Bekker Kphi
                            0,     # Bekker Kc
                            1.1,   # Bekker n exponent
                            0,     # Mohr cohesive limit (Pa)
                            30,    # Mohr friction limit (degrees)
                            0.01,  # Janosi shear coefficient (m)
                            2e8,   # Elastic stiffness (Pa/m), before plastic yield
                            3e4    # Damping (Pa s/m), proportional to negative vertical speed (optional)
)
terrain.Initialize(20, 20, 0.02)

# Create the Irrlicht driver system
driver = veh.ChInteractiveDriverIRR(vis)

# Set the time response for steering and throttle keyboard inputs.
steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1
driver.SetSteeringDelta(1.0 / steering_time)
driver.SetThrottleDelta(1.0 / throttle_time)
driver.SetBrakingDelta(1.0 / braking_time)

# ---------------
# Simulation loop
# ---------------

# output vehicle mass
print("VEHICLE MASS: ",  vehicle.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(1.0 / 0.01)

# Initialize simulation frame counter s
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run() :
    time = vehicle.GetSystem().GetChTime()

    # Render scene and output POV-Ray data
    if (step_number % render_steps == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    driver.Advance(0.01)
    terrain.Advance(0.01)
    vehicle.Advance(0.01)
    vis.Advance(0.01)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(0.01)