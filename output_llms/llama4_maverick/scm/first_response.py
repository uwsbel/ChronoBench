import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import numpy as np

# =============================================================================
# Simulation parameters
# =============================================================================

# Output directory
out_dir = "HMMWV_SCM_DEMO"

# Simulation step size
step_size = 2e-3

# Time interval between two render frames
render_step_size = 1.0 / 50  # 50 FPS

# =============================================================================
# Create the vehicle system
# =============================================================================

# Create the HMMWV vehicle
init_loc = chrono.ChVectorD(0, 0, 1.0)
init_orient = chrono.Q_from_AngZ(0)

vehicle = veh.HMMWV_Full(
    init_loc,
    init_orient,
    "HMMWV",
    veh.HMMWV_VehicleType::kNone,
    veh.HMMWV_TireType::kRigid,
    True,
)

# Set the tire model to use mesh visualization
tire_vis_type = veh.VisualizationType_MESH

# Disable the powertrain visualization
powertrain_vis_type = veh.VisualizationType_NONE

# Set the visualization type for the vehicle components
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(tire_vis_type)

# Initialize the vehicle
vehicle.Initialize()

# =============================================================================
# Create the terrain
# =============================================================================

# SCM terrain parameters
terrain_params = veh.SCMDeformableTerrain::Params()
terrain_params.setSoilParameters(
    2e6,   # Bekker Kphi
    0,     # Bekker Kc
    1.0,   # Bekker n exponent
    0,     # Mohr-Coulomb cohesion (C)
    30,    # Mohr-Coulomb friction angle (deg)
    1000,  # Janosi shear coefficient (K)
    2e8,   # Elastic stiffness (k)
    3e4    # Damping (d)
)

terrain = veh.SCMDeformableTerrain(vehicle.GetSystem())
terrain.SetSoilParameters(
    terrain_params.Kphi,
    terrain_params.Kc,
    terrain_params.n,
    terrain_params.C,
    terrain_params.phi,
    terrain_params.K,
    terrain_params.elastic_K,
    terrain_params.damping_R,
)

# Enable moving patch feature
terrain.EnableMovingPatch(vehicle.GetChassis(), init_loc, 5, 5, 0.2)

# Set up the visualization of sinkage
terrain.EnableVisualization(True)
terrain.EnableVisualizationSinkage(True)

# =============================================================================
# Create the Irrlicht application
# =============================================================================

# Create the Irrlicht visualization
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV SCM Demo")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 1.75), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddLightDirectional()
vis.AttachVehicle(vehicle.GetVehicle())

# =============================================================================
# Driver inputs
# =============================================================================

# Create the interactive driver system
driver = veh.ChIrrGuiDriver(vis)

# Set the initial driver inputs
driver.SetSteering(0)
driver.SetThrottle(0)
driver.SetBraking(0)

# =============================================================================
# Simulation loop
# =============================================================================

# Simulation loop
realtime_timer = chrono.ChRealtimeStepTimer()
while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Driver inputs
    driver_inputs = driver.GetInputs()
    vehicle.SetDriverInputs(driver_inputs)

    # Update the vehicle
    vehicle.Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time)

    # Update the visualization
    vis.Synchronize("", driver_inputs)

    # Advance simulation
    vehicle.Advance(step_size)
    terrain.Advance(step_size)

    # Spin in place for real time
    realtime_timer.Spin(step_size)

# Execute the script
if __name__ == "__main__":
    main()

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import numpy as np

def main():
    # =============================================================================
    # Simulation parameters
    # =============================================================================

    # Output directory
    out_dir = "HMMWV_SCM_DEMO"

    # Simulation step size
    step_size = 2e-3

    # Time interval between two render frames
    render_step_size = 1.0 / 50  # 50 FPS

    # =============================================================================
    # Create the vehicle system
    # =============================================================================

    # Create the HMMWV vehicle
    init_loc = chrono.ChVectorD(0, 0, 1.0)
    init_orient = chrono.Q_from_AngZ(0)

    vehicle = veh.HMMWV_Full(
        init_loc,
        init_orient,
        "HMMWV",
        veh.HMMWV_VehicleType::kNone,
        veh.HMMWV_TireType::kRigid,
        True,
    )

    # Set the tire model to use mesh visualization
    tire_vis_type = veh.VisualizationType_MESH

    # Disable the powertrain visualization
    powertrain_vis_type = veh.VisualizationType_NONE

    # Set the visualization type for the vehicle components
    vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetTireVisualizationType(tire_vis_type)

    # Initialize the vehicle
    vehicle.Initialize()

    # =============================================================================
    # Create the terrain
    # =============================================================================

    # SCM terrain parameters
    terrain_params = veh.SCMDeformableTerrain::Params()
    terrain_params.setSoilParameters(
        2e6,   # Bekker Kphi
        0,     # Bekker Kc
        1.0,   # Bekker n exponent
        0,     # Mohr-Coulomb cohesion (C)
        30,    # Mohr-Coulomb friction angle (deg)
        1000,  # Janosi shear coefficient (K)
        2e8,   # Elastic stiffness (k)
        3e4    # Damping (d)
    )

    terrain = veh.SCMDeformableTerrain(vehicle.GetSystem())
    terrain.SetSoilParameters(
        terrain_params.Kphi,
        terrain_params.Kc,
        terrain_params.n,
        terrain_params.C,
        terrain_params.phi,
        terrain_params.K,
        terrain_params.elastic_K,
        terrain_params.damping_R,
    )

    # Enable moving patch feature
    terrain.EnableMovingPatch(vehicle.GetChassis(), init_loc, 5, 5, 0.2)

    # Set up the visualization of sinkage
    terrain.EnableVisualization(True)
    terrain.EnableVisualizationSinkage(True)

    # =============================================================================
    # Create the Irrlicht application
    # =============================================================================

    # Create the Irrlicht visualization
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("HMMWV SCM Demo")
    vis.SetWindowSize(1280, 720)
    vis.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 1.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddLightDirectional()
    vis.AttachVehicle(vehicle.GetVehicle())

    # =============================================================================
    # Driver inputs
    # =============================================================================

    # Create the interactive driver system
    driver = veh.ChIrrGuiDriver(vis)

    # Set the initial driver inputs
    driver.SetSteering(0)
    driver.SetThrottle(0)
    driver.SetBraking(0)

    # =============================================================================
    # Simulation loop
    # =============================================================================

    # Simulation loop
    realtime_timer = chrono.ChRealtimeStepTimer()
    while vis.Run():
        time = vehicle.GetSystem().GetChTime()

        # Driver inputs
        driver_inputs = driver.GetInputs()
        vehicle.SetDriverInputs(driver_inputs)

        # Update the vehicle
        vehicle.Synchronize(time, driver_inputs, terrain)
        terrain.Synchronize(time)

        # Update the visualization
        vis.Synchronize("", driver_inputs)

        # Advance simulation
        vehicle.Advance(step_size)
        terrain.Advance(step_size)

        # Spin in place for real time
        realtime_timer.Spin(step_size)

if __name__ == "__main__":
    main()