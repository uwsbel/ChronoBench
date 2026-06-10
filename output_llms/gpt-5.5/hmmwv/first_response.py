# hmmwv_full_irrlicht.py
# PyChrono simulation: full HMMWV on flat rigid terrain with Irrlicht visualization

import math
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


# ---------------------------------------------------------------------
# Small API compatibility helpers for different PyChrono versions
# ---------------------------------------------------------------------
try:
    ChVector = chrono.ChVector3d
except AttributeError:
    ChVector = chrono.ChVectorD

try:
    ChCoordsys = chrono.ChCoordsysd
except AttributeError:
    ChCoordsys = chrono.ChCoordsysD

try:
    ContactMaterialNSC = chrono.ChContactMaterialNSC
except AttributeError:
    ContactMaterialNSC = chrono.ChMaterialSurfaceNSC


def main():
    # -----------------------------------------------------------------
    # Chrono / vehicle data paths
    # -----------------------------------------------------------------
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

    # -----------------------------------------------------------------
    # Simulation parameters
    # -----------------------------------------------------------------
    contact_method = chrono.ChContactMethod_NSC

    step_size = 1.0e-3
    tire_step_size = 1.0e-3

    render_fps = 50
    render_step_size = 1.0 / render_fps
    render_steps = int(math.ceil(render_step_size / step_size))

    # Initial HMMWV pose
    init_loc = ChVector(0.0, 0.0, 0.7)
    init_yaw = 0.0
    init_rot = chrono.QuatFromAngleZ(init_yaw)

    # Terrain parameters
    terrain_length = 200.0
    terrain_width = 200.0
    terrain_friction = 0.9
    terrain_restitution = 0.01

    # Driver response times
    steering_time = 1.0
    throttle_time = 1.0
    braking_time = 0.3

    # -----------------------------------------------------------------
    # Create and configure the full HMMWV
    # -----------------------------------------------------------------
    hmmwv = veh.HMMWV_Full()

    hmmwv.SetContactMethod(contact_method)
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)

    hmmwv.SetInitPosition(ChCoordsys(init_loc, init_rot))

    # Full vehicle configuration
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)

    # TMEASY tire model
    hmmwv.SetTireType(veh.TireModelType_TMEASY)
    hmmwv.SetTireStepSize(tire_step_size)

    hmmwv.Initialize()

    # Primitive visualization for all vehicle components
    hmmwv.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)

    # -----------------------------------------------------------------
    # Create flat rigid terrain
    # -----------------------------------------------------------------
    terrain = veh.RigidTerrain(hmmwv.GetSystem())

    terrain_mat = ContactMaterialNSC()
    terrain_mat.SetFriction(terrain_friction)
    terrain_mat.SetRestitution(terrain_restitution)

    terrain_pose = ChCoordsys(ChVector(0.0, 0.0, 0.0), chrono.QuatFromAngleZ(0.0))

    patch = terrain.AddPatch(
        terrain_mat,
        terrain_pose,
        terrain_length,
        terrain_width
    )

    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)

    terrain.Initialize()

    # -----------------------------------------------------------------
    # Irrlicht visualization system
    # -----------------------------------------------------------------
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("PyChrono Full HMMWV - Rigid Terrain")
    vis.SetWindowSize(1280, 720)

    # Chase camera target point relative to vehicle chassis
    vis.SetChaseCamera(ChVector(0.0, 0.0, 1.75), 6.0, 0.5)

    vis.AttachVehicle(hmmwv.GetVehicle())
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddLightDirectional()

    # -----------------------------------------------------------------
    # Interactive Irrlicht driver
    # -----------------------------------------------------------------
    driver = veh.ChInteractiveDriverIRR(vis)

    driver.SetSteeringDelta(render_step_size / steering_time)
    driver.SetThrottleDelta(render_step_size / throttle_time)
    driver.SetBrakingDelta(render_step_size / braking_time)

    driver.Initialize()

    # -----------------------------------------------------------------
    # Simulation loop
    # -----------------------------------------------------------------
    realtime_timer = chrono.ChRealtimeStepTimer()
    step_number = 0

    while vis.Run():
        time = hmmwv.GetSystem().GetChTime()

        # Render at 50 FPS
        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        # Get driver inputs
        driver_inputs = driver.GetInputs()

        # Synchronize modules
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        # Advance modules
        driver.Advance(step_size)
        terrain.Advance(step_size)
        hmmwv.Advance(step_size)
        vis.Advance(step_size)

        # Enforce real-time execution
        realtime_timer.Spin(step_size)

        step_number += 1


if __name__ == "__main__":
    main()