"""CityBus rigid-terrain demo using PyChrono NSC contact and Irrlicht.

The script initializes a catalog CityBus with an explicit spawn pose and TMeasy
tires on a textured rigid terrain patch. The bus is rendered with mixed mesh and
primitive visualization and is controlled by an Irrlicht interactive driver.
"""

import math

import pychrono.core as chrono
import pychrono.vehicle as veh


# === Constants === prompt parameters and inferred demo defaults
step_size = 1e-3
tire_step_size = step_size
sim_end = 6.0
render_fps = 50.0
render_step_size = 1.0 / render_fps
render_steps = max(1, math.ceil(render_step_size / step_size))  # precomputed once
terrain_length = 200.0
terrain_width = 20.0
terrain_friction = 0.9
terrain_restitution = 0.01
init_loc = chrono.ChVector3d(0.0, 0.0, 0.5)
init_rot = chrono.QUNIT
chase_track_point = chrono.ChVector3d(0.0, 0.0, 1.5)
chase_distance = 24.0
chase_height = 4.0


def build_simulation():
    """Create the CityBus, rigid terrain, visualization, and driver stack."""
    # === Vehicle === catalog wrapper owns the Chrono system and rigid bodies
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

    vehicle = veh.CityBus()
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
    vehicle.SetTireType(veh.TireModelType_TMEASY)  # prompt: explicit tire model
    vehicle.SetTireStepSize(tire_step_size)
    vehicle.Initialize()

    system = vehicle.GetSystem()  # cache: wrapper-owned ChSystemNSC reused below
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    chassis = vehicle.GetChassisBody()  # cache: chassis body reused for logging
    veh_obj = vehicle.GetVehicle()  # cache: vehicle aggregate reused for tire checks
    print("VEHICLE MASS: ", veh_obj.GetMass())
    # wrapper-created bodies: chassis, suspension links, steering links, wheels, and tires
    # wrapper-created joints: suspension and steering constraints inside the CityBus model

    vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

    # === Terrain === flat rigid road patch with custom texture and NSC material
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(terrain_friction)
    patch_mat.SetRestitution(terrain_restitution)
    terrain = veh.RigidTerrain(system)
    patch = terrain.AddPatch(
        patch_mat,
        chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
        terrain_length,
        terrain_width,
    )
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 20)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    spindle_positions = []
    tire_radii = []
    for axle_index in range(veh_obj.GetNumberAxles()):
        for side in (veh.LEFT, veh.RIGHT):
            spindle_positions.append(veh_obj.GetSpindlePos(axle_index, side))
        for wheel in veh_obj.GetAxles()[axle_index].GetWheels():
            tire_radii.append(wheel.GetTire().GetRadius())
    wheel_bottom_z = min(pos.z for pos in spindle_positions) - max(tire_radii)
    terrain_z = terrain.GetHeight(init_loc)
    assert wheel_bottom_z >= terrain_z - 0.10, (
        f"CityBus wheel bottom starts below terrain: {wheel_bottom_z:.3f} vs {terrain_z:.3f}"
    )

    # === Visualization === vehicle-aware Irrlicht chase camera and lighting
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("CityBus on Rigid Terrain")
    vis.SetWindowSize(1280, 720)
    vis.SetChaseCamera(chase_track_point, chase_distance, chase_height)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddLightDirectional()
    vis.AttachVehicle(veh_obj)

    # === Driver === interactive driver controls steering, throttle, and braking
    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta(render_step_size / 1.0)
    driver.SetThrottleDelta(render_step_size / 1.0)
    driver.SetBrakingDelta(render_step_size / 0.3)
    driver.Initialize()

    return vehicle, system, chassis, terrain, vis, driver


def main():
    """Run the real-time vehicle loop."""
    vehicle, system, chassis, terrain, vis, driver = build_simulation()
    realtime_timer = chrono.ChRealtimeStepTimer()
    step_number = 0

    try:

        # === Main loop === render at 50 FPS and advance all vehicle subsystems
        while vis.Run() and system.GetChTime() < sim_end:
            if step_number % render_steps == 0:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()

            for _ in range(render_steps):
                time = system.GetChTime()
                driver.Synchronize(time)
                driver_inputs = driver.GetInputs()  # cache: one input struct reused by sync calls
                terrain.Synchronize(time)
                vehicle.Synchronize(time, driver_inputs, terrain)
                vis.Synchronize(time, driver_inputs)


                driver.Advance(step_size)
                terrain.Advance(step_size)
                vehicle.Advance(step_size)
                vis.Advance(step_size)
                step_number += 1
                realtime_timer.Spin(step_size)
                if system.GetChTime() >= sim_end:
                    break
    except (RuntimeError, ValueError, OSError, IOError) as exc:  # solver or file-system failure
        print(f"CityBus simulation failed: {exc}")
        raise
    finally:
        pass


if __name__ == "__main__":
    main()
