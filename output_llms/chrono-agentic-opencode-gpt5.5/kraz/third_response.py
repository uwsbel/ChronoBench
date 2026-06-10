"""KRAZ tractor-trailer and sedan on a rigid highway mesh.

The model uses the NSC vehicle wrappers on a shared Bullet collision system. A
KRAZ tractor-trailer is initialized at an angled highway pose, a BMW sedan is
initialized on the same rigid mesh terrain, and a scripted sedan driver applies
fixed throttle and steering while the truck and trailer states are recorded.
"""

import math

import pychrono.core as chrono
import pychrono.vehicle as veh


# === Parameters === named constants keep the scenario reproducible
STEP_SIZE = 2.0e-3
TIRE_STEP_SIZE = STEP_SIZE
SIM_END = 6.0
RENDER_FPS = 20.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

TRUCK_INIT_POS = chrono.ChVector3d(-18.0, -1.8, 0.95)
TRUCK_INIT_YAW = math.radians(6.0)
TRUCK_INIT_ROT = chrono.QuatFromAngleAxis(TRUCK_INIT_YAW, chrono.ChVector3d(0, 0, 1))
SEDAN_INIT_POS = chrono.ChVector3d(-27.0, 2.1, 0.55)
SEDAN_INIT_YAW = math.radians(1.0)
SEDAN_INIT_ROT = chrono.QuatFromAngleAxis(SEDAN_INIT_YAW, chrono.ChVector3d(0, 0, 1))

SEDAN_THROTTLE = 0.45
SEDAN_STEERING = 0.04
SEDAN_BRAKING = 0.0


class FixedSedanDriver(veh.ChDriver):
    """Simple driver system that holds the sedan inputs constant."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        self.SetThrottle(SEDAN_THROTTLE)
        self.SetSteering(SEDAN_STEERING)
        self.SetBraking(SEDAN_BRAKING)


def main():
    # === Vehicle data and truck setup === catalog vehicle paths and KRAZ wrapper
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

    truck = veh.Kraz()
    truck.SetContactMethod(chrono.ChContactMethod_NSC)
    truck.SetChassisCollisionType(veh.CollisionType_NONE)
    truck.SetChassisFixed(False)
    truck.SetInitPosition(chrono.ChCoordsysd(TRUCK_INIT_POS, TRUCK_INIT_ROT))
    truck.SetTireStepSize(TIRE_STEP_SIZE)  # KRAZ wrapper uses its rigid tire model
    truck.Initialize()

    system = truck.GetSystem()  # cache: wrapper-owned system reused by terrain and sedan
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    tractor = truck.GetTractor()  # cache: tractor vehicle handle reused for vis and logs
    tractor_body = truck.GetTractorChassisBody()  # cache: tractor chassis state source
    trailer = truck.GetTrailer()  # cache: trailer wrapper reused for logs
    trailer_body = trailer.GetChassis().GetBody()  # cache: trailer chassis state source
    print("VEHICLE MASS: ", tractor.GetMass())

    # === Sedan setup === second vehicle shares the KRAZ system
    sedan = veh.BMW_E90(system)
    sedan.SetContactMethod(chrono.ChContactMethod_NSC)
    sedan.SetChassisCollisionType(veh.CollisionType_NONE)
    sedan.SetChassisFixed(False)
    sedan.SetInitPosition(chrono.ChCoordsysd(SEDAN_INIT_POS, SEDAN_INIT_ROT))
    sedan.SetTireType(veh.TireModelType_RIGID)
    sedan.SetTireStepSize(TIRE_STEP_SIZE)
    sedan.Initialize()
    sedan_body = sedan.GetChassisBody()  # cache: sedan chassis state source

    truck.SetChassisVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_MESH)
    truck.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES, veh.VisualizationType_PRIMITIVES)
    truck.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    truck.SetWheelVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_MESH)
    truck.SetTireVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_MESH)
    sedan.SetChassisVisualizationType(veh.VisualizationType_MESH)
    sedan.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    sedan.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    sedan.SetWheelVisualizationType(veh.VisualizationType_MESH)
    sedan.SetTireVisualizationType(veh.VisualizationType_MESH)

    # === Terrain === rigid highway mesh with NSC contact material
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    terrain = veh.RigidTerrain(system)
    highway_patch = terrain.AddPatch(
        patch_mat,
        chrono.CSYSNORM,
        veh.GetDataFile("terrain/meshes/Highway_col.obj"),
    )
    highway_patch.SetTexture(veh.GetDataFile("terrain/textures/asphalt.jpg"), 80, 80)
    highway_visual = chrono.ChVisualShapeModelFile()
    highway_visual.SetFilename(veh.GetDataFile("terrain/meshes/Highway_vis.obj"))
    highway_patch.GetGroundBody().AddVisualShape(highway_visual)
    terrain.Initialize()

    # === Visualization and drivers === Irrlicht vehicle visualizer plus two drivers
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("KRAZ and Sedan on Highway Mesh")
    vis.SetWindowSize(1280, 720)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.8), 12.0, 1.0)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddLightDirectional()
    vis.AttachVehicle(tractor)

    truck_driver = veh.ChInteractiveDriverIRR(vis)
    truck_driver.SetSteeringDelta((1.0 / RENDER_FPS) / 1.0)
    truck_driver.SetThrottleDelta((1.0 / RENDER_FPS) / 1.0)
    truck_driver.SetBrakingDelta((1.0 / RENDER_FPS) / 0.3)
    truck_driver.Initialize()
    sedan_driver = FixedSedanDriver(sedan.GetVehicle())
    sedan_driver.Initialize()

    realtime_timer = chrono.ChRealtimeStepTimer()
    truck_state_log = []
    trailer_state_log = []

    # === Main loop === synchronize both vehicles, record truck and trailer state
    try:
        while vis.Run() and system.GetChTime() < SIM_END:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

            for _ in range(RENDER_EVERY):
                sim_time = system.GetChTime()
                truck_inputs = truck_driver.GetInputs()
                sedan_driver.Synchronize(sim_time)
                sedan_inputs = sedan_driver.GetInputs()
                truck_state_log.append((sim_time, tractor_body.GetPos(), tractor_body.GetRot()))
                trailer_state_log.append((sim_time, trailer_body.GetPos(), trailer_body.GetRot()))


                truck_driver.Synchronize(sim_time)
                terrain.Synchronize(sim_time)
                truck.Synchronize(sim_time, truck_inputs, terrain)
                sedan.Synchronize(sim_time, sedan_inputs, terrain)
                vis.Synchronize(sim_time, truck_inputs)

                truck_driver.Advance(STEP_SIZE)
                sedan_driver.Advance(STEP_SIZE)
                terrain.Advance(STEP_SIZE)
                truck.Advance(STEP_SIZE)
                sedan.Advance(STEP_SIZE)
                vis.Advance(STEP_SIZE)

                if system.GetChTime() >= SIM_END:
                    break

            realtime_timer.Spin(STEP_SIZE)
    except (RuntimeError, ValueError) as exc:  # solver divergence or invalid vehicle state
        print(f"simulation failed: {exc}")
        raise
    except (OSError, IOError) as exc:  # output path or file-system failure
        print(f"output failure: {exc}")
        raise
    finally:
        pass


if __name__ == "__main__":
    main()
