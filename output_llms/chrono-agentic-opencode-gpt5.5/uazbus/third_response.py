"""UAZ bus on rigid terrain with a fixed transverse box obstacle.

The simulation uses a vehicle-owned NSC system with Bullet collision, a rigid
terrain patch, rigid tires, and a scripted constant throttle of 0.5 so the bus
drives forward and tests mobility over the obstacle.
"""

import math

import pychrono.core as chrono
import pychrono.vehicle as veh


# === Constants === named parameters make the vehicle/obstacle setup explicit
STEP_SIZE = 1e-3
TIRE_STEP_SIZE = 1e-3
SIM_END = 8.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, math.ceil((1.0 / RENDER_FPS) / STEP_SIZE))  # precomputed once

TERRAIN_LENGTH = 80.0
TERRAIN_WIDTH = 20.0
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01

BUS_INIT_POS = chrono.ChVector3d(0.0, 0.0, 0.55)
BUS_INIT_ROT = chrono.QUNIT
OBSTACLE_SIZE = chrono.ChVector3d(0.5, 5.0, 0.2)
OBSTACLE_POS = chrono.ChVector3d(5.0, 0.0, 0.1)
OBSTACLE_DENSITY = 1000.0
THROTTLE_VALUE = 0.5


# === Vehicle and owned system === wrapper creates the NSC system and rigid bodies
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

vehicle = veh.UAZBUS()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(BUS_INIT_POS, BUS_INIT_ROT))
vehicle.SetTireType(veh.TireModelType_RIGID)  # prompt: rigid tire model
vehicle.SetTireStepSize(TIRE_STEP_SIZE)
vehicle.Initialize()

system = vehicle.GetSystem()  # cache: vehicle-owned system reused by terrain and obstacle
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetSolverType(chrono.ChSolver.Type_PSOR)
system.GetSolver().AsIterative().SetMaxIterations(80)

chassis = vehicle.GetChassisBody()  # cache: fetched once, reused every loop
veh_model = vehicle.GetVehicle()  # cache: mass, speed, and spindle queries
# wrapper-created bodies include chassis, suspensions, steering, wheels, tires, and driveline joints
print("VEHICLE MASS: ", veh_model.GetMass())

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)


# === Terrain and obstacle === rigid road plus fixed collision box across the lane
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(TERRAIN_FRICTION)
patch_mat.SetRestitution(TERRAIN_RESTITUTION)

terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 40)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

obstacle = chrono.ChBodyEasyBox(
    OBSTACLE_SIZE.x,
    OBSTACLE_SIZE.y,
    OBSTACLE_SIZE.z,
    OBSTACLE_DENSITY,
    True,
    True,
    patch_mat,
)
obstacle.SetName("fixed_box_obstacle")
obstacle.SetPos(OBSTACLE_POS)
obstacle.SetFixed(True)
system.AddBody(obstacle)
system.GetCollisionSystem().BindAll()


# === Driver === scripted constant throttle drives the bus forward
driver = veh.ChDriver(veh_model)
driver.SetSteering(0.0)
driver.SetThrottle(THROTTLE_VALUE)
driver.SetBraking(0.0)
driver.Initialize()


# === Visualization === vehicle-aware Irrlicht window and chase camera
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("UAZ bus rigid-tire obstacle test")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(veh_model)


# === Review files === CSV/log outputs exist only for validation and video review


# === Main loop === synchronize and advance the full vehicle subsystem stack
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        if step_number % RENDER_EVERY == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        sim_time = system.GetChTime()
        driver_inputs = driver.GetInputs()  # cache: one driver input struct per step


        driver.Synchronize(sim_time)
        terrain.Synchronize(sim_time)
        vehicle.Synchronize(sim_time, driver_inputs, terrain)
        vis.Synchronize(sim_time, driver_inputs)

        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        vehicle.Advance(STEP_SIZE)
        vis.Advance(STEP_SIZE)

        step_number += 1
        realtime_timer.Spin(STEP_SIZE)
except (RuntimeError, ValueError) as exc:  # solver divergence / invalid state
    print(f"simulation failed: {exc}")
    raise
finally:
    pass
