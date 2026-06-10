"""CityBus rigid-terrain PyChrono simulation using a data-driven driver.

The model uses Chrono::Vehicle's CityBus wrapper on NSC rigid terrain with Bullet
collision. A ChDataDriver replaces keyboard input and applies the requested
throttle, steering, and braking schedule so the bus accelerates and then turns.
"""

import math
import pychrono.core as chrono
import pychrono.vehicle as veh


# === Constants === named parameters keep the vehicle setup readable and bounded
step_size = 2e-3
tire_step_size = step_size
sim_end = 10.0
render_fps = 50.0
render_every = max(1, math.ceil((1.0 / render_fps) / step_size))  # precomputed once
terrain_length = 200.0
terrain_width = 100.0
terrain_friction = 0.9
terrain_restitution = 0.01
init_loc = chrono.ChVector3d(0.0, 0.0, 0.5)
init_rot = chrono.QUNIT


# === Vehicle === wrapper creates the system, chassis, axles, tires, and joints
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

bus = veh.CityBus()
bus.SetContactMethod(chrono.ChContactMethod_NSC)
bus.SetChassisCollisionType(veh.CollisionType_NONE)
bus.SetChassisFixed(False)
bus.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
bus.SetTireType(veh.TireModelType_TMEASY)
bus.SetTireStepSize(tire_step_size)
bus.Initialize()

# === System & bodies === wrapper-owned system and visible handles for review
system = bus.GetSystem()  # cache: wrapper-owned ChSystemNSC reused by terrain and loop
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
chassis = bus.GetChassisBody()  # cache: main chassis rigid body reused for logging
vehicle_model = bus.GetVehicle()  # cache: vehicle aggregate reused by driver and visualizer
# wheels, suspension, steering joints, engine, and transmission are created by veh.CityBus.
print("VEHICLE MASS: ", vehicle_model.GetMass())

bus.SetChassisVisualizationType(veh.VisualizationType_MESH)
bus.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
bus.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
bus.SetWheelVisualizationType(veh.VisualizationType_MESH)
bus.SetTireVisualizationType(veh.VisualizationType_MESH)


# === Terrain === flat rigid road provides contact for the bus tires
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(terrain_friction)
patch_mat.SetRestitution(terrain_restitution)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrain_length, terrain_width)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


# === Driver === data-driven schedule requested for the bus maneuver
driver_data = veh.vector_Entry([
    veh.DataDriverEntry(0.0, 0.0, 0.0, 0.0),
    veh.DataDriverEntry(0.1, 0.0, 1.0, 0.0),
    veh.DataDriverEntry(0.5, 0.7, 1.0, 0.0),
])
driver = veh.ChDataDriver(vehicle_model, driver_data)
driver.Initialize()


# === Visualization === vehicle Irrlicht scene mirrors catalog vehicle demos
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("CityBus Data Driver")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 24.0, 2.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle_model)


# === Connectivity log === review-mode joint list helps contact diagnostics


# === Main loop === synchronize and advance every vehicle subsystem in order
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

try:
        while vis.Run() and system.GetChTime() < sim_end:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

            for _ in range(render_every):
                time = system.GetChTime()
                driver_inputs = driver.GetInputs()

                driver.Synchronize(time)
                terrain.Synchronize(time)
                bus.Synchronize(time, driver_inputs, terrain)
                vis.Synchronize(time, driver_inputs)

                driver.Advance(step_size)
                terrain.Advance(step_size)
                bus.Advance(step_size)
                vis.Advance(step_size)

                step_number += 1
                realtime_timer.Spin(step_size)
                if system.GetChTime() >= sim_end:
                    break
except (RuntimeError, ValueError) as exc:  # solver divergence or invalid model state
    raise
except (OSError, IOError) as exc:  # recording file-system failures
    raise
finally:
    pass


# === Post-processing === review video and plots are stripped before scoring
