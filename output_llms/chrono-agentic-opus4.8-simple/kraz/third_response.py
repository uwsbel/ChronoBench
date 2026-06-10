import math
import os
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                    # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')               # locate bundled vehicle data

step_size = 2e-3                                                        # integration step (s)
tire_step_size = 1e-3                                                   # tire substep (s)
sim_end = 12.0                                                          # total sim duration (s)

truck_init_loc = chrono.ChVector3d(-3.0, -40.0, 1.0)                   # truck start (highway runs along +Y, lane at x=-3)
truck_init_rot = chrono.QuatFromAngleZ(math.pi / 2)                    # truck heading: +Y (down the highway)
sedan_init_loc = chrono.ChVector3d(3.0, -35.0, 0.8)                    # sedan start (adjacent lane at x=+3)
sedan_init_rot = chrono.QuatFromAngleZ(math.pi / 2)                    # sedan heading: +Y

sys = chrono.ChSystemNSC()                                             # NSC system shared by both vehicles
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)       # Bullet collision (contact scene)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))       # g = 9.81 down
sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)                # stable iterative solver for vehicle contact
sys.GetSolver().AsIterative().SetMaxIterations(150)                    # solver iteration cap
sys.SetMaxPenetrationRecoverySpeed(4.0)                                # limit contact recovery speed

terrain = veh.RigidTerrain(sys)                                        # rigid terrain on the shared system
patch_mat = chrono.ChContactMaterialNSC()                             # NSC contact material for the road
patch_mat.SetFriction(0.9)                                            # road friction
patch_mat.SetRestitution(0.01)                                        # near-zero bounce
highway_mesh = veh.GetDataFile("terrain/meshes/Highway_col.obj")      # predefined highway collision mesh
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, highway_mesh)    # add the highway mesh patch
vis_mesh = chrono.ChTriangleMeshConnected()                           # load the highway visual mesh
vis_mesh.LoadWavefrontMesh(veh.GetDataFile("terrain/meshes/Highway_vis.obj"), True, True)
vis_shape = chrono.ChVisualShapeTriangleMesh()                        # visual shape wrapping the mesh
vis_shape.SetMesh(vis_mesh)                                           # bind the loaded mesh
vis_shape.SetMutable(False)                                           # static visual geometry
patch.GetGroundBody().AddVisualShape(vis_shape, chrono.ChFramed())   # attach highway visuals to the ground body
terrain.Initialize()                                                  # build the terrain

truck = veh.Kraz(sys)                                                 # Kraz tractor-trailer on the shared system
truck.SetChassisFixed(False)                                         # MANDATORY — fixed chassis won't move
truck.SetInitPosition(chrono.ChCoordsysd(truck_init_loc, truck_init_rot))  # truck spawn pose
truck.SetTireStepSize(tire_step_size)                               # tractor/trailer tire substep
truck.Initialize()                                                  # build the tractor + trailer assembly

rigid_tire_json = veh.GetDataFile("generic/tire/RigidTire.json")    # rigid tire definition
tractor = truck.GetTractor()                                        # tractor vehicle handle (carries the tires)
for axle in range(tractor.GetNumberAxles()):                        # re-fit every tractor wheel with a rigid tire
    for wheel in tractor.GetAxle(axle).GetWheels():
        rigid_tire = veh.ReadTireJSON(rigid_tire_json)              # fresh rigid tire per wheel
        tractor.InitializeTire(rigid_tire, wheel,                   # replace TMeasy with rigid model
                               veh.VisualizationType_MESH,
                               veh.ChTire.CollisionType_SINGLE_POINT)

truck.SetChassisVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_MESH)      # tractor + trailer mesh
truck.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
truck.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES, veh.VisualizationType_PRIMITIVES)
truck.SetWheelVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_MESH)         # wheels mesh
truck.SetTireVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_MESH)          # tires mesh

sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)    # keep Bullet after vehicle init (contact scene)

sedan = veh.BMW_E90(sys)                                            # second vehicle (sedan) on the shared system
sedan.SetChassisFixed(False)                                       # sedan chassis free to move
sedan.SetInitPosition(chrono.ChCoordsysd(sedan_init_loc, sedan_init_rot))  # sedan spawn pose
sedan.SetTireType(veh.TireModelType_TMEASY)                        # sedan rides on TMeasy tires
sedan.SetTireStepSize(tire_step_size)                             # sedan tire substep
sedan.Initialize()                                                # build the sedan
sedan.SetChassisVisualizationType(veh.VisualizationType_MESH)     # sedan mesh visuals
sedan.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
sedan.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
sedan.SetWheelVisualizationType(veh.VisualizationType_MESH)
sedan.SetTireVisualizationType(veh.VisualizationType_MESH)

truck_driver_data = veh.vector_Entry([                            # scripted forward maneuver for the truck
    veh.DataDriverEntry(0.0, 0.0, 0.0, 0.0),                       # (time, steering, throttle, braking)
    veh.DataDriverEntry(0.5, 0.0, 1.0, 0.0),                       # ramp to full throttle quickly
    veh.DataDriverEntry(sim_end, 0.0, 1.0, 0.0),                   # hold full throttle straight ahead
])
truck_driver = veh.ChDataDriver(tractor, truck_driver_data)       # truck driver acts on the tractor
truck_driver.Initialize()                                         # initialize truck driver

sedan_throttle = 0.5                                              # sedan fixed forward throttle
sedan_steering = 0.0                                              # sedan fixed (straight) steering
sedan_driver_data = veh.vector_Entry([                            # sedan moves forward with fixed inputs
    veh.DataDriverEntry(0.0, sedan_steering, sedan_throttle, 0.0),
    veh.DataDriverEntry(sim_end, sedan_steering, sedan_throttle, 0.0),
])
sedan_driver = veh.ChDataDriver(sedan.GetVehicle(), sedan_driver_data)  # sedan driver
sedan_driver.Initialize()                                         # initialize sedan driver

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                  # vehicle Irrlicht visualization
vis.SetWindowTitle("Kraz truck and sedan on highway")            # window title
vis.SetWindowSize(1280, 1024)                                    # window size
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 12.0, 1.0)  # chase camera following the truck
vis.Initialize()                                                # build the Irrlicht device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # PyChrono logo
vis.AddSkyBox()                                                  # sky box
vis.AddLightDirectional()                                       # directional light (vehicle truths use this)
vis.AttachVehicle(truck.GetTractor())                           # bind the tractor visuals to the window

print("TRACTOR MASS: ", truck.GetTractor().GetMass())                          # report tractor mass
print("TRAILER MASS: ", truck.GetTrailer().GetChassis().GetBody().GetMass())   # report trailer chassis mass

render_step_size = 1.0 / 50.0                                   # 50 fps render cadence
render_steps = math.ceil(render_step_size / step_size)         # physics steps per rendered frame

render_fps = 30.0                                                                # review video fps
render_every = max(1, round(1.0 / (render_fps * step_size)))                     # untagged cadence constant

log_info = True                                                 # fire the truck-state log once
realtime_timer = chrono.ChRealtimeStepTimer()                  # spin to keep wall-clock near sim time
step_number = 0                                                # physics step counter
while vis.Run() and sys.GetChTime() < sim_end:
    time = sys.GetChTime()                                     # current sim time

    if step_number % render_steps == 0:                        # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    truck_inputs = truck_driver.GetInputs()                    # truck driver inputs
    sedan_inputs = sedan_driver.GetInputs()                    # sedan driver inputs

    truck_driver.Synchronize(time)                             # update truck driver
    sedan_driver.Synchronize(time)                             # update sedan driver
    truck.Synchronize(time, truck_inputs, terrain)             # update truck against terrain
    sedan.Synchronize(time, sedan_inputs, terrain)             # update sedan against terrain
    terrain.Synchronize(time)                                  # update terrain
    vis.Synchronize(time, truck_inputs)                        # update visualization

    truck_driver.Advance(step_size)                            # advance truck driver
    sedan_driver.Advance(step_size)                            # advance sedan driver
    truck.Advance(step_size)                                   # advance truck subsystems
    sedan.Advance(step_size)                                   # advance sedan subsystems
    terrain.Advance(step_size)                                 # advance terrain
    vis.Advance(step_size)                                     # advance visualization

    sys.DoStepDynamics(step_size)                              # step the shared system (both vehicles) once


    if log_info and time > 2.0:                                # store/report truck tractor + trailer state once
        tr_pos = tractor.GetChassisBody().GetPos()             # tractor chassis position
        tr_vel = tractor.GetChassisBody().GetPosDt()           # tractor chassis velocity
        trl_pos = truck.GetTrailer().GetChassis().GetBody().GetPos()  # trailer chassis position
        print("t = ", time)
        print("  tractor pos: ", tr_pos.x, "  ", tr_pos.y, "  ", tr_pos.z)
        print("  tractor vel: ", tr_vel.x, "  ", tr_vel.y, "  ", tr_vel.z)
        print("  trailer pos: ", trl_pos.x, "  ", trl_pos.y, "  ", trl_pos.z)
        log_info = False                                       # disable further logging

    step_number += 1                                           # advance step counter
    realtime_timer.Spin(step_size)                             # spin in place to match wall-clock
