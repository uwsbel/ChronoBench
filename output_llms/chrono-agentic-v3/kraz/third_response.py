"""
Kraz tractor-trailer truck + BMW E90 sedan on a highway mesh terrain.

System type: ChSystemNSC (NSC contact, rigid terrain).
Main bodies:
  - Kraz tractor-trailer (veh.Kraz) — interactive driver
  - BMW E90 sedan (veh.BMW_E90 sharing the truck system) — scripted driver
    with fixed throttle (0.8) and zero steering, moving forward on the highway.
Terrain: RigidTerrain with a predefined highway mesh (Highway_col.obj collision,
    Highway_vis.obj visual).
Tire model: RIGID for both vehicles.
Expected behavior: the truck drives forward on the highway road; the sedan also
    drives forward at fixed throttle. Tractor and trailer positions/headings are
    logged each step.
"""

import math
import os
import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh


# === Data paths ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Named constants ===
initLoc        = chrono.ChVector3d(7, -60, 0.5)
initRot        = chrono.QuatFromAngleZ(1.57)
initLoc_sedan  = chrono.ChVector3d(3, -65, 0.5)
initRot_sedan  = chrono.QuatFromAngleZ(1.57)

vis_type               = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE
tire_model             = veh.TireModelType_RIGID   # prompt: rigid tire model for truck
contact_method         = chrono.ChContactMethod_NSC

step_size        = 1e-3
tire_step_size   = step_size
sim_end          = 20.0
render_step_size = 1.0 / 20.0   # ~20 FPS render cadence
render_steps     = math.ceil(render_step_size / step_size)  # precomputed once

trackPoint = chrono.ChVector3d(0.0, 0.0, 1.75)


# === Truck (Kraz tractor-trailer) ===
truck = veh.Kraz()
truck.SetContactMethod(contact_method)
truck.SetChassisCollisionType(chassis_collision_type)
truck.SetChassisFixed(False)
truck.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
truck.Initialize()
truck.SetChassisVisualizationType(vis_type, vis_type)
truck.SetSteeringVisualizationType(vis_type)
truck.SetSuspensionVisualizationType(vis_type, vis_type)
truck.SetWheelVisualizationType(vis_type, vis_type)
truck.SetTireVisualizationType(vis_type, vis_type)

# === Sedan (BMW E90) — shares the truck's system ===
sedan = veh.BMW_E90(truck.GetSystem())
sedan.SetContactMethod(contact_method)
sedan.SetChassisCollisionType(chassis_collision_type)
sedan.SetChassisFixed(False)
sedan.SetInitPosition(chrono.ChCoordsysd(initLoc_sedan, initRot_sedan))
sedan.SetTireType(tire_model)
sedan.SetTireStepSize(tire_step_size)
sedan.Initialize()
sedan.SetChassisVisualizationType(vis_type)
sedan.SetSuspensionVisualizationType(vis_type)
sedan.SetSteeringVisualizationType(vis_type)
sedan.SetWheelVisualizationType(vis_type)
sedan.SetTireVisualizationType(vis_type)

# === System & bodies (created by the veh.Kraz wrapper) ===
truck.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system  = truck.GetSystem()               # ChSystemNSC owned by the truck wrapper
chassis = truck.GetTractorChassisBody()   # cache: tractor chassis rigid body
# trailer: truck.GetTrailer().GetChassis().GetBody()
# sedan chassis: sedan.GetChassisBody()
# joints: suspension + steering links created inside wrappers

# === Terrain — predefined highway mesh ===
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(system)
_highway_col = chrono.GetChronoDataFile("synchrono/meshes/Highway_col.obj")
_highway_vis = chrono.GetChronoDataFile("synchrono/meshes/Highway_vis.obj")
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    _highway_col,
    True, 0.01, False
)
vis_mesh = chrono.ChTriangleMeshConnected().CreateFromWavefrontFile(
    _highway_vis, True, True
)
tri_mesh_shape = chrono.ChVisualShapeTriangleMesh()
tri_mesh_shape.SetMesh(vis_mesh)
tri_mesh_shape.SetMutable(False)
patch.GetGroundBody().AddVisualShape(tri_mesh_shape)
terrain.Initialize()

print("VEHICLE MASS: ", truck.GetTractor().GetMass())

# === Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Kraz truck + sedan on highway')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 25.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(truck.GetTractor())

# === Drivers ===
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time  = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

driver_sedan = veh.ChDriver(sedan.GetVehicle())
driver_sedan.Initialize()

# === Review-only setup ===

# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number    = 0

try:
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()

        # Sedan scripted inputs — move forward at fixed throttle, zero steering
        driver_sedan.SetThrottle(0.8)
        driver_sedan.SetSteering(0.0)


        # Render at throttled cadence
        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        # Collect truck state (tractor + trailer)
        tractor_pos     = truck.GetTractorChassisBody().GetPos()   # cache: used this step
        trailer_body    = truck.GetTrailer().GetChassis().GetBody()
        trailer_pos     = trailer_body.GetPos()
        tractor_heading = truck.GetTractorChassisBody().GetRot().GetCardanAnglesZYX().z
        trailer_heading = trailer_body.GetRot().GetCardanAnglesZYX().z


        driver_inputs       = driver.GetInputs()
        driver_inputs_sedan = driver_sedan.GetInputs()

        driver.Synchronize(time)
        driver_sedan.Synchronize(time)
        terrain.Synchronize(time)
        truck.Synchronize(time, driver_inputs, terrain)
        sedan.Synchronize(time, driver_inputs_sedan, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(step_size)
        driver_sedan.Advance(step_size)
        terrain.Advance(step_size)
        truck.Advance(step_size)
        sedan.Advance(step_size)
        vis.Advance(step_size)

        step_number += 1
        realtime_timer.Spin(step_size)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback; traceback.print_exc()
    raise
finally:
    pass
