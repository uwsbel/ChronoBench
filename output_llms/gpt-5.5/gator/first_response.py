# PyChrono Gator vehicle on rigid flat terrain with Irrlicht visualization
# Controls:
#   Arrow left/right : steering
#   Arrow up         : throttle
#   Arrow down       : braking

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


# ---------------------------------------------------------------------
# Compatibility aliases for different PyChrono versions
# ---------------------------------------------------------------------
ChVector = getattr(chrono, "ChVector3d", chrono.ChVectorD)
ChCoordsys = getattr(chrono, "ChCoordsysd", chrono.ChCoordsysD)


def make_contact_material(contact_method):
    """Create a contact material compatible with the selected Chrono version."""
    if contact_method == chrono.ChContactMethod_NSC:
        if hasattr(chrono, "ChContactMaterialNSC"):
            mat = chrono.ChContactMaterialNSC()
        else:
            mat = chrono.ChMaterialSurfaceNSC()
    else:
        if hasattr(chrono, "ChContactMaterialSMC"):
            mat = chrono.ChContactMaterialSMC()
        else:
            mat = chrono.ChMaterialSurfaceSMC()

    mat.SetFriction(0.9)
    mat.SetRestitution(0.01)
    return mat


# ---------------------------------------------------------------------
# Global simulation parameters
# ---------------------------------------------------------------------
contact_method = chrono.ChContactMethod_NSC

init_location = ChVector(0.0, 0.0, 0.45)
init_orientation = chrono.QUNIT

step_size = 1.0 / 50.0          # 50 Hz dynamics update
render_step_size = 1.0 / 50.0   # 50 FPS rendering
tire_step_size = 1.0 / 1000.0

terrain_length = 200.0
terrain_width = 200.0
terrain_height = 0.0

steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3

# Chrono vehicle data path
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")


# ---------------------------------------------------------------------
# Create and initialize the Gator vehicle
# ---------------------------------------------------------------------
gator = veh.Gator()

gator.SetContactMethod(contact_method)
gator.SetChassisFixed(False)
gator.SetInitPosition(ChCoordsys(init_location, init_orientation))

# TMEASY tire model
gator.SetTireType(veh.TireModelType_TMEASY)
gator.SetTireStepSize(tire_step_size)

gator.Initialize()

# Mesh visualization for all vehicle components
gator.SetChassisVisualizationType(veh.VisualizationType_MESH)
gator.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
gator.SetSteeringVisualizationType(veh.VisualizationType_MESH)
gator.SetWheelVisualizationType(veh.VisualizationType_MESH)
gator.SetTireVisualizationType(veh.VisualizationType_MESH)

system = gator.GetSystem()


# ---------------------------------------------------------------------
# Create flat rigid terrain with custom texture
# ---------------------------------------------------------------------
terrain = veh.RigidTerrain(system)

terrain_mat = make_contact_material(contact_method)

terrain_patch = terrain.AddPatch(
    terrain_mat,
    ChCoordsys(ChVector(0.0, 0.0, terrain_height), chrono.QUNIT),
    terrain_length,
    terrain_width,
)

terrain_patch.SetColor(chrono.ChColor(0.45, 0.55, 0.35))

# Custom terrain texture from Chrono vehicle data directory
terrain_patch.SetTexture(
    veh.GetDataFile("terrain/textures/tile4.jpg"),
    terrain_length / 4.0,
    terrain_width / 4.0,
)

terrain.Initialize()


# ---------------------------------------------------------------------
# Irrlicht visualization system
# ---------------------------------------------------------------------
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("PyChrono Gator on Rigid Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(ChVector(0.0, 0.0, 0.75), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(gator.GetVehicle())


# ---------------------------------------------------------------------
# Interactive driver
# ---------------------------------------------------------------------
driver = veh.ChInteractiveDriverIRR(vis)

driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()


# ---------------------------------------------------------------------
# Real-time simulation loop at 50 FPS
# ---------------------------------------------------------------------
realtime_timer = chrono.ChRealtimeStepTimer()

while vis.Run():
    time = system.GetChTime()

    # Driver inputs
    driver_inputs = driver.GetInputs()

    # Synchronize modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    gator.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Render scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    gator.Advance(step_size)
    vis.Advance(step_size)

    # Maintain real-time execution at 50 Hz
    realtime_timer.Spin(step_size)