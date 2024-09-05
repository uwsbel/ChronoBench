```python
import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain
# terrain_model = veh.RigidTerrain.BOX
terrainHeight = 0      # terrain height
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Poon chassis tracked by the camera
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create the Gator vehicle, set parameters, and initialize
vehicle = veh.Gator()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)

vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())

# Terrain patches
terrain_patches = [
    veh.RigidTerrainPatch(
        chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
        terrainLength, terrainWidth,
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 0),