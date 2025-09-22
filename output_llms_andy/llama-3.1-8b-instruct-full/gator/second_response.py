import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() +'vehicle/')

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
patch1 = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(-16, 0, 0), chrono.QUNIT), 
    32, 32)
patch1.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)
patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
patch1.SetBump(veh.GetDataFile("terrain/bump256.bmp"), 10.0, 10.0, 0.05)

patch2 = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(16, 0, 0.15), chrono.QUNIT), 
    32, 32)
patch2.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 6.0, 6.0)
patch2.SetColor(chrono.ChColor(0.8, 0.5, 0.5))
patch2.SetBump(veh.GetDataFile("terrain/bump256.bmp"), 10.0, 10.0, 0.05)

patch3 = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, -42, 0), chrono.QUNIT), 
    64, 32)
patch3.SetTexture(veh.GetDataFile("terrain/textures/asphalt.jpg"), 12.0, 12.0)
patch3.SetColor(chrono.ChColor(0.5, 0.5, 0.8))
patch3.SetBump(veh.GetDataFile("terrain/bump256.bmp"), 12.0, 12.0, 0.06)

patch4 = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, 42, 0), chrono.QUNIT), 
    64, 32)
patch4.SetTexture(veh.GetDataFile("terrain/height_maps/bump64.bmp"), 12.0, 12.0)
patch4.SetColor(chrono.ChColor(1.0, 0.5, 0.5))
patch4.SetBump(veh.GetDataFile("terrain/bump256.bmp"), 12.0, 12.0, 0.06)

terrain
print("error happened with only start ```python")