import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_MESH


tire_model = veh.TireModelType_TMEASY


terrain_patches = [
    {"name": "patch1", "texture": "terrain/textures/tile1.jpg", "height_map": "", "bump": 0.0},
    {"name": "patch2", "texture": "terrain/textures/tile2.jpg", "height_map": "", "bump": 0.0},
    {"name": "patch3", "texture": "terrain/textures/tile3.jpg", "height_map": "", "bump": 0.0},
    {"name": "patch4", "texture": "terrain/textures/tile4.jpg", "height_map": "terrain/heightmaps/heightmap1.png", "bump": 0.5}
]

terrainHeight = 0      
terrainLength = 100.0  
terrainWidth = 100.0   


trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)


contact_method = chrono.ChContactMethod_NSC
contact_vis = False


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50  


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


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())

for patch in terrain_patches:
    coordsys = chrono.ChCoordsysd(chrono.ChVector3d(0, 0, patch['bump']), chrono.QUNIT)
    patch_obj = terrain.AddPatch(patch_mat, coordsys, terrainLength, terrainWidth)
    patch_obj.SetTexture(veh.GetDataFile(patch['texture']), 200, 200)
    patch_obj.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    if patch['height_map']:
        patch_obj.SetHeightMap(veh.GetDataFile(patch['height_map']))

terrain.Initialize()