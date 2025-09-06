import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_TMEASY



terrainHeight = 0      
terrainLength = 100.0  
terrainWidth = 100.0   
terrainPatches = [
    (chrono.ChVector3d(-50, -50, 0), chrono.QUNIT, terrainLength / 2, terrainWidth / 2, "terrain/textures/tile1.jpg", chrono.ChColor(0.5, 0.3, 0.2)),
    (chrono.ChVector3d(50, -50, 0), chrono.QUNIT, terrainLength / 2, terrainWidth / 2, "terrain/textures/tile2.jpg", chrono.ChColor(0.3, 0.5, 0.2)),
    (chrono.ChVector3d(-50, 50, 0), chrono.QUNIT, terrainLength / 2, terrainWidth / 2, "terrain/textures/tile3.jpg", chrono.ChColor(0.2, 0.3, 0.5)),
    (chrono.ChVector3d(50, 50, 0), chrono.QUNIT, terrainLength / 2, terrainWidth / 2, "terrain/textures/tile4.jpg", chrono.ChColor(0.8, 0.8, 0.5)),
    (chrono.ChVector3d(0, 0, 0), chrono.QUNIT, terrainLength / 2, terrainWidth / 2, "terrain/textures/heightmap.jpg", chrono.ChColor(0.5, 0.5, 0.5), 2.0),  
]


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


terrain = veh.RigidTerrain(vehicle.GetSystem())
for patch_data in terrainPatches:
    position, rotation, length, width, texture, color, bump_height = patch_data
    patch = terrain.AddPatch(chrono.ChContactMaterialNSC(), chrono.ChCoordsysd(position, rotation), length, width)
    patch.SetTexture(veh.GetDataFile(texture), 200, 200)
    patch.SetColor(color)
    if bump_height is not None:
        patch.SetHeightMap(veh.GetDataFile("terrain/heightmaps/heightmap.png"), bump_height)

terrain.Initialize()