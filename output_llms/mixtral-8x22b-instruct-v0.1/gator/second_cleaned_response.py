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


patch1 = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    terrainLength, terrainWidth)
patch1.SetTexture(veh.GetDataFile("terrain/textures/tile1.jpg"), 200, 200)
patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

patch2 = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(50, 0, 0), chrono.QUNIT),
    terrainLength, terrainWidth)
patch2.SetTexture(veh.GetDataFile("terrain/textures/tile2.jpg"), 200, 200)
patch2.SetColor(chrono.ChColor(0.8, 0.5, 0.8))

patch3 = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 50, 0), chrono.QUNIT),
    terrainLength, terrainWidth)
patch3.SetTexture(veh.GetDataFile("terrain/textures/tile3.jpg"), 200, 200)
patch3.SetColor(chrono.ChColor(0.5, 0.8, 0.8))


patch4 = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(-50, -50, 0), chrono.QUNIT),
    terrainLength, terrainWidth)
patch4.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch4.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
patch4.SetHeightMap(veh.GetDataFile("terrain/height_maps/hmap1.bmp"))


bump = chrono.ChFunction_Const(0.5)
patch1.SetBump(bump)
patch2.SetBump(bump)
patch3.SetBump(bump)
patch4.SetBump(bump)

terrain.Initialize()