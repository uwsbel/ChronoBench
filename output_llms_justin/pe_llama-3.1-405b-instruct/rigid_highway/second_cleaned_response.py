import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math


chrono.SetChronoDataPath('/path/to/chrono/data/')




patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    chrono.GetChronoDataFile('vehicle/terrain/meshes/Highway_col.obj'),
    True, 0.01, False)
vis_mesh = chrono.ChTriangleMeshConnected().CreateFromWavefrontFile(veh.GetDataFile("terrain/meshes/Highway_vis.obj"), True, True)
tri_mesh_shape = chrono.ChVisualShapeTriangleMesh()
tri_mesh_shape.SetMesh(vis_mesh)
tri_mesh_shape.SetMutable(False)
patch.GetGroundBody().AddVisualShape(tri_mesh_shape)


new_patch_mat = chrono.ChContactMaterialNSC()
new_patch_mat.SetFriction(0.8)
new_patch_mat.SetRestitution(0.01)
new_patch = terrain.AddPatch(new_patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, -42, 0), chrono.QUNIT),
    chrono.GetChronoDataFile('vehicle/terrain/meshes/bump.obj'),
    True, 0.01, False)
new_vis_mesh = chrono.ChTriangleMeshConnected().CreateFromWavefrontFile(veh.GetDataFile("terrain/meshes/bump.obj"), True, True)
new_tri_mesh_shape = chrono.ChVisualShapeTriangleMesh()
new_tri_mesh_shape.SetMesh(new_vis_mesh)
new_tri_mesh_shape.SetMutable(False)
new_patch.GetGroundBody().AddVisualShape(new_tri_mesh_shape)
new_patch.GetGroundBody().SetVisualShapeColor(chrono.ChColor(0.5, 0.5, 0.8))
new_patch.GetGroundBody().SetTexture(chrono.GetChronoDataFile('vehicle/terrain/textures/dirt.jpg'), 6.0, 6.0)

terrain.Initialize()