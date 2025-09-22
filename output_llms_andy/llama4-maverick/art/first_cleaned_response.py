import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


chrono.SetChronoDataPath("/path/to/chrono/data/")


init_loc = chrono.ChVectorD(0, 0.5, -2.0)
init_orient = chrono.ChQuaternionD(1, 0, 0, 0)
contact_method = chrono.ChContactMethod_NSC
vehicle = veh.ARTcar(init_loc, init_orient, contact_method)


vehicle.SetContactMethod(contact_method)
vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetChassisCollideType(veh.CollisionType_NONE)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)


vehicle.Initialize()


terrain = veh.RigidTerrain(vehicle.GetSystem())
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain_mat = patch_mat
dim = chrono.ChVectorD(100, 1, 100)
loc = chrono.ChVectorD(0, -0.5, 0)
terrain.ConstructPatch(terrain_mat, dim, loc)


img_file = chrono.GetChronoDataFile("textures/grass.jpg")
terrain.SetTexture(veh.VisualizationType_MESH, img_file, 100, 100)


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('ARTcar Demo')
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 1.75), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AttachVehicle(vehicle.GetVehicle())


driver = veh.ChIrrGuiDriver(vis)
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)


while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    driver_inputs = driver.GetInputs()
    vehicle.Advance(time)
    vis.Advance(0.02)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    driver.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain.GetGroundHeight(chrono.ChVectorD(0, 0, 0)))