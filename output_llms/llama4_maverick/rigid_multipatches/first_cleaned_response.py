import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import numpy as np


print('Initialize PyChrono')


print('Create the vehicle system')
vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVector3d(-5, -5, 0.5), chrono.ChQuaterniond(1, 0, 0, 0)))
vehicle.SetEngineType(veh.EngineModelType_SIMPLE)
vehicle.SetDriveType(veh.DrivetrainModelType_SIMPLE)
vehicle.SetSteeringType(veh.SteeringTypeType_PITMAN_ARM)
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.Initialize()


print('Set visualization for all vehicle components')
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)


print('Create the terrain patches')
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch1_mat = chrono.ChMaterialSurfaceSMC()
patch1 = terrain.AddPatch(patch1_mat, chrono.ChCoordsysD(chrono.ChVector3d(-16, -16, 0), chrono.QUNIT), 32, 32)
patch1.SetTexture(veh.GetDataFile('terrain/textures/dirt.jpg'), 16, 16)
patch1.EnableVisualization(True)

patch2_mat = chrono.ChMaterialSurfaceSMC()
patch2 = terrain.AddPatch(patch2_mat, chrono.ChCoordsysD(chrono.ChVector3d(0, -16, 0), chrono.QUNIT), 32, 32)
patch2.SetTexture(veh.GetDataFile('terrain/textures/concrete.jpg'), 16, 16)
patch2.EnableVisualization(True)


mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh(veh.GetDataFile('terrain/meshes/bump.obj'), True, True)
patch3_mat = chrono.ChMaterialSurfaceSMC()
patch3 = terrain.AddPatch(patch3_mat, chrono.ChCoordsysD(chrono.ChVector3d(-8, 0, 0), chrono.QUNIT), mesh)
patch3.EnableVisualization(True)


br = chrono.ChVector3d(16, 16, 0.3)
ur = chrono.ChVector3d(0, 0, 0)
verts = 100 * np.array([[0, 0, 0], [br.x, 0, 0], [0, br.y, 0], [br.x, br.y, 0]])
hmat = chrono.ChMatrixDynamic(2, 2)
hmat(0, 0) = ur.z
hmat(0, 1) = ur.z
hmat(1, 0) = ur.z
hmat(1, 1) = br.z
patch4_mat = chrono.ChMaterialSurfaceSMC()
patch4 = terrain.AddPatch(patch4_mat, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), hmat, br, 100, 100)
patch4.EnableVisualization(True)

terrain.Initialize()


print('Create the Irrlicht application')
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV on Complex Terrain')
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())


print('Create the interactive driver')
driver = veh.ChIrrGuiDriver(vis.GetDevice(), vehicle.GetVehicle())


print('Simulation loop')
while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    driver_inputs = driver.GetInputs()
    vehicle.SetDriverInputs(driver_inputs)
    vehicle.Advance(0.02)
    vis.Advance(0.02)
    vis.Render()