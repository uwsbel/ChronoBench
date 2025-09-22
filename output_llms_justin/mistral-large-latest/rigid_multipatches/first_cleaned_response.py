import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.sensor as sens


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


system = chrono.ChSystemNSC()
vehicle = veh.WheeledVehicle(system, veh.VehicleSide('LEFT'))



vehicle.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.ChQuaternionD(1, 0, 0, 0)))
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)


terrain = veh.RigidTerrain(system)


mterrain = veh.RigidTerrain(system)
patch_data = [
    (chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0), 100, 100),  
    (chrono.ChVectorD(10, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0), 50, 50),   
    (chrono.ChVectorD(-10, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0), 60, 60),  
]


for pos, rot, width, length in patch_data:
    patch = terrain.AddPatch(chrono.ChCoordsysD(pos, rot), width, length)
    patch.SetTexture(chrono.GetChronoDataFile("textures/terrain/grass.jpg"))


height_map = veh.HeightMapTerrain(system)
height_map.Initialize(chrono.GetChronoDataPath() + "terrain/height_maps/test.bmp", 100, 100, 5)



driver = veh.ChDriver(vehicle)

driver.Initialize()


vis = irr.ChIrrApp(vehicle.GetSystem(), "HMMWV Simulation", chrono.ChCoreIrrWizard.Dimensions(800, 600))
vis.AddTypicalSky()
vis.AddTypicalLogo()
vis.AddTypicalCamera(chrono.ChVectorD(0, -6, 2), chrono.ChVectorD(0, 0, 1))
vis.AddTypicalLights()
vis.EnableShadows()


time_step = 0.01


vis.SetTimestep(time_step)

while vis.GetDevice().run():
    vis.BeginScene()
    vis.DrawAll()

    
    driver.Synchronize(time_step)

    
    vis.DoStep()

    vis.EndScene()