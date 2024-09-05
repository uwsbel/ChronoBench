import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as chronovehicle


chrono.SetChronoDataPath("data/")
my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
my_system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)




vehicle = chronovehicle.ChVehicle("HMMWV")
vehicle.SetSystem(my_system)


chassis = chronovehicle.ChBodyEasyBox(
    "chassis", 4.0, 2.0, 1.5, 1000
)  
vehicle.AddChassis(chassis)


wheel_radius = 0.5
wheel_width = 0.2
for i in range(4):
    wheel = chronovehicle.ChWheel(
        "wheel_" + str(i),
        chrono.ChVectorD(0, 0, 0),  
        chrono.ChQuaternionD(1, 0, 0, 0),  
        wheel_radius,
        wheel_width,
        chrono.ChVectorD(0, -1, 0),  
    )
    vehicle.AddWheel(wheel)


terrain = chrono.ChTerrainSCM()
terrain.SetSystem(my_system)
terrain.SetSize(100, 100)  


terrain.SetSoilParameters(
    density=1500,
    friction=0.5,
    cohesion=1000,
    adhesion=0.8,
    restitution=0.2,
)


terrain.SetMovingPatch(True)
terrain.SetPatchSize(5, 5)  




vehicle.SetPos(chrono.ChVectorD(0, 1, 0))
vehicle.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))


vis = chronoirr.ChIrrApp(my_system, "HMMWV Simulation", chronoirr.dimension2du(1280, 720))
vis.AddTypicalSky()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(0, 5, -10))


vis.SetShowMesh(True)
vis.SetShowContacts(True)


terrain.SetVisualizationType(chrono.ChTerrainSCM.VisualizationType_HEIGHTMAP)
terrain.SetSinkageVisualization(True)


driver = chronovehicle.ChDriver(vehicle)
vis.SetDriver(driver)


while vis.GetDevice().run():
    vis.BeginScene()
    vis.Render()
    my_system.DoStepDynamics(0.02)  
    vis.EndScene()