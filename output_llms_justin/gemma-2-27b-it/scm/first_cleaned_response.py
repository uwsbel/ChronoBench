import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as chronovehicle


chrono.SetChronoDataPath("data/")
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))



vehicle = chronovehicle.ChVehicle(system)
vehicle.SetChassisFixed(False)
vehicle.SetChassisPosition(chrono.ChVectorD(0, 0.5, 0))
vehicle.SetChassisBody(chrono.ChBodyEasyBox(2.5, 1.0, 0.8, 1000))


tire_radius = 0.4
tire_width = 0.2
for i in range(4):
    tire = chronovehicle.ChTire(chrono.ChMaterialSurfaceNSC())
    tire.SetRadius(tire_radius)
    tire.SetWidth(tire_width)
    tire.SetFriction(2.0)
    vehicle.AddTire(tire)


terrain = chrono.ChTerrainSCM(system)
terrain.SetSize(20, 20)
terrain.SetBottomLevel(0)
terrain.SetTexture("textures/terrain.png")


terrain.SetYoungModulus(1e6)
terrain.SetPoissonRatio(0.3)
terrain.SetFriction(0.5)


terrain.SetMovingPatch(True)
terrain.SetPatchSize(5, 5)
terrain.SetPatchCenter(vehicle.GetChassis().GetPos())




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("HMMWV Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataPath() + 'logo_pychrono.png')


vehicle.SetChassisVisualizationType(chrono.VisualizationType_MESH)
for tire in vehicle.GetTires():
    tire.SetVisualizationType(chrono.VisualizationType_MESH)


terrain.SetSinkageVisualization(True)





while vis.Run():
    
    system.DoStepDynamics(0.01)

    
    terrain.SetPatchCenter(vehicle.GetChassis().GetPos())

    
    vis.Render()