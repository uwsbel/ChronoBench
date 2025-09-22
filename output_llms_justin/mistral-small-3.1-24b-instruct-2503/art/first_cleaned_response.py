import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.postprocess as postprocess
import os


chrono.SetChronoDataPath(os.path.dirname(__file__))


system = chrono.ChSystemNSC()


terrain = chrono.ChRigidTerrain()
terrain.SetTexture(chrono.ChTexture("texture/terrain.jpg"), True)
terrain.SetPlane(chrono.ChCoordsys(chrono.ChVector(0, 0, 0), chrono.ChVector(0, 0, 1)))
terrain.SetContactFrictionCoefficient(0.9)
terrain.SetContactRestitutionCoefficient(0.2)
terrain.SetContactMaterialBehavior(chrono.ChMaterialSurface.NSC)
terrain.SetMaterialSurface(chrono.ChMaterialSurface.NSC)
system.Add(terrain)


vehicle = chrono.ChVehicleARTcar()
vehicle.SetChassisBody(chrono.ChBodyEasyBox(1.5, 1, 0.5, 1000, True, True))
vehicle.SetInitPosition(chrono.ChVector(0, 0, 1))
vehicle.SetInitOrientation(chrono.Q_from_AngAxis(0, chrono.ChVector(0, 1, 0)))
vehicle.SetContactMethod(chrono.ChMaterialSurface.NSC)
vehicle.SetVisualizationType(chrono.ChVehicleVisualizationType.SHADOW_VIS)
system.Add(vehicle.GetChassisBody())


vehicle.AddWheel(chrono.ChVector(-0.5, 0.5, 0.25), chrono.ChVector(0.5, 0.5, 0.25), 0.3, 0.1)
vehicle.AddWheel(chrono.ChVector(-0.5, -0.5, 0.25), chrono.ChVector(0.5, -0.5, 0.25), 0.3, 0.1)
vehicle.AddWheel(chrono.ChVector(0.5, 0.5, 0.25), chrono.ChVector(-0.5, 0.5, 0.25), 0.3, 0.1)
vehicle.AddWheel(chrono.ChVector(0.5, -0.5, 0.25), chrono.ChVector(-0.5, -0.5, 0.25), 0.3, 0.1)


driver = chrono.ChDriverARTcar()
driver.SetVehicle(vehicle)
driver.Initialize()


visualization = chronoirr.ChIrrApp()
visualization.AddTypicalSky()
visualization.AddTypicalLogo()
visualization.AddTypicalCamera(chrono.ChVector(0, -5, 3))
visualization.AddTypicalLights()
visualization.AssetBindAll()
visualization.AssetUpdateAll()
visualization.AddVisualSystem(system)


step_size = 1.0 / 50.0  
visualization.SetTimestep(step_size)


while visualization.Run():
    visualization.BeginScene()
    visualization.Render()
    visualization.EndScene()

    
    driver.SynchronizeForces()
    system.DoStepDynamics(step_size)


visualization.Close()