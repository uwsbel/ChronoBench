import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


chrono.ChSystemSMC()




vehicle = veh.Kraz()
vehicle.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.Q_from_AngAxis(chrono.CH_C_PI, chrono.VECT_X)))


terrain = chrono.ChBodyEasyBox(100, 100, 0.1, 1000, True, True)
terrain.SetPos(chrono.ChVectorD(0, 0, -0.05))
terrain.SetBodyFixed(True)
terrain.GetMaterialSurface().SetFriction(0.9)
terrain.GetMaterialSurface().SetRestitution(0.0)


driver = veh.ChIrrGuiDriver(vehicle, chronoirr.GetDevice())


vehicle.GetVehicle().SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.GetVehicle().SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.GetVehicle().SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.GetVehicle().SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(vehicle.GetSystem())
vis.AddCamera(chrono.ChVectorD(0, -10, 1), chrono.ChVectorD(0, 0, 0))
vis.AddLight(chrono.ChVectorD(10, 10, 10), chrono.ChColor(1, 1, 1))
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('PyChrono Kraz Simulation')
vis.Initialize()


step_size = 0.01
sim_time = 0

while vis.Run():
    vis.BeginScene()
    vis.Render()

    
    vehicle.Synchronize(sim_time)
    driver.Synchronize(sim_time)
    vehicle.Advance(step_size)
    sim_time += step_size

    vis.EndScene()