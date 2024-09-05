import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


chrono.ChSystemSMC()




terrain = chrono.ChBodyEasyBox(200, 1, 200, 1000, True, True)
terrain.SetPos(chrono.ChVectorD(0, -1, 0))
terrain.SetBodyFixed(True)
terrain.GetMaterialSurface().SetFriction(0.9)
terrain.GetMaterialSurface().SetRestitution(0.0)
chrono.GetSystem().Add(terrain)


vehicle = veh.ChVehicleM113()
vehicle.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.Q_from_AngAxis(chrono.CH_C_PI, chrono.VECT_X)))
chrono.GetSystem().Add(vehicle.GetChassisBody())


driver = veh.ChIrrGuiDriver(vehicle, chronoirr.GetDevice())


vehicle.GetChassisBody().SetPos(chrono.ChVectorD(0, 0.5, 0))
vehicle.GetChassisBody().SetWvel_par(chrono.ChVectorD(0, 0, 0))


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(chrono.GetSystem())
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("M113 Vehicle Simulation")
vis.AddCamera(chrono.ChVectorD(0, 2, -5), chrono.ChVectorD(0, 0.5, 0))
vis.AddLight(chrono.ChVectorD(10, 10, 10), chrono.ChColor(1.0, 1.0, 1.0))
vis.Initialize()


step_size = 0.01
sim_time = 0

while vis.Run():
    vis.BeginScene()
    vis.Render()

    
    driver.Synchronize(sim_time)
    vehicle.Synchronize(sim_time, chrono.GetSystem())
    chrono.GetSystem().DoStepDynamics(step_size)
    sim_time += step_size

    vis.EndScene()