import pychrono as chrono
import pychrono.irrlicht as vis
import pychrono.postprocess as postprocess


chrono.SetChronoData(chrono.ChSystemNSC)
system = chrono.ChSystemNSC()


terrain = chrono.ChBodyEasyCylinder(10, 10, 0.1, 1000)
terrain.SetPos(chrono.ChVectorD(0, 0, 0))
terrain.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())
terrain.GetMaterialSurface().SetFriction(0.8)
terrain.GetMaterialSurface().SetRestitution(0.2)
system.AddBody(terrain)



vehicle = chrono.ChBodyEasyBox(1, 1, 1, 1000)
vehicle.SetPos(chrono.ChVectorD(0, 0, 0.1))
system.AddBody(vehicle)



driver = chrono.ChDriver()
driver.SetVehicle(vehicle)


vis_app = vis.ChIrrApp(system, 'UAZBUS Simulation', 'data/irrlicht.cfg')
vis_app.AddCamera(chrono.ChVectorD(5, 5, 5), chrono.ChVectorD(0, 0, 0))
vis_app.SetAmbientLight(chrono.ChColor(0.2, 0.2, 0.2))
vis_app.SetSunLight(chrono.ChColor(0.8, 0.8, 0.8), chrono.ChVectorD(1, 1, 1))


while vis_app.Run():
    
    driver.Update()

    
    system.DoStepDynamics(chrono.ChTime(1.0/60.0))

    
    vis_app.Render()



postprocess.SaveData(system, 'data/simulation_data.csv')