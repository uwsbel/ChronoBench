import pychrono as chrono
import pychrono.irrlicht as vis


chrono.SetChronoDataPath("path/to/chrono/data")  
system = chrono.ChSystemNSC()




vehicle = chrono.ChBodyEasyBox(1.5, 2.0, 1.8, 1000)  
vehicle.SetPos(chrono.ChVectorD(0, 0, 0.5))  
vehicle.SetBodyFixed(True)  
system.Add(vehicle)


terrain = chrono.ChBodyEasyBox(10, 10, 0.1, 10000)  
terrain.SetPos(chrono.ChVectorD(0, 0, 0))  
terrain.SetBodyFixed(True)  
terrain.SetMaterialSurface(chrono.ChMaterialSurface(0.8, 0.2))  
system.Add(terrain)








vis_app = vis.ChIrrApp(system, 'UAZBUS Simulation')
vis_app.AddCamera(
    vis.IrrlichtCamera(
        pos=chrono.ChVectorD(5, 5, 2),
        target=chrono.ChVectorD(0, 0, 0),
        up=chrono.ChVectorD(0, 1, 0)
    )
)
vis_app.SetCamera(vis_app.GetActiveCamera())
vis_app.SetBackgroundSkyColor(chrono.ChColor(0.8, 0.8, 1))  
vis_app.SetAmbientLight(chrono.ChColor(0.3, 0.3, 0.3))  
vis_app.SetSunLight(chrono.ChColor(1, 1, 1), chrono.ChVectorD(1, 1, -1))  


while vis_app.Run():
    system.DoStepDynamics(1/60.0)  
    vis_app.Render()  
    vis_app.Sync()