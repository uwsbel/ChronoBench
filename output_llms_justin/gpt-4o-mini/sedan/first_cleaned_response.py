import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath('/path/to/chrono/data/')  
system = chrono.ChSystemNSC()  



ground = chrono.ChBody()
ground.SetBodyFixed(True)  
ground.SetPos(chrono.ChVectorD(0, 0, 0))  
ground.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())  
system.Add(ground)


sphere_radius = 0.5
sphere = chrono.ChBody()
sphere.SetPos(chrono.ChVectorD(0, sphere_radius, 0))  
sphere.SetMass(1.0)  
sphere.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))  
sphere.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())  
system.Add(sphere)



gravity = chrono.ChVectorD(0, -9.81, 0)  
system.Set_G_acc(gravity)  


initial_velocity = chrono.ChVectorD(0, 0, 0)  
sphere.SetPos_dt(initial_velocity)  


time_step = 0.01  
end_time = 5.0  

while system.GetChTime() < end_time:
    system.DoStepDynamics(time_step)  
    print(f"Time: {system.GetChTime():.2f}, Sphere Position: {sphere.GetPos()}")  


app = chronoirr.ChIrrApp(system, "PyChrono Simulation", chronoirr.dimension2d(800, 600))
app.AddTypicalSky()
app.AddTypicalCamera(chronoirr.vector3df(0, 5, -10))
app.AddTypicalLights()
app.AssetBindAll()
app.AssetUpdateAll()


while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()
    app.EndScene()
    system.DoStepDynamics(time_step)