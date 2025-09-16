import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math







mysystem = chrono.ChSystemNSC()
mysystem.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))







terrain = chrono.ChBodyEasyBox(100, 1, 100, 1000, True, True)
terrain.SetBodyFixed(True)
terrain.SetPos(chrono.ChVectorD(0, -1, 0))
terrain.SetMaterialSurface(chrono.ChMaterialSurfaceNSC(0.8, 0.2))
mysystem.Add(terrain)







kraz = chrono.ChVehicle("Kraz")


kraz_chassis = chrono.ChBodyEasyBox(3, 1, 5, 1000)
kraz_chassis.SetPos(chrono.ChVectorD(0, 1, 0))
kraz.SetChassisBody(kraz_chassis)
mysystem.Add(kraz_chassis)


wheel_radius = 0.5
wheel_width = 0.2
for i in range(4):
    wheel = chrono.ChWheel4(chrono.ChCoordsysD(
        chrono.ChVectorD(
            (i % 2) * 2.5 - 1.5, -wheel_radius, (i // 2) * 2.5 - 1.5
        )
    ))
    wheel.SetRadius(wheel_radius)
    wheel.SetWidth(wheel_width)
    wheel.SetFriction(0.8)
    wheel.SetMaterial(chrono.ChMaterialSurfaceNSC(0.8, 0.2))
    kraz.AddWheel(wheel)

mysystem.Add(kraz)







driver = chrono.ChDriver(kraz)
driver.SetSteeringGain(1.0)
driver.SetThrottleGain(1.0)
driver.SetBrakingGain(1.0)







vis = chronoirr.ChIrrApp(mysystem, "Kraz Simulation")
vis.SetWindowSize(1200, 800)
vis.AddTypicalSky()
vis.AddTypicalLights()
vis.SetCameraPosition(chrono.ChVectorD(5, 5, 5))
vis.SetCameraLookAt(chrono.ChVectorD(0, 1, 0))







while vis.GetDevice().run():
    vis.BeginScene()
    vis.Render()
    mysystem.DoStepDynamics(0.01)

    
    driver.SetSteering(math.sin(vis.GetChTime()))
    driver.SetThrottle(0.5)

    vis.EndScene()