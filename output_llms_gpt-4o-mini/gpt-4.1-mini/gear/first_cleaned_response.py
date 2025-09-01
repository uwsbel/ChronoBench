import pychrono as chrono
import pychrono.irrlicht as chronoirr




system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  





truss = chrono.ChBodyEasyBox(0.5, 0.05, 0.5, 1000, True, True)
truss.SetBodyFixed(True)
truss.SetPos(chrono.ChVectorD(0, 0, 0))
truss.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.2, 0.7))
system.AddBody(truss)




bar_length = 1.2
bar_thickness = 0.05
bar = chrono.ChBodyEasyBox(bar_length, bar_thickness, bar_thickness, 1000, True, True)
bar.SetPos(chrono.ChVectorD(0, 0.1, 0))
bar.GetVisualShape(0).SetColor(chrono.ChColor(0.7, 0.2, 0.2))
system.AddBody(bar)


bar_truss_joint = chrono.ChLinkLockRevolute()
bar_truss_joint.Initialize(bar, truss, chrono.ChCoordsysD(bar.GetPos(), chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.VECT_X)))
system.AddLink(bar_truss_joint)













gear_thickness = 0.05
gear_radius_sun = 0.2
gear_radius_planet = 0.1


sun_gear = chrono.ChBodyEasyCylinder(gear_thickness, gear_radius_sun, 1000, True, True)
sun_gear.SetBodyFixed(True)
sun_gear.SetPos(chrono.ChVectorD(0, 0.1, 0))
sun_gear.GetVisualShape(0).SetColor(chrono.ChColor(0.8, 0.8, 0.1))
system.AddBody(sun_gear)


planet_gear = chrono.ChBodyEasyCylinder(gear_thickness, gear_radius_planet, 1000, True, True)

planet_pos = chrono.ChVectorD(bar_length / 2 - gear_radius_planet, 0.1, 0)
planet_gear.SetPos(planet_pos)
planet_gear.GetVisualShape(0).SetColor(chrono.ChColor(0.1, 0.8, 0.1))
system.AddBody(planet_gear)


planet_bar_joint = chrono.ChLinkLockRevolute()
planet_bar_joint.Initialize(planet_gear, bar, chrono.ChCoordsysD(planet_pos, chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.VECT_X)))
system.AddLink(planet_bar_joint)












gear_ratio = gear_radius_sun / gear_radius_planet







gear_sun_planet = chrono.ChLinkGear()
gear_sun_planet.Initialize(sun_gear, planet_gear, chrono.ChFrameD(), chrono.ChFrameD())
gear_sun_planet.SetRatio(-gear_ratio)  
system.AddLink(gear_sun_planet)




motor = chrono.ChLinkMotorRotationAngle()
motor.Initialize(bar, truss, chrono.ChFrameD(bar.GetPos(), chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.VECT_X)))
system.AddLink(motor)


omega = chrono.CH_C_PI  


motor.SetAngleFunction(chrono.ChFunction_Ramp(0, omega))




application = chronoirr.ChIrrApp(system, "Epicyclic Gear System Simulation with PyChrono", chronoirr.dimension2du(1024, 768))

application.AddTypicalLights()
application.AddLogo()
application.SetChaseCamera(target=bar, dist=2.5, height=0.7)
application.AssetBindAll()
application.AssetUpdateAll()




application.SetTimestep(0.005)

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()

    
    application.GetIGUIEnvironment().addStaticText(
        "Epicyclic Gear Simulation: Bar driven with constant angular velocity, " 
        "planet gear meshes with fixed sun gear",
        chrono.irrlicht.irrlicht.EGUI_ALIGNMENT.ALIGN_CENTER,  
        chrono.irrlicht.irrlicht.dimension2di(100, 20),  
        chrono.irrlicht.irrlicht.dimension2di(1024, 40),
        None, 1)

    system.DoStepDynamics(application.GetTimestep())

    application.EndScene()