import pychrono as chrono
import pychrono.irrlicht as chronoirr







system = chrono.ChSystemNSC()


system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
system.SetSolverMaxIterations(150)
system.SetMaxItersSpeed(10)
system.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)


ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, -1, 0))
ground.SetName("Ground")
ground.SetCollide(False)
ground.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())


system.AddBody(ground)


mass = chrono.ChBody()
mass_radius = 0.1
mass_density = 1000  
mass_volume = (4/3) * chrono.CH_C_PI * (mass_radius**3)
mass_mass = mass_volume * mass_density
mass.SetMass(mass_mass)
mass.SetInertiaXX(chrono.ChVectorD(mass_mass * mass_radius**2 / 5))
mass.SetPos(chrono.ChVectorD(0, 1, 0))
mass.SetName("Mass")


system.AddBody(mass)


mass_asset = chrono.ChSphereShape(mass_radius)
mass_visual_shape = chrono.ChVisualShapeSphere(mass_radius)
mass.AddAsset(mass_asset)
mass.AddAsset(mass_visual_shape)


spring_damper = chrono.ChLinkTSDA()
spring_damper.Initialize(ground, mass,
                         True, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, -1, 0),
                         False, chrono.ChVectorD(), chrono.ChVectorD())
system.AddLink(spring_damper)


spring_damper.SetSpringCoefficient(1000)  
spring_damper.SetDampingCoefficient(10)   







application = chronoirr.ChIrrApp(system, "Mass-Spring-Damper System", chrono.core.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngAxis(chrono.CH_C_PI / 2, chrono.ChVectorD(0, 1, 0))))


application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chrono.ChVectorD(1, 1, 1), chrono.ChVectorD(0, 1, 0))


time_step = 1e-3
application.SetTimestep(time_step)
application.SetTryRealtime(True)


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()