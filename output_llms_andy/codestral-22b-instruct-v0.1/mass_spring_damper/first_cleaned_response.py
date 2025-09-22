import pychrono as chrono
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()


application = irr.ChIrrApp(system, 'Mass-Spring-Damper System', chrono.ChVector(0, 0, -2), chrono.ChVector(0, 0, 0), chrono.ChVector(0, 1, 0), 1.5)
application.AddTypicalLights()
application.AddTypicalCamera(chrono.ChVector(0, 0, -3))
application.AddTypicalLogo()
application.AddSkyBox()
application.AddLightWithShadow(chrono.ChVector(1.5, 5.5, -2.5), chrono.ChVector(0, 0, 0), 3, 2.2, 7.2)


ground = chrono.ChBodyEasyBox(10, 1, 10, 1000, chrono.ChMaterialSpec_unfribed())
system.Add(ground)


mass = chrono.ChBodyEasySphere(0.5, 10, chrono.ChMaterialSpec_unfribed())
mass.SetPos(chrono.ChVector(0, 2, 0))
system.Add(mass)


mass_shape = chrono.ChSphereShape()
mass_shape.GetSphereGeometry().rad = 0.5
mass_shape_asset = chrono.ChVisualShape()
mass_shape_asset.AddAsset(mass_shape)
mass.AddAsset(mass_shape_asset)


spring_damper = chrono.ChLinkTSDA()
spring_damper.Initialize(mass, ground, chrono.ChCoordsys<>(chrono.ChVector(0, 2, 0)))
system.Add(spring_damper)


spring_shape = chrono.ChSpringShape()
spring_shape.p1 = chrono.ChVector(0, 1, 0)
spring_shape.p2 = chrono.ChVector(0, 2, 0)
spring_shape.spring_coils = 10
spring_shape.spring_length = 1
spring_shape.spring_radius = 0.05
spring_shape.spring_thickness = 0.01
spring_shape_asset = chrono.ChVisualShape()
spring_shape_asset.AddAsset(spring_shape)
spring_shape_asset.SetColor(chrono.ChColor(0.8, 0.8, 0))
spring_damper.AddAsset(spring_shape_asset)


system.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
system.SetSolverMaxIterations(100)
system.SetTimestep(0.005)


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()