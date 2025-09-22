import pychrono.core as chrono
import pychrono.irrlicht as chronoirr






system = chrono.ChSystemNSC()
system.Set_G_gravity(chrono.ChVectorD(0, -9.81, 0))


system.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED)
system.SetSolverType(chrono.ChSolver.Type_PSOR)
system.SetSolverMaxIterations(50)
system.SetSolverMaxIterationsBilateral(100)






truss_material = chrono.ChMaterialSurfaceNSC()
truss_material.SetFriction(0.5)
truss_material.SetRestitution(0.2)

truss = chrono.ChBodyEasyBox(2, 0.1, 0.1, 1000, True, True, truss_material)
truss.SetBodyFixed(True)
system.Add(truss)


bar_material = chrono.ChMaterialSurfaceNSC()
bar_material.SetFriction(0.3)

bar_radius = 0.05
bar_length = 1.5
bar = chrono.ChBodyEasyCylinder(bar_radius, bar_length, 1000, True, True, bar_material)
bar.SetPos(chrono.ChVectorD(0, 0.5, 0))
system.Add(bar)


revolute_joint = chrono.ChLinkRevolute()
revolute_joint.Initialize(truss, bar, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
system.AddLink(revolute_joint)


sun_gear_radius = 0.2
sun_gear = chrono.ChBodyEasyCylinder(sun_gear_radius, 0.1, 1000, True, True, bar_material)
sun_gear.SetPos(chrono.ChVectorD(0, 0.5, 0.2))
system.Add(sun_gear)


planet_gear_radius = 0.1
planet_gear = chrono.ChBodyEasyCylinder(planet_gear_radius, 0.1, 1000, True, True, bar_material)
planet_gear.SetPos(chrono.ChVectorD(sun_gear_radius + planet_gear_radius, 0.5, 0.2))
system.Add(planet_gear)


revolute_joint_sun = chrono.ChLinkRevolute()
revolute_joint_sun.Initialize(
    bar, sun_gear, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.2), chrono.ChQuaternionD(1, 0, 0, 0))
)
system.AddLink(revolute_joint_sun)


gear_joint = chrono.ChLinkGears()
gear_joint.Initialize(
    sun_gear, planet_gear, chrono.ChVectorD(0, 0, 1), sun_gear_radius, planet_gear_radius
)
system.AddLink(gear_joint)




motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(
    sun_gear,  
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),  
)
motor.SetMotorFunction(chrono.ChFunction_Const(3.14))  
system.AddLink(motor)




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("Epicyclic Gears")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(2, 1, 2), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()




while vis.Run():
    dt = 0.01
    system.DoStepDynamics(dt)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()