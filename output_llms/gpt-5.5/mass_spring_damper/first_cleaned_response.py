import pychrono as chrono
import pychrono.irrlicht as irr






system = chrono.ChSystemNSC()


system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))


system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
system.GetSolver().AsIterative().SetMaxIterations(50)






ground = chrono.ChBody()
ground.SetName("ground")
ground.SetFixed(True)
ground.SetPos(chrono.ChVector3d(0, 0, 0))


anchor_shape = chrono.ChVisualShapeSphere(0.06)
anchor_mat = chrono.ChVisualMaterial()
anchor_mat.SetDiffuseColor(chrono.ChColor(0.1, 0.1, 0.9))
anchor_shape.AddMaterial(anchor_mat)
ground.AddVisualShape(anchor_shape)

system.Add(ground)






mass = chrono.ChBody()
mass.SetName("moving_mass")
mass.SetMass(1.0)
mass.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))


mass.SetPos(chrono.ChVector3d(1.5, 0, 0))


mass.SetPosDt(chrono.ChVector3d(0, 0, 0))


mass_shape = chrono.ChVisualShapeBox(0.25, 0.25, 0.25)
mass_mat = chrono.ChVisualMaterial()
mass_mat.SetDiffuseColor(chrono.ChColor(0.8, 0.2, 0.2))
mass_shape.AddMaterial(mass_mat)
mass.AddVisualShape(mass_shape)

system.Add(mass)






spring = chrono.ChLinkTSDA()
spring.SetName("linear_spring_damper")



ground_anchor = chrono.ChVector3d(0, 0, 0)
mass_anchor = chrono.ChVector3d(1.5, 0, 0)

spring.Initialize(
    ground,
    mass,
    False,
    ground_anchor,
    mass_anchor
)


spring.SetRestLength(1.0)
spring.SetSpringCoefficient(50.0)   
spring.SetDampingCoefficient(2.0)   


spring_visual = chrono.ChVisualShapeSpring(0.05, 80, 15)
spring.AddVisualShape(spring_visual)

system.AddLink(spring)






vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)

vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("PyChrono: Single Mass-Spring-Damper with ChLinkTSDA")
vis.Initialize()

vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(
    chrono.ChVector3d(0.75, 1.0, 3.0),
    chrono.ChVector3d(0.75, 0.0, 0.0)
)
vis.AddTypicalLights()






time_step = 1e-3

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    system.DoStepDynamics(time_step)