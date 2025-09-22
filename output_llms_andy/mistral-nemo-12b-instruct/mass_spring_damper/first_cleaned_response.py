import pychrono as chrono
import pyirrlicht as ir


physics = chrono.ChPhysicsEngine()
physics.Set_Gravity(chrono.ChVectorD(0, -9.81, 0))
chrono.ChSystem().SetPhysicsEngine(physics)


system = chrono.ChSystemSMC()
system.Set_Gravity(chrono.ChVectorD(0, -9.81, 0))
system.Set_SolverType(chrono.ChSolverType_SOR)


ground = chrono.ChBodyEasyBox(10, 10, 10, 1000, False)
ground.SetPos(chrono.ChVectorD(0, -5, 0))
system.Add(ground)

mass = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1, True)
mass.SetPos(chrono.ChVectorD(0, 2, 0))
system.Add(mass)

spring_damper = chrono.ChLinkTSDA()
spring_damper.Initialize(mass, ground, chrono.ChVectorD(0, 1, 0), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, -1, 0), chrono.ChVectorD(0, 0, 0))
spring_damper.Set_Spring_Coef(chrono.ChVectorD(100, 0, 0))
spring_damper.Set_Damper_Coef(chrono.ChVectorD(2, 0, 0))
system.AddLink(spring_damper)


app = ir.CIrrlichtApplication(irrlicht.C IrrlichtDevice(irrlicht.C IrrlichtDevice_GL), 800, 600, 16, False, True, "PyChrono Irrlicht Example")
app.setGravity(chrono.ChVectorD(0, -9.81, 0))
app.AddLight(chrono.ChVectorD(0, 10, 0), chrono.ChColor(0.8, 0.8, 0.8))
app.SetCamera(chrono.ChVectorD(1, 3, -5), chrono.ChVectorD(0, 0, 0))


mass_shape = chrono.ChBoxShape(0.5, 0.5, 0.5)
mass_shape.SetMaterial(chrono.ChMaterialSurface().DefaultMaterial(chrono.ChColor(0.8, 0.2, 0.2)))
mass.AddAsset(mass_shape)

spring_shape = chrono.ChCylinderShape(0.1, 2)
spring_shape.SetMaterial(chrono.ChMaterialSurface().DefaultMaterial(chrono.ChColor(0.2, 0.8, 0.2)))
spring_damper.AddAsset(spring_shape)


while app.GetDevice().run():
    system.DoStepDynamics(0.01)
    app.BeginScene(True, True, chrono.ChColor(0.9, 0.9, 0.9))
    app.DrawAll()
    app.EndScene()