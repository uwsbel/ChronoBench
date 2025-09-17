import pychrono as chrono
from pychrono.irrlicht import *


physics = chrono.ChPhysicsEngine()
physics.SetIntegrationType(chrono.ChIntegrationType.SYSTEM)
physics.Set_G global_Solver_parallel(true)
physics.Set_G global_Solver_iterations(50)
physics.Set_G global_Solver_tolerance(1e-3)
physics.Set_G global_Solver_useLocks(true)
physics.Set_G global_Solver_enableWarmStart(true)
physics.Set_G global_Solver_enableFastIterations(true)
physics.Set_G global_Solver_enableJacobianFactorization(true)
physics.Set_G global_Solver_enablePiroStep(true)


terrain = chrono.ChTerrain()
terrain.SetMaterial(chrono.ChMaterialSurfaceNSC())
terrain.GetMaterial().SetFriction(0.6)
terrain.GetMaterial().SetRestitution(0.3)
terrain.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
terrain.AddBox(50, 0.5, 50, chrono.ChVectorD(0, -0.5, 0))
physics.Add(terrain)


vehicle = chrono.ChBodyEasyBox(2.5, 2, 4.5, 1000, True, True)
vehicle.SetPos(chrono.ChVectorD(0, 1, 0))
vehicle.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
vehicle.SetBodyFixed(False)
physics.Add(vehicle)


driver = chrono.ChDriver()
driver.Initialize(vehicle)
driver.SetSteering(0)
driver.SetThrottle(0)
driver.SetBrake(0)
physics.Add(driver)


app = IrrlichtApplication(physics, chrono.ChVectorD(0, 1.5, -5), chrono.ChVectorD(0, 0.5, 0))
app.SetLightType(irrlicht.LT_DIRECTIONAL)
app.SetAmbientLight(irrlicht.SColorf(0.3, 0.3, 0.3, 1))
app.SetSunDirection(chrono.ChVectorD(0, -1, 0))
app.SetSunColor(irrlicht.SColorf(1, 1, 1, 1))


while app.GetDevice().run():
    app.BeginScene(True, True, irrlicht.SColor(0, 100, 101, 140))
    app.DrawAll()
    app.EndScene()

    physics.DoStepDynamics(0.01)

    
    driver.SetSteering(sin(chrono.GetChronoTime() * 2))
    driver.SetThrottle(0.5 + 0.5 * sin(chrono.GetChronoTime() * 2))
    driver.SetBrake(0)

    app.PumpEvents()
    if app.GetDevice().isWindowActive():
        app.GetDevice().sleep(100)