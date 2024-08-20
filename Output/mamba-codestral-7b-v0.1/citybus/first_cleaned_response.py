import pychrono


system = pychrono.ChSystemSMC()


terrain = pychrono.RigidTerrain(system)

terrain.SetTexture(pychrono.GetChronoDataFile("textures/concrete.png"))

bus = pychrono.ChVehicle(system)

bus.SetChassisFixed(False)
bus.SetInitPosition(pychrono.ChVectorD(0, 0, 0))
bus.SetInitForwardVelocity(pychrono.ChVectorD(10, 0, 0))


bus.SetTireType(pychrono.TireModelType.RIGID_TIRE)


camera = pychrono.ChCameraUniversal()
camera.SetPosition(pychrono.ChVectorD(0, 2, -5))
camera.SetTarget(pychrono.ChVectorD(0, 0, 0))


driver = pychrono.ChIrrDriver(system, bus, camera)


while system.GetChTime() < 10.0:
    system.DoStepDynamics(0.02)
    driver.Render()