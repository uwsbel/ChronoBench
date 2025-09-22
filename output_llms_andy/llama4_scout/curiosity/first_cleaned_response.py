import chrono as ch
import chrono.irrlicht as chirr


ch.ChEngine.Initialize()


app = chirr.ChIrrApp()


ground = ch.ChBody()
ground.SetBodyFixed(True)


ground_shape = ch.ChCollisionShapeBox(ground, ch.ChVector3d(100, 1, 100))
ground_shape.SetFriction(0.8)
ground_shape.SetRestitution(0.3)


rover = ch.ChBody()
rover.SetPos(ch.ChVector3d(0, 1, 0))


steering_driver = ch.ChSteeringDriver(rover)
steering_driver.SetMaxSteeringAngle(ch.CH_C_PI / 4)


scene = app.GetSceneNode()


camera = scene.GetActiveCamera()
camera.SetPosition(ch.ChVector3d(0, 10, -20))
camera.LookAt(ch.ChVector3d(0, 0, 0))


scene.AddLight(chirr.ChIrrLight(scene, ch.ChVector3d(-10, 10, -10), ch.ChColor(1, 1, 1), 100))


rover_shape = ch.ChVisualShapeBox(rover, ch.ChVector3d(1, 0.5, 1), ch.ChVector3d(0, 0, 0), ch.ChQuaternion(1, 0, 0, 0), ch.ChColor(1, 0, 0))
rover_shape.SetTexture(ch.GetChronoDataFile("textures/blue.png"))


system = ch.ChSystemNSC()
system.Add(ground)
system.Add(rover)
system.Add(steering_driver)


system.SetGravity(ch.ChVector3d(0, -9.81, 0))
system.SetSolverType(ch.ChSolverType.PERFECTLY_PLASTIC)
system.SetTimestep(0.01)


while app.Run():
    steering_input = 0.1 * ch.Sin(app.GetTime())
    steering_driver.SetSteeringInput(steering_input)
    system.DoStepDynamics(0.01)
    app.Render()
    app.UpdateCamera()