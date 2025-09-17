import chrono as ch
import math


ch.InitChronic()


world = ch.ChWorld()


ground = ch.ChBodyEasyBox(
    name='ground',
    mass=0,
    pos=(0, 0, 0),
    rot=(0, 0, 0),
    size=(10, 10, 0.1),
    color=(0.5, 0.5, 0.5),
    elasticity=0.9,
    friction=0.9
)
world.Add(ground)


rover = ch.ChBodyEasyCylinder(
    name='rover',
    mass=100,
    pos=(0, 0, 0.1),
    rot=(0, 0, 0),
    size=(0.5, 0.5, 0.5),
    color=(1, 0, 0),
    elasticity=0.9,
    friction=0.9
)
world.Add(rover)


motor = ch.ChMotor()
rover.AddMotor(motor)


motor.SetAngle(0)


camera = ch.ChIrrSceneNode()
camera.SetPosition(0, 0, 5)
camera.SetRotation(0, 0, 0)
camera.SetScale(1, 1, 1)
world.Add(camera)


driver = ch.ChIrrAppSimpleGUI(world, camera)
driver.SetTTarget(1 / 60.0)
driver.SetCurrentTime(0)
driver.EnableDevice(ch.ChDeviceIdMouse)
driver.EnableLighting(True)
driver.EnableShadows(True)
driver.AddLogo("chrono logo.png")
driver.AddTexture("terrain texture.png")


while True:
    driver.BeginScene()
    driver.DrawAll()
    driver.EndScene()
    driver.NextFrame()