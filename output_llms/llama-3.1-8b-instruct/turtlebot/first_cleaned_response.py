import chrono as c
import irrlicht
import time


c.init_data()
c.init_system()
c.init_irrlicht()


ground = c.ChBody()
ground.SetBodyType(c.CH_BODY_TYPE_BOX)
ground.SetPosition(c.ve(0, 0, 0))
ground.SetMass(0)
ground.SetFriction(1)
ground.SetRestitution(0)
c.AddGround(ground)


robot = c.ChBody()
robot.SetBodyType(c.CH_BODY_TYPE_BOX)
robot.SetPosition(c.ve(0, 1, 0))
robot.SetOrientation(c.ve(0, 0, 0))
robot.SetMass(1)
robot.SetFriction(1)
robot.SetRestitution(0)
c.AddBody(robot)


motor1 = c.ChMotor()
motor1.SetMaxForce(10)
motor1.SetPosition(0)
robot.AddMotor(motor1)

motor2 = c.ChMotor()
motor2.SetMaxForce(10)
motor2.SetPosition(0)
robot.AddMotor(motor2)


robot.SetName("Turtlebot")
ground.SetName("Ground")


device = c.CreateIrrlichtDevice(irrlicht.VideoMode(800, 600), irrlicht.E_DRIVER_TYPE_OPENGL3)
view = device.GetViewport(0)
view.SetTargetFrameRate(60)
view.SetAspectRatioConstraint(irrlicht.SCA_FIT)


camera = device.GetCameraSceneNode()
camera.SetPosition(c.ve(0, 10, 10))
camera.SetTarget(c.ve(0, 0, 0))
light = device.GetLightSceneNode()
light.SetPosition(c.ve(0, 10, 10))
light.SetDiffuseColor(c.ve(1, 1, 1))
light.SetSpecularColor(c.ve(1, 1, 1))
light.SetIntensity(1)


t = 0
while t < 10:
    
    if t >= 1 and t < 3:
        motor1.SetTorque(10)
        motor2.SetTorque(-10)
    else:
        motor1.SetTorque(0)
        motor2.SetTorque(0)

    
    c.Update()
    c.Render()

    
    t += 0.1
    time.sleep(0.1)


c.DeleteDevice(device)
c.shutdown()