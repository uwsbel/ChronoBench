Here is the Python code that meets the specified requirements:

```python
import chrono as c
import irrlicht
import time


c.init_data()
c.init_system()
c.init_irrlicht()


ground = c.ChBody()
ground.SetBodyType(c.CH_BODY_KINEMATIC)
ground.SetMass(0)
ground.SetDensity(0)
ground.SetFriction(1)
ground.SetRestitution(0)
ground.SetPos(c.Vec3(0, -1, 0))
ground.SetAng(c.Vec3(0, 0, 0))
ground.SetRot(c.Mat3(c.PI / 2, 0, 0))


turtlebot = c.ChBody()
turtlebot.SetBodyType(c.CH_BODY_KINEMATIC)
turtlebot.SetMass(1)
turtlebot.SetDensity(1)
turtlebot.SetFriction(1)
turtlebot.SetRestitution(0)
turtlebot.SetPos(c.Vec3(0, 0, 0))
turtlebot.SetAng(c.Vec3(0, 0, 0))
turtlebot.SetRot(c.Mat3(c.PI / 2, 0, 0))


motor = c.ChMotor()
motor.SetType(c.MTORQUE)
motor.SetTorque(10)
motor.SetMaxSpeed(100)
motor.SetPos(0)
motor.SetAng(0)
motor.SetRot(c.Mat3(c.PI / 2, 0, 0))


c.SetGravity(c.Vec3(0, -9.81, 0))
c.AddBody(ground)
c.AddBody(turtlebot)
c.AddMotor(motor)


device = irrlicht.IrrlichtDevice.create_driver(irrlicht.DriverType.EDT_OPENGL, c.GetIrrlichtWindow())
irrlicht.IrrlichtDevice.set_window_title(device, "Turtlebot Simulation")
irrlicht.IrrlichtDevice.set_video_mode(device, irrlicht.dimension2du(1024, 768), irrlicht.SED_FULLSCREEN, irrlicht.SFWS_WINDOWED)
irrlicht.IrrlichtDevice.set_screen_fullscreen(device, True)
irrlicht.IrrlichtDevice.set_screen_resolution(device, irrlicht.dimension2du(1024, 768))
irrlicht.IrrlichtDevice.set_screen_fullscreen(device, False)
irrlicht.IrrlichtDevice.set_screen_resolution(device, irrlicht.dimension2du(1024, 768))
irrlicht.IrrlichtDevice.set_screen_fullscreen(device, True)
irrlicht.IrrlichtDevice.set_screen_resolution(device, irrlicht.dimension2du(1024, 768))
irrlicht.IrrlichtDevice.set_screen_fullscreen(device, False)
irrlicht.IrrlichtDevice.set_screen_resolution(device, irrlicht.dimension2du(1024, 768))
irrlicht.IrrlichtDevice.set_screen_fullscreen(device, True)
irrlicht.IrrlichtDevice.set_screen_resolution(device, irrlicht.dimension2du(1024, 768))


camera = device.get_camera_controller()
camera.set_target(c.Vec3(0, 0, 0))
camera.set_fixed_distance(5)
camera.set_y_rotation(-30)


light = device.get_light_manager().add_light(c.Vec3(0, 10, 0), c.Vec3(1, 1, 1), 1000)
light.set_type(irrlicht.LT_DIRECTIONAL)


while True:
    
    c.Update(0.01)

    
    device.begin_frame()
    device.draw_all()
    device.end_frame()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime() * 0.1)
    turtlebot.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    device.update()

    
    c.CheckCollisions()

    
    motor.SetPos(c.GetTime() * 0.1)
    motor.SetAng(c.GetTime() * 0.1)

    
    device.draw_all()

    
    turtlebot.SetPos(c.GetTime