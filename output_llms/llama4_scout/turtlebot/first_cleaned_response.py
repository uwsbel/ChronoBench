import chrono as ch
import chrono.irrlicht as chirr


ch.ChEngine.Initialize()


system = ch.ChSystemNSC()


system.SetGravitationalAcceleration(ch.ChVector3d(0, -9.81, 0))


ground = ch.ChBodyEasyTerrain(
    "ground",
    1000,  
    1000,  
    20,   
    ch.ChVector3d(0, -10, 0),  
    ch.ChQuaterniond(1, 0, 0, 0),  
    False  
)


system.AddBody(ground)


class Turtlebot:
    def __init__(self, name, pos, rot):
        self.name = name
        self.pos = pos
        self.rot = rot
        self.system = system

        
        self.chassis = ch.ChBodyEasyBox(
            name + "_chassis",
            0.5,  
            0.3,  
            0.2,  
            1000,  
            ch.ChVector3d(pos.x, pos.y, pos.z),  
            ch.ChQuaterniond(rot.x, rot.y, rot.z, rot.w)  
        )
        self.chassis.SetInertiaXX(ch.ChVector3d(1, 1, 1))
        self.system.AddBody(self.chassis)

        
        self.left_wheel = ch.ChBodyEasyCylinder(
            name + "_left_wheel",
            0.1,  
            0.05,  
            1000,  
            ch.ChVector3d(pos.x - 0.2, pos.y, pos.z - 0.1),  
            ch.ChQuaterniond(rot.x, rot.y, rot.z, rot.w)  
        )
        self.left_wheel.SetInertiaXX(ch.ChVector3d(1, 1, 1))
        self.system.AddBody(self.left_wheel)

        
        self.right_wheel = ch.ChBodyEasyCylinder(
            name + "_right_wheel",
            0.1,  
            0.05,  
            1000,  
            ch.ChVector3d(pos.x + 0.2, pos.y, pos.z - 0.1),  
            ch.ChQuaterniond(rot.x, rot.y, rot.z, rot.w)  
        )
        self.right_wheel.SetInertiaXX(ch.ChVector3d(1, 1, 1))
        self.system.AddBody(self.right_wheel)

        
        self.left_joint = ch.ChLinkLockRevolute()
        self.left_joint.Initialize(self.chassis, self.left_wheel, ch.ChFrame3d(ch.ChVector3d(0, 0, 0), ch.ChQuaterniond(1, 0, 0, 0)))
        self.system.AddLink(self.left_joint)

        self.right_joint = ch.ChLinkLockRevolute()
        self.right_joint.Initialize(self.chassis, self.right_wheel, ch.ChFrame3d(ch.ChVector3d(0, 0, 0), ch.ChQuaterniond(1, 0, 0, 0)))
        self.system.AddLink(self.right_joint)

        
        self.left_motor = ch.ChLinkMotorRotationSpeed()
        self.left_motor.Initialize(self.chassis, self.left_wheel, ch.ChFrame3d(ch.ChVector3d(0, 0, 0), ch.ChQuaterniond(1, 0, 0, 0)))
        self.system.AddLink(self.left_motor)

        self.right_motor = ch.ChLinkMotorRotationSpeed()
        self.right_motor.Initialize(self.chassis, self.right_wheel, ch.ChFrame3d(ch.ChVector3d(0, 0, 0), ch.ChQuaterniond(1, 0, 0, 0)))
        self.system.AddLink(self.right_motor)


turtlebot = Turtlebot("Turtlebot", ch.ChVector3d(0, 0, 0.5), ch.ChQuaterniond(1, 0, 0, 0))


app = chirr.IrrlichtApplication(system, ch.ChVector3d(0, 0, 0), ch.ChVector3d(0, -10, -10))


app.SetCameraPosition(ch.ChVector3d(0, -5, 5))
app.SetCameraTarget(ch.ChVector3d(0, 0, 0))


app.EnableSunLight()


time_step = 0.01
t_end = 10.0
t = 0

while t < t_end:
    
    if t < 2:
        turtlebot.left_motor.SetSpeed(ch.ChRadians(10))
        turtlebot.right_motor.SetSpeed(ch.ChRadians(10))
    elif t < 4:
        turtlebot.left_motor.SetSpeed(ch.ChRadians(-10))
        turtlebot.right_motor.SetSpeed(ch.ChRadians(10))
    else:
        turtlebot.left_motor.SetSpeed(ch.ChRadians(0))
        turtlebot.right_motor.SetSpeed(ch.ChRadians(0))

    
    system.Update(time_step)

    
    app.Render()

    
    t += time_step

    
    if app.GetInputReceiver().IsKeyDown(irr.K_ESCAPE):
        break


app.Exit()