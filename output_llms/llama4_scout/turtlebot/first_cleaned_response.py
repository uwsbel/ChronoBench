import chrono as ch
import math


ch.ChronoEnvironment.set_GEAR()


system = ch.ChSystemNSC()


system.set_G_acc(0, -9.81, 0)


ground = ch.ChBodyEasyBodyBox(ch.ChFrame(ch.Vector(0, -1, 0), ch.Quaternion()),
                               10, 0.1, 10,  // x, y, z dimensions
                               1000,         // density
                               True)         // fixed
ground.SetPos(ch.Vector(0, -1, 0))
system.AddBody(ground)


Turtlebot = ch.ChBodyEasyBodyAssimp("models/Turtlebot.obj",
                                     1,  // density
                                     False)  // not fixed
Turtlebot.SetPos(ch.Vector(0, 0.5, 0))
Turtlebot.SetRot(ch.Quaternion(1, 0, 0, 0))  // Reset rotation
system.AddBody(Turtlebot)


irrApp = ch.IrrlichtApplication(system)
irrApp.SetTimestep(0.02)
irrApp.SetRenderMode(ch.IrrlichtApplication.RM_TIRE)


irrApp.SetCameraPos(ch.Vector(0, 0, -5))
irrApp.SetCameraTarget(ch.Vector(0, 0, 0))


irrApp.AddLight(ch.IrrlichtLight(0, 0, 10, ch.Vector(0, 0, 10), ch.Vector(1, 1, 1)))


def Simulate():
    time = 0
    turn_time_left = 5
    turn_time_right = 10
    turn_duration = 2

    left_motor = ch.ChMotorSimpleSpeed()
    right_motor = ch.ChMotorSimpleSpeed()

    left_motor.Initialize(Turtlebot.GetBodyPartDynamics(0), ground.GetBody(), ch.ChFrame(ch.Vector(-0.2, 0.4, 0), ch.Quaternion()))
    right_motor.Initialize(Turtlebot.GetBodyPartDynamics(0), ground.GetBody(), ch.ChFrame(ch.Vector(0.2, 0.4, 0), ch.Quaternion()))

    system.AddMotor(left_motor)
    system.AddMotor(right_motor)

    while irrApp.Run():
        if time > turn_time_left and time <= turn_time_left + turn_duration:
            left_motor.SetSpeed(-0.5)
            right_motor.SetSpeed(0.5)
        elif time > turn_time_right and time <= turn_time_right + turn_duration:
            left_motor.SetSpeed(0.5)
            right_motor.SetSpeed(-0.5)
        else:
            left_motor.SetSpeed(0)
            right_motor.SetSpeed(0)

        system.Update()
        irrApp.Synchronize("", 0)
        irrApp.Render()

        time += irrApp.GetTimestep()


Simulate()