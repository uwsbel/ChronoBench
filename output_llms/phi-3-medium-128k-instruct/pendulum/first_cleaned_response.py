import pychrono

import pychrono.Irrlicht

import pychrono.utils as utils




chrono = pychrono.ChSystemNSC()




ground = pychrono.ChBodyAO(pychrono.Vec3(0, 0, 0), pychrono.ChVectorD(0, 0, 0), pychrono.ChBodyTypes.STATIC_SPHERE)

ground.SetMass(1e6)  

ground.SetInertia(pychrono.ChVectorD(0, 0, 0))

chrono.AddBody(ground)




pendulum = pychrono.ChBodyAO(pychrono.ChVectorD(0, 0, 0), pychrono.ChVectorD(0, 0, 0), pychrono.ChBodyTypes.DYNAMIC_SPHERE)

pendulum.SetMass(1)  

pendulum.SetInertia(pychrono.ChVectorD(0, 0, 0))

pendulum.SetPosition(pychrono.ChVectorD(0, 0, 1))  

pendulum.SetVelocity(pychrono.ChVectorD(0, 0, 0))  

chrono.AddBody(pendulum)




joint = pychrono.ChRevoluteJoint(pendulum, ground, pychrono.ChVectorD(0, 0, 1), pychrono.ChVectorD(0, 0, 0))

joint.SetMaxForce(1e5)  

chrono.AddJoint(joint)




irrlicht = pychrono.IrrlichtVisualizer(chrono)




log = pychrono.ChLog(pychrono.ChLogLevel.LOG_ALL)

log.AddSystem(chrono)




while chrono.GetChTime() < 10:  

    chrono.DoStepDynamics(0.01)  

    irrlicht.Draw()  

    log.Print(chrono.GetChTime())  

    

    position = pendulum.GetPos()

    velocity = pendulum.GetLinearVel()

    print(f"Time: {chrono.GetChTime():.2f}, Position: {position}, Velocity: {velocity}")




irrlicht.End()

chrono.Destroy()