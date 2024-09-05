import pychrono




chrono = pychrono.ChSystemNSC()






ground = pychrono.Plane3D(pychrono.ChVector3D(0, 0, 0), pychrono.ChVector3D(0, 0, 1))

ground.setBody(chrono.groundBody())




ball = pychrono.ChBodySphere(1)

ball.setBody(chrono.body())

ball.setPos(pychrono.ChVector3D(0, 0, 5))






ball.setMass(1)




gravity = pychrono.ChVector3D(0, 0, -9.81)

ball.addForce(gravity)




ground.addRigidBody(ball)




while (1):

    

    chrono.DoStepDynamics(0.01)


    

    print("Ball position:", ball.getPos())


    

    if ball.getPos().z <= 0:

        print("Ball has hit the ground.")

        break