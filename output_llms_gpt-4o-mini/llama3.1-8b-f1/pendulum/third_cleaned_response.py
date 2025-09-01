import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math








change_mass = False


initLoc = chrono.ChVector3d(0, 0, 1.2)





swingUpMethod = 1






pivotMethod = 1






collisionType = 2








print( "Copyright (c) 2017 projectchrono.org\n")


sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(collisionType)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))


ground = chrono.ChBody()
ground.SetFixed(True)
sys.Add(ground)


pend1 = chrono.ChBody()
pend1.SetMass(1)
pend1.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
sys.Add(pend1)


pend2 = chrono.ChBody()
pend2.SetMass(1)
pend2.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
sys.Add(pend2)


ball = chrono.ChVisualShapeSphere(0.1)
ball.SetTexture(chrono.GetChronoDataFile("textures/checker2.png"))
pend1.AddVisualShape(ball)
pend2.AddVisualShape(ball)


link1 = chrono.ChLinkLockRevolute()
link1.Initialize(pend1, pend2, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleZ(0)))
sys.AddLink(link1)

link2 = chrono.ChLinkLockRevolute()
if (pivotMethod == 1):
    link2.Initialize(pend2, ground, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleZ(0)))
elif (pivotMethod == 2):
    link2.Initialize(pend2, ground, chrono.ChFramed(chrono.ChVector3d(0, 0, 0.5, 0, 0, 0), chrono.QuatFromAngleZ(0)))
elif (pivotMethod == 3):
    link2.Initialize(pend2, ground, chrono.ChFramed(chrono.ChVector3d(0, 0, 0.67, 0, 0, 0), chrono.QuatFromAngleZ(0)))
sys.AddLink(link2)


pend1.SetPos(chrono.ChVector3d(initLoc.x, initLoc.y, initLoc.z))
pend2.SetPos(chrono.ChVector3d(0, 0, 0))


pend1.EnableCollision(True)
pend2.EnableCollision(True)


if (collisionType == 1):
    pend1.GetCollisionModel().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    pend2.GetCollisionModel().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
elif (collisionType == 2):
    pend1.GetCollisionModel().SetCollisionSystemType(chrono.ChCollisionSystem.Type_NSC)
    pend2.GetCollisionModel().SetCollisionSystemType(chrono.ChCollisionSystem.Type_NSC)
elif (collisionType == 3):
    pend1.GetCollisionModel().SetCollisionSystemType(chrono.ChCollisionSystem.Type_SMC)
    pend2.GetCollisionModel().SetCollisionSystemType(chrono.ChCollisionSystem.Type_SMC)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('Double Pendulum')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(2,2,3))
vis.AddTypicalLights()


driver = chronoirr.ChDriverIRR()
driver.SetChTime(0.0)
driver.SetChStepSize(1e-3)
driver.SetChMaxSteps(1000)
driver.SetWindowSize(1024,768)
driver.SetWindowTitle('Double Pendulum')
driver.Initialize()
driver.AddTypicalLights()
driver.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
driver.AddSkyBox()
driver.AddCamera(chrono.ChVector3d(2,2,3))
driver.AddTypicalOrbitCamera(chrono.ChVector3d(2,2,3),0.5,100.0)
driver.AddLightWithShadow(chrono.ChVector3d(2,10,5),chrono.ChVector3d(0,0,0),50,100.0,512)




driver.SetMouseTimeResponse(0.2)




driver.SetDriverSystemMode(driver.DRIVERSYS_MODE_GETTIME | driver.DRIVERSYS_MODE_VISUALIZATION | driver.DRIVERSYS_MODE_MOUSE)


pend1.SetStepSize(1e-3)
pend2.SetStepSize(1e-3)



ang1 = 0
ang2 = 0







plot = False



path = [
    chrono.ChVector3d(-2, 0, 1.2),
    chrono.ChVector3d(-1.5, -0.5, 1.1),
    chrono.ChVector3d(-1, -1, 1.0),
    chrono.ChVector3d(-0.5, -1.5, 0.9),
    chrono.ChVector3d(0, -2, 0.8),
    chrono.ChVector3d(0.5, -2.5, 0.7),
    chrono.ChVector3d(1, -3, 0.6),
    chrono.ChVector3d(1.5, -3.5, 0.5),
    chrono.ChVector3d(2, -4, 0.4),
    chrono.ChVector3d(2.5, -4.5, 0.3),
    chrono.ChVector3d(3, -5, 0.2),
    chrono.ChVector3d(3.5, -5.5, 0.1),
    chrono.ChVector3d(4, -6, 0),
    ]
plot_path = True





print ("Double Pendulum Demo\n")


vis1 = 0
vis2 = 0



def change_mass_callback(mass):
    pend1.SetMass(mass)
    pend2.SetMass(mass)
    print('Mass changed to ' + str(mass) + '\n')
    return mass * 1.1


if (change_mass):
    change_mass_callback_data = [1.0, 0.5]


time = 0
time_end = 100

while (time < time_end):

    
    time = sys.GetChTime()

    
    if (time >= time_end):
        break

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    vis1 = pend1.GetAngleY() * (180.0 / chrono.CH_PI)
    vis2 = pend2.GetAngleY() * (180.0 / chrono.CH_PI)

    
    print("Time = " + str(time) + "  |ang1| = " + str(abs(vis1)) + "  |ang2| = " + str(abs(vis2)) + "\n")

    
    if (abs(vis1) >= 100):
        break

    
    
    if (driver.HasRendered()) :
        mposx = driver.GetMouseX()
        mposy = driver.GetMouseY()
        ratio = (mposx - 0.5) * 0.2
        if (link1.GetA() > 0):
            link1.SetA(2 + ratio)
            link2.SetA(2 + ratio)
        vis.Synchronize()
        driver.Synchronize()

    
    sys.DoStepDynamics(0.0025)


return