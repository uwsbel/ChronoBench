import chrono

import chrono.irrlicht as ir




app = chrono.ChApplication(chrono.VNULL, False, True, 50.0)




floor = chrono.ChBody(

    name="floor",

    mass=1000.0,

    invInertiaXX=1.0,

    invInertiaXY=0.0,

    invInertiaYZ=0.0,

    invInertiaZZ=1.0,

    pos=chrono.ChVectorD(0.0, 0.0, 0.0),

    eulerAngles=chrono.ChQuaternionD(0.0, 0.0, 0.0),

    type=chrono.ChBody.CT_TRUSSES)




crankshaft = chrono.ChBody(

    name="crankshaft",

    mass=50.0,

    invInertiaXX=1.0,

    invInertiaXY=0.0,

    invInertiaYZ=0.0,

    invInertiaZZ=1.0,

    pos=chrono.ChVectorD(0.0, 0.0, 0.0),

    eulerAngles=chrono.ChQuaternionD(0.0, 0.0, 0.0),

    type=chrono.ChBody.CT_ROTATION)




connecting_rod = chrono.ChBody(

    name="connecting_rod",

    mass=10.0,

    invInertiaXX=1.0,

    invInertiaXY=0.0,

    invInertiaYZ=0.0,

    invInertiaZZ=1.0,

    pos=chrono.ChVectorD(0.0, 0.0, 0.0),

    eulerAngles=chrono.ChQuaternionD(0.0, 0.0, 0.0),

    type=chrono.ChBody.CT_ROTATION)




piston = chrono.ChBody(

    name="piston",

    mass=5.0,

    invInertiaXX=1.0,

    invInertiaXY=0.0,

    invInertiaYZ=0.0,

    invInertiaZZ=1.0,

    pos=chrono.ChVectorD(0.0, 0.0, 0.0),

    eulerAngles=chrono.ChQuaternionD(0.0, 0.0, 0.0),

    type=chrono.ChBody.CT_ROTATION)




crankshaft_joint = chrono.ChLinkUDrvCrank(

    name="crankshaft_joint",

    bodyA=crankshaft,

    bodyB=piston,

    axis=chrono.ChVectorD(0.0, 1.0, 0.0),

    parentA=crankshaft,

    parentB=piston)


connecting_rod_joint = chrono.ChLinkUDrvRod(

    name="connecting_rod_joint",

    bodyA=connecting_rod,

    bodyB=piston,

    axis=chrono.ChVectorD(0.0, 0.0, 1.0),

    parentA=connecting_rod,

    parentB=piston)




motor = chrono.ChLinkUDrvMotor(

    name="motor",

    bodyA=crankshaft,

    bodyB=crankshaft_joint,

    axis=chrono.ChVectorD(1.0, 0.0, 0.0),

    parentA=crankshaft,

    parentB=crankshaft_joint)




crankshaft.SetPos(chrono.ChVectorD(0.0, 0.0, 0.0))

piston.SetPos(chrono.ChVectorD(0.0, 0.0, 0.0))

connecting_rod.SetPos(chrono.ChVectorD(0.0, 0.0, 0.0))




app.Add(floor)

app.Add(crankshaft)

app.Add(connecting_rod)

app.Add(piston)




vis = ir.IrrlichtVisualizer(app)

vis.SetWindowTitle("Crank-Slider Mechanism")

vis.SetWindowSize(1280, 720)

vis.SetWindowVisible(True)




camera = ir.IrrlichtDevice.GetCamera()

camera.SetPosition(chrono.ChVectorD(5.0, 5.0, 5.0))

camera.SetFov(60.0)




light = ir.IrrlichtDevice.GetLight()

light.SetColor(ir.vector3df(1.0, 1.0, 1.0))

light.SetPos(chrono.ChVectorD(0.0, 0.0, 10.0))




logo = ir.IrrlichtDevice.GetLogo()

logo.SetTexture(ir.vector4df(1.0, 1.0, 1.0, 1.0))

logo.SetPosition(chrono.ChVectorD(0.0, 0.0, 0.0))




app.Step(1.0 / 60.0, True)




app.RenderGraphics()

app.MainLoop()