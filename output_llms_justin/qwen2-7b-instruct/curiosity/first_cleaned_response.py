import pychrono as chrono
import pychrono.irrlicht as chronoirr


chrono.Ch纪元.Initialize()


def create_terrain():
    
    ground = chrono.ChBodyEasyBox(100, 1, 1, 1000, True, True)
    ground.SetPos(chrono.ChVectorD(0, -1, 0))
    ground.SetBodyFixed(True)
    ground.SetCollide(True)
    ground.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())
    ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("terrain/earth.jpg"))
    return ground


terrain = create_terrain()


def create_rover():
    
    rover = chrono.ChBodyEasyImportTetGen(chrono.GetChronoDataFile("models/curiosity_rover.obj"), 1000, True, True)
    rover.SetPos(chrono.ChVectorD(0, 0, 0))
    rover.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
    motor = chrono.ChMotorDriver()
    motor.SetBody(rover)
    motor.SetMaxTorque(chrono.ChVectorD(0, 0, 1000))
    motor.SetMaxSpeed(chrono.ChVectorD(0, 0, 10))
    return rover, motor


rover, motor = create_rover()


vis_system = chronoirr.ChIrrApp(chrono.GetSystem(), "PyChrono Curiosity Rover Simulation", chronoirr.dimension2du(1280, 720))
vis_system.AddTypicalLogo("logo.ppm")
vis_system.AddTypicalSky()
vis_system.AddTypicalLights()
vis_system.AddTypicalCamera(chrono.ChVectorD(0, 0, 10))
vis_system.AddTypicalLogo("logo.ppm", 0.15, chrono.GetChronoDataFile("screenshots/"))


vis_system.AddTypicalMesh(terrain)
vis_system.AddTypicalMesh(rover)


while vis_system.GetDevice().run():
    
    motor.Update()
    motor.Step()
    vis_system.BeginScene()
    vis_system.DrawAll()
    vis_system.EndScene()


chrono.Ch纪元.Finalize()