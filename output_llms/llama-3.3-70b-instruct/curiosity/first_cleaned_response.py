import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


sys = chrono.ChSystemNSC()


ground = chrono.ChBodyEasyBox(sys, 10, 1, 10, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -1, 0))
ground.SetBodyFixed(True)
sys.Add(ground)


rover = veh.ChWheeledVehicle(sys)
rover.AddChassis(chrono.ChBodyEasyBox(sys, 1, 0.5, 1, 100, True, True))
rover.AddWheel(chrono.ChWheel(sys, chrono.ChVectorD(0.5, 0, 0.5), chrono.ChVectorD(0, 1, 0), 0.2, 0.1))
rover.AddWheel(chrono.ChWheel(sys, chrono.ChVectorD(-0.5, 0, 0.5), chrono.ChVectorD(0, 1, 0), 0.2, 0.1))
rover.AddWheel(chrono.ChWheel(sys, chrono.ChVectorD(0.5, 0, -0.5), chrono.ChVectorD(0, 1, 0), 0.2, 0.1))
rover.AddWheel(chrono.ChWheel(sys, chrono.ChVectorD(-0.5, 0, -0.5), chrono.ChVectorD(0, 1, 0), 0.2, 0.1))
rover.SetMotorType(veh.ChVehicleMotorType.MOTOR_TYPE_TORQUE)
rover.SetMotorForce(100)
sys.Add(rover.GetChassis())


rover.GetChassis().SetPos(chrono.ChVectorD(0, 1, 0))
rover.GetChassis().SetRot(chrono.ChQuaternionD(1, 0, 0, 0))


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(0, 5, -10))
vis.AddCamera(chrono.ChVectorD(0, 5, -10), chrono.ChVectorD(0, 0, 0), 30)


vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Curiosity Rover Simulation")


vis.GetCamera().SetPosition(chrono.ChVectorD(0, 5, -10))
vis.GetCamera().SetLookAt(chrono.ChVectorD(0, 0, 0))


vis.AddTexture(ground, "ground_texture.png")
vis.AddLogo("NASA_logo.png", chrono.ChVectorD(0.5, 0.5, 0))


while vis.Run():
    sys.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()