import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.sensor as sens


print("Copyright (c) 2023 Project Chrono")


sys = chrono.ChSystemNSC()


ground_mat = chrono.ChMaterialSurfaceNSC()
ground_mat.SetFriction(0.9)
ground_mat.SetRestitution(0.0)

ground_body = chrono.ChBody()
ground_body.SetBodyFixed(True)
ground_body.SetPos(chrono.ChVectorD(0, 0, 0))
ground_shape = chrono.ChBoxShape()
ground_shape.GetBoxGeometry().Size = chrono.ChVectorD(100, 1, 100)
ground_body.AddVisualShape(ground_shape)
ground_body.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(ground_body)


curiosity_init_pos = chrono.ChFrameD(chrono.ChVectorD(0, 1, 0))
curiosity = veh.Curiosity(sys, curiosity_init_pos)


driver = veh.ChDriver(curiosity.GetVehicle())
driver.SetSteering(0.0)
driver.SetThrottle(0.3)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Curiosity Rover Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(5, 5, 5), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(10, 15, 10), chrono.ChVectorD(0, 0, 0), 30, 5, 15, 60)
vis.EnableShadows()


while vis.Run():
    time = sys.GetChTime()
    driver_inputs = driver.GetInputs()
    curiosity.SetDriverInputs(driver_inputs)
    sys.DoStepDynamics(1e-3)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()