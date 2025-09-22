import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.sensor as sens


print("Copyright (c) 2023 Project Chrono")


sys = chrono.ChSystemNSC()


ground_mat = chrono.ChMaterialSurfaceNSC()
ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetCollide(True)
ground.GetCollisionModel().ClearModel()
ground.GetCollisionModel().AddBox(ground_mat, 100, 1, 100, chrono.ChFrame(chrono.ChVector3f(0, -1, 0)))
ground.GetCollisionModel().BuildModel()
ground.SetPos(chrono.ChVector3f(0, 0, 0))
sys.Add(ground)


init_loc = chrono.ChVector3f(0, 0.5, -5)
init_rot = chrono.ChQuaternionf(1, 0, 0, 0)
rover = veh.Curiosity(sys, init_loc, init_rot)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Curiosity Rover Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3f(5, 5, 5), chrono.ChVector3f(0, 0, 0))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3f(10, 15, 10), chrono.ChVector3f(0, 2.5, 0), 15, 4, 15, 120)
vis.EnableShadows()


while vis.Run():
    time = sys.GetChTime()
    sys.DoStepDynamics(1e-3)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    if time % 4 < 2:
        rover.SetSteering(0.5)
    else:
        rover.SetSteering(-0.5)

    
    rover.Update()

    
    vis.GetDevice().run()