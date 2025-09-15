import os
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetSolverType(chrono.ChSolver.Type_PSOR)
system.SetSolverMaxIterations(100)


terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysD(), 200, 100)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
terrain.Initialize()


rover = veh.Viper(system)
rover.SetDriverCollision(True)
rover.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 1, 0), chrono.QUNIT))
rover.SetChassisVisualizationType(veh.VisualizationType_MESH)
rover.SetWheelVisualizationType(veh.VisualizationType_MESH)


driver = veh.InteractiveDriver(rover.GetVehicle())
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.Initialize()


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Curiosity Rover Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(5, 3, -5))
vis.AddLightWithShadow(chrono.ChVectorD(10, 10, -5), chrono.ChVectorD(0, 0, 0), 50, 5, 50, 512)


vis.EnableShadows(True)
vis.EnableAntiAliasing(True)
vis.EnableGrid(False)
rover.GetVehicle().GetChassis().GetBody().GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/rover_body.png"))


time_step = 0.005
while vis.Run():
    vis.BeginScene()
    vis.Render()
    
    
    driver.Synchronize(time_step)
    rover.Synchronize(time_step, driver.GetInputs())
    system.Synchronize(time_step)
    system.DoStepDynamics(time_step)
    
    
    cam_pos = rover.GetChassisPos() + chrono.ChVectorD(-5, 2, -5)
    vis.GetActiveCamera().setTarget(irr.vector3df(rover.GetChassisPos().x, rover.GetChassisPos().y, rover.GetChassisPos().z))
    
    
    vis.GetGUIEnvironment().addStaticText(
        "NASA/JPL - Curiosity Rover Simulation\nUse Arrow Keys to Drive\nW/S - Throttle\nA/D - Steer",
        irr.recti(10, 10, 400, 80), True, False)
    
    vis.EndScene()