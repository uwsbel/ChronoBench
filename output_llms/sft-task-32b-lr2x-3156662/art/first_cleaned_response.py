import pychrono as chrono
import pychrono.vehicle as veh
import math


system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))


artcar = veh.ARTcar()
artcar.SetContactMethod(chrono.ChContactMethod.NSC)
artcar.SetChassisFixed(False)
artcar.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.5), chrono.QUNIT))
artcar.SetPowertrainType(veh.PowertrainModelType_SIMPLE)
artcar.SetTireType(veh.TireModelType_TMEASY)
artcar.SetTireStepSize(1e-3)
artcar.Initialize()

artcar.SetChassisVisualizationType(veh.VisualizationType.PRIMITIVES)
artcar.SetSuspensionVisualizationType(veh.VisualizationType.PRIMITIVES)
artcar.SetSteeringVisualizationType(veh.VisualizationType.PRIMITIVES)
artcar.SetWheelVisualizationType(veh.VisualizationType.NONE)
artcar.SetTireVisualizationType(veh.VisualizationType.NONE)


terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 20.0, 20.0)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
patch.SetTexture(chrono.GetChronoDataFile("terrain/textures/tile4.jpg"), 200, 200)
terrain.Initialize()


vis = chrono.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("PyChrono ARTcar Simulation")
vis.SetCameraVertical(chrono.CameraVerticalDirection_Z)
vis.SetCamera(chrono.ChVector3d(-6, 0, 3), chrono.ChVector3d(0, 0, 0.5))
vis.EnableShadows(True)
vis.EnableSkyBox(True)
vis.EnableEnvironmentalLighting(True)
vis.EnableLight(0, chrono.ChVector3d(0, 0, 0), chrono.ChColor(1, 1, 1), 50)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(-6, 0, 3), chrono.ChVector3d(0, 0, 0.5))
vis.AddTypicalLights()


driver = veh.ChInteractiveDriverIRR(vis)
driver.SetVehicle(artcar)
driver.SetSteeringDelta(0.05)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.05)
driver.Initialize()


time_step = 1.0 / 50.0
simulation_time = 0.0


while vis.Run():
    simulation_time = system.GetChTime()
    
    
    driver_inputs = driver.GetInputs()
    driver.Synchronize(simulation_time)
    
    
    artcar.Synchronize(simulation_time, driver_inputs, terrain)
    terrain.Synchronize(simulation_time)
    
    
    driver.Advance(time_step)
    artcar.Advance(time_step)
    terrain.Advance(time_step)
    system.DoStepDynamics(time_step)
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    vis.SuggestNextStep(time_step)
    vis.WaitNextFrame(time_step * 1000)