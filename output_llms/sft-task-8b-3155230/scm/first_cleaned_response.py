import pychrono as chrono
from pychrono.vehicle import HMMWV, SCMDeformableTerrain, DriverType, DriverKeyboard
from pychrono import irrlicht
import numpy as np


system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))


terrain = SCMDeformableTerrain(system)
terrain.SetSoilParams(1800, 100000, 10000, 0.1, 0.05, 0.01, 0.005)
terrain.SetMovingPatch(True)
terrain.SetColor(chrono.ChColor(0.3, 0.6, 0.3))
terrain.Initialize(100, 100, 0.5, 0.5)
terrain.SetPlotting(True)
terrain.SetFalseColor(True)


vehicle = HMMWV.Final()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0.5, 0.5), chrono.QUNIT))
vehicle.SetTireType(HMMWV.TireType_RIGID)
vehicle.SetTrackShed(True)
vehicle.SetBrakeType(HMMWV.BrakeType_SIMPLE)
vehicle.SetEngineType(HMMWV.EngineType_SIMPLE)
vehicle.SetUseGearBox(True)
vehicle.Initialize(chrono.ChVector3d(0, 0.5, 0.5), chrono.QUNIT, terrain)
vehicle.SetVisualizationType(HMMWV.VisualizationType_MESH)


driver = DriverKeyboard(vehicle)
driver.SetSteeringDelta(0.05)
driver.SetThrottleDelta(0.05)
driver.SetBrakingDelta(0.05)


vis = irrlicht.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('HMMWV on SCM Terrain')
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2, 2), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()


time = 0.0
step = 0.001
while vis.Run():
    time += step
    vis.BeginScene()
    vis.Render()
    
    
    driver.Synchronize(time)
    driver.UpdateControls()
    
    
    vehicle.Synchronize(time, driver.GetInputs())
    terrain.Synchronize(time)
    
    
    vehicle.DoStepDynamics(step)
    terrain.DoStepDynamics(step)
    
    vis.EndScene()
    chrono.ChTimeStep.SetTimeStep(step)