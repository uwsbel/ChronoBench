import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as robot
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.sensor as sens


system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)


terrain = veh.SCMTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200, 100)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
terrain.Initialize()


m113 = veh.M113()
m113.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, -1.5, 1.1), chrono.ChQuaterniond(1, 0, 0, 0)))
m113.SetEngineType(veh.EngineModelType_SHAFTS)
m113.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
m113.SetDriveType(veh.DrivelineTypeWV_AWD)
m113.SetTireType(veh.TireModelType_TMEASY)
m113.SetTireStepSize(1e-3)
m113.Initialize()
m113.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
m113.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
m113.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
m113.SetWheelVisualizationType(veh.VisualizationType_NONE)
m113.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)

m113.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('M113')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.75), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(m113.GetVehicle())


driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(0.06)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)
driver.Initialize()


while vis.Run() :
    time = m113.GetSystem().GetChTime()

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    m113.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    m113.Advance(step_size)
    vis.Advance(step_size)