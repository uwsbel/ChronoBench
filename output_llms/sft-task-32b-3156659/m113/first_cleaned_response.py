import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import numpy as np


system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))


vehicle = veh.M113()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.5), chrono.QUNIT))
vehicle.SetInitFwdVel(0)
vehicle.SetPowertrainType(veh.PowertrainModelType_SIMPLE)
vehicle.SetDriveType(veh.DrivelineTypeWV_AWD)
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.Initialize()
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)


terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 100.0, 100.0)
patch.SetFriction(0.9)
patch.SetRestitution(0.01)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


driver = veh.ChIrrGuiDriver(vehicle.GetVehicle())
driver.SetSteeringDelta(1.0)
driver.SetThrottleDelta(1.2)
driver.SetBrakingDelta(0.8)
driver.SetInputs(veh.DriverInputs(0, 0, 0))


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('M113 Vehicle Simulation')
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(6, 3, 3), chrono.ChVector3d(0, 0, 1.7))
vis.AddTypicalLights()
vis.EnableShadows(True)


time_step = 0.001
real_time_factor = 1.0
sim_time = 0.0


while vis.Run() and not driver.GetExit():
    
    driver_inputs = driver.GetInputs()
    
    
    vehicle.Synchronize(sim_time, driver_inputs, terrain)
    terrain.Synchronize(sim_time)
    driver.Synchronize(sim_time)
    
    
    vehicle.Advance(time_step)
    terrain.Advance(time_step)
    driver.Advance(time_step)
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    sim_time += time_step

    
    chrono.ChRealtimeStepDamping(time_step)