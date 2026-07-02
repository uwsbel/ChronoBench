import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import numpy as np


chrono.SetChronoDataPath('/usr/local/chrono/data/')
veh.SetVehicleDataPath('/usr/local/chrono_vehicle/data/')

system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVectorD(0, 0, -9.81))


vehicle = veh.Gator(system)
vehicle.SetContactMethod(veh.ContactMethod_NSC)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT))
vehicle.SetInitFwdVisual(chrono.QUNIT)
vehicle.SetPowertrainType(veh.PowertrainModelType_SIMPLE)
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_CYLINDER)
vehicle.SetTireVisualizationType(veh.VisualizationType_NONE)


terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT), 100.0, 100.0)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


driver = veh.ChInteractiveDriverIRR(vehicle)
driver.SetSteeringDelta(0.03)
driver.SetThrottleDelta(0.03)
driver.SetBrakingDelta(0.03)
driver.Initialize()


manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVectorF(10, 10, 10), chrono.ChColor(1, 1, 1), 500.0)

camera = sens.ChCameraSensor(
    vehicle.GetChassisBody(),  
    30,                        
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 2), chrono.QuatFromAngleAxisD(0, chrono.ChVectorD(0, 1, 0))),
    1280, 720,                
    1.409,                    
    0.01, 100.0              
)
camera.SetName("Vehicle Camera")
camera.SetLensFlareEnabled(False)
camera.SetDistortionEnabled(False)
manager.AddSensor(camera)


vis = chrono.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('PyChrono Gator Simulation')
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, -6, 2), chrono.ChVectorD(0, 0, 1))
vis.AddTypicalLights()


step_size = 1e-3
max_time = 10.0
time = 0

while vis.Run() and time < max_time:
    
    driver_inputs = driver.GetInputs()
    driver.Synchronize(time)
    driver.Advance(step_size)

    
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)

    
    manager.Update(time)

    
    system.DoStepDynamics(step_size)
    time += step_size

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

vis.Run()