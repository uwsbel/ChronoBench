import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import numpy as np

# 1. Initialize PyChrono environment and core components
chrono.SetChronoDataPath('/usr/local/chrono/data/')
veh.SetVehicleDataPath('/usr/local/chrono_vehicle/data/')

system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVectorD(0, 0, -9.81))

# 2. Create Gator vehicle and set visualization types
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

# 3. Create rigid terrain
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT), 100.0, 100.0)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# 4. Create interactive driver
driver = veh.ChInteractiveDriverIRR(vehicle)
driver.SetSteeringDelta(0.03)
driver.SetThrottleDelta(0.03)
driver.SetBrakingDelta(0.03)
driver.Initialize()

# 5. Create sensor manager
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVectorF(10, 10, 10), chrono.ChColor(1, 1, 1), 500.0)

camera = sens.ChCameraSensor(
    vehicle.GetChassisBody(),  # parent body
    30,                        # update rate (Hz)
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 2), chrono.QuatFromAngleAxisD(0, chrono.ChVectorD(0, 1, 0))),
    1280, 720,                # image size
    1.409,                    # horizontal FOV (radians)
    0.01, 100.0              # near/far clipping planes
)
camera.SetName("Vehicle Camera")
camera.SetLensFlareEnabled(False)
camera.SetDistortionEnabled(False)
manager.AddSensor(camera)

# 6. Set up visualization
vis = chrono.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('PyChrono Gator Simulation')
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, -6, 2), chrono.ChVectorD(0, 0, 1))
vis.AddTypicalLights()

# 7. Simulation loop parameters
step_size = 1e-3
max_time = 10.0
time = 0

while vis.Run() and time < max_time:
    # Update driver inputs
    driver_inputs = driver.GetInputs()
    driver.Synchronize(time)
    driver.Advance(step_size)

    # Update terrain and vehicle
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)

    # Update sensor manager
    manager.Update(time)

    # Advance simulation
    system.DoStepDynamics(step_size)
    time += step_size

    # Render
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

vis.Run()