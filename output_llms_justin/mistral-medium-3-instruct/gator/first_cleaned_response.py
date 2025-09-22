import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.vehicle.gator as gator
import pychrono.vehicle.terrain as terrain
import math






vehicle = veh.WheelVehicle(chrono.GetChronoVehicleDataPath())
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
vehicle.SetChassisFixed(False)
vehicle.SetInitializeFwdVel(0.0)


vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)


gator.Create(vehicle)


vehicle.Initialize(chrono.ChCoordsys<>(chrono.ChVectorD<>(0, 0.5, 0), chrono.ChQuaternionD<>(1, 0, 0, 0)))


terrain = terrain.RigidTerrain(vehicle.GetSystem())
terrain.SetContactMaterialProperties(2e7, 0.8)
terrain.SetTexture(chrono.GetChronoDataPath() + "vehicle/terrain/textures/tile4.jpg", 200, 200)
terrain.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize(20, 20, 0, -2)


app = chronoirr.ChIrrApp(vehicle.GetSystem(), 'Gator Vehicle Demo', chrono.irr.dimension2d<int>(1280, 720))
app.AddTypicalLogo()
app.AddTypicalSky()
app.AddTypicalLights()
app.AddTypicalCamera(chrono.irr.core.vector3df(0, 1.5, -5))
app.AssetBindAll()
app.AssetUpdateAll()


driver = veh.ChInteractiveDriverIRR(app)
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.01)
driver.SetBrakingDelta(0.02)
driver.Initialize()






step_size = 0.01
time_end = 100
time = 0


while app.GetDevice().run() and time < time_end:
    time = vehicle.GetSystem().GetChTime()

    
    driver_inputs = driver.GetInputs()

    
    vehicle.Update(driver_inputs.m_throttle, driver_inputs.m_steering, driver_inputs.m_braking, step_size)

    
    app.BeginScene()
    app.DrawAll()
    app.DoStepDynamics(step_size)
    app.EndScene()

    
    time += step_size