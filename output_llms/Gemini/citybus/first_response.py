import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math

# -------------------------------------------------------------------
# Create the Chrono simulation system
# -------------------------------------------------------------------
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# -------------------------------------------------------------------
# Create the vehicle system
# -------------------------------------------------------------------
vehicle = chrono.ChVehicle(system, chrono.ChMaterialSurfaceNSC())
vehicle.SetChassisFixed(False)
vehicle.SetChassisCollisionType(chrono.ChCollisionSystem.Type_MESH)
vehicle.SetChassisMeshFilename("citybus/body.obj")
vehicle.SetCollide(True)
vehicle.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 1.5, 0)))

# Set vehicle parameters
vehicle.SetMass(5000)
vehicle.SetInertiaXX(chrono.ChVectorD(1200, 1000, 2500))
vehicle.SetCOMlocation(chrono.ChVectorD(0, 0.5, 0))

# -------------------------------------------------------------------
# Create the tire system
# -------------------------------------------------------------------
tire_model = chrono.ChPacejkaTire("generic_tire")
tire_model.Load("tire_data.tir")

# Set tire parameters
tire_model.SetFriction(1.0)
tire_model.SetVerticalStiffness(200000)
tire_model.SetDamping(50)

# Create the axles and wheels
for axle_index in range(2):
    axle = chrono.ChVehicleRigidAxle(vehicle, "axle_" + str(axle_index))
    vehicle.AddAxle(axle)
    axle.SetMass(100)
    axle.SetInertia(chrono.ChVectorD(20, 20, 20))

    for wheel_index in range(2):
        wheel = chrono.ChVehicleRigidWheel(axle, "wheel_" + str(axle_index) + "_" + str(wheel_index), tire_model)
        axle.AddWheel(wheel)
        wheel.SetMass(50)
        wheel.SetInertia(chrono.ChVectorD(10, 10, 10))
        wheel.SetVisualizationType(chrono.VisualizationType_MESH)
        wheel.SetMeshFilename("citybus/wheel.obj")

        if axle_index == 0:
            wheel.SetSteeringAngle(0)
        else:
            wheel.SetBrakeTorque(0)

# -------------------------------------------------------------------
# Create the terrain system
# -------------------------------------------------------------------
terrain = chrono.ChTerrain(system)
terrain.Initialize(chrono.ChCoordsysD(), 100, 100, 0.1)
terrain.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))

# -------------------------------------------------------------------
# Create the Irrlicht visualization system
# -------------------------------------------------------------------
application = chronoirr.ChIrrApp(system)
application.AddTypicalLights()
application.AddSkyBox()
application.SetShowProfiler(True)
application.SetTimestep(0.02)
application.AssetBindAll()
application.AssetUpdateAll()

# Set camera settings
camera = application.GetDevice().getSceneManager().addCameraSceneNode(
    None,
    chrono.ChVectorD(0, 5, -10),
    chrono.ChVectorD(0, 1, 0),
)
application.GetDevice().getSceneManager().setActiveCamera(camera)

# -------------------------------------------------------------------
# Simulation loop
# -------------------------------------------------------------------
application.SetStepManage(True)

while application.GetDevice().run():
    # Get user inputs for steering, throttle, and braking
    steering_input = 0.0
    throttle_input = 0.0
    braking_input = 0.0

    # Apply user inputs to the vehicle
    for axle_index in range(2):
        for wheel_index in range(2):
            wheel = vehicle.GetAxle(axle_index).GetWheel(wheel_index)

            if axle_index == 0:
                wheel.SetSteeringAngle(steering_input)
            else:
                wheel.SetBrakeTorque(braking_input)

    vehicle.GetEngine().SetThrottle(throttle_input)

    # Update the camera position to follow the vehicle
    camera.setPosition(vehicle.GetChassisBody().GetPos() + chrono.ChVectorD(0, 5, -10))
    camera.setTarget(vehicle.GetChassisBody().GetPos())

    # Advance the simulation
    application.DoStep()
    application.DrawAll()

    # Set the simulation frame rate
    chrono.ChRealtimeStepTimer().Spin(1.0 / 50)