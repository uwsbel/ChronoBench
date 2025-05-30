import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import os

# ---------------------------------------------------------------------
# Create the simulation system and set default parameters
# ---------------------------------------------------------------------
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))  # Set gravitational acceleration

# Set default contact method
chrono.SetChronoDataPath(os.environ['CHRONO_DATA_DIR'])
contact_method = chrono.ChContactMethod_NSC
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.005)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.005)

# ---------------------------------------------------------------------
# Create the vehicle and set its parameters
# ---------------------------------------------------------------------
vehicle = veh.GatorVehicle(system)
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisFixed(False)
vehicle.SetChassisCollisionType(veh.ChassisCollisionType_MESH)
vehicle.SetChassisMeshFilename(chrono.GetChronoDataFile("vehicle/gator/obj/gator.obj"))
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1), chrono.ChQuaternionD(1, 0, 0, 0)))
vehicle.Initialize()

# Set tire model to TMEasy
tire_model = veh.TMeasyTire()
tire_model.LoadTireData(chrono.GetChronoDataFile("vehicle/gator/tire/HMMWV_RadialTire.tir"))
for axle in vehicle.GetAxles():
    axle.SetTireModel(tire_model)

# ---------------------------------------------------------------------
# Create the terrain and set its parameters
# ---------------------------------------------------------------------
terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT), 
                        chrono.ChVectorD(100, 100, 1))  # Terrain dimensions
patch.SetContactFrictionCoefficient(0.8)
patch.SetContactRestitutionCoefficient(0.01)
patch.SetContactMaterialProperties(2e7, 0.3)  # Young's modulus, Poisson ratio
# Set custom texture (replace with your texture file)
patch.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
terrain.Initialize()

# ---------------------------------------------------------------------
# Create the driver system
# ---------------------------------------------------------------------
driver = veh.ChInteractiveDriverIRR(vehicle)
driver.Initialize()

# ---------------------------------------------------------------------
# Create the visualization application
# ---------------------------------------------------------------------
application = irr.ChIrrApp(system, "Gator Simulation", irr.dimension2du(1000, 800))
application.AddTypicalSky()
application.AddTypicalLights()
application.SetChaseCamera(vehicle.GetChassisBody(), 6.0, 0.5)
application.AssetBindAll()
application.AssetUpdateAll()

# ---------------------------------------------------------------------
# Simulation loop
# ---------------------------------------------------------------------
application.SetTimestep(0.02)  # Set time step for 50 fps

while application.GetDevice().run():
    time = system.GetChTime()

    # Update driver inputs
    driver.Synchronize(time)

    # Advance simulation by one step
    system.DoStepDynamics(application.GetTimestep())

    # Update visualization
    application.BeginScene()
    application.DrawAll()
    application.EndScene()