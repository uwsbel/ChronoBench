import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math

# ---------------------------------------------------------------------
#  Create the simulation system and set default parameters
# ---------------------------------------------------------------------
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Set default contact material properties
ground_material = chrono.ChMaterialSurfaceNSC()
ground_material.SetFriction(0.9)
ground_material.SetRestitution(0.1)

# ---------------------------------------------------------------------
#  Create the UAZBUS vehicle
# ---------------------------------------------------------------------
vehicle = veh.ChWheeledVehicle(system, "UAZBUS")
vehicle.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 1.5, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringType(veh.SteeringType_PITMAN_ARM)
vehicle.SetDriveType(veh.DriveType_AWD)
vehicle.SetTireType(veh.TireModelType_TMEASY)

# Set initial vehicle conditions
vehicle.SetVehicleSpeed(0, 1)

# ---------------------------------------------------------------------
#  Create the rigid terrain
# ---------------------------------------------------------------------
terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(ground_material, 
                         chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT), 
                         200, 200)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 20, 20)
terrain.Initialize()

# ---------------------------------------------------------------------
#  Create the driver system
# ---------------------------------------------------------------------
driver = veh.ChDriver(system)
driver.Initialize()

# ---------------------------------------------------------------------
#  Create the visualization application
# ---------------------------------------------------------------------
vis = irr.ChIrrApp(system, "UAZBUS Simulation", irr.dimension2du(1280, 720))
vis.SetSkyBox()
vis.AddTypicalLights(chrono.ChVectorD(30, -30, 100), chrono.ChVectorD(30, 30, 100), 250, 130, 130)
vis.AddTypicalCamera(chrono.ChVectorD(10, 5, 15), chrono.ChVectorD(0, 1, 0))

# ---------------------------------------------------------------------
#  Simulation loop
# ---------------------------------------------------------------------
time_step = 1e-3
render_step = 1 / 60

while vis.GetDevice().run():
    time = system.GetChTime()

    # Synchronize vehicle systems
    vehicle.Synchronize(time)

    # Advance the simulation by one step
    system.DoStepDynamics(time_step)

    # Update the driver inputs
    driver.Synchronize(time)

    # Update visualization
    if time % render_step <= 1e-4:
        vis.BeginScene()
        vis.DrawAll()
        vis.EndScene()