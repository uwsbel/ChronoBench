import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import os

# Initialize Chrono system and Irrlicht visualization
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
app = chronoirr.ChIrrApp(system, "HMMWV on Custom Mesh Terrain", chronoirr.dimension2du(1280, 720))
app.SetSkyBox()
app.SetCamera(chronoirr.vector3df(0, 2, -8), chronoirr.vector3df(0, 0, 0), chronoirr.vector3df(0, 1, 0))
app.SetLightDirection(chronoirr.vector3df(0, -1, 0), True)

# Create HMMWV vehicle with TMEASY tires and mesh visualization
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.5), chrono.QUNIT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(1e-3)
hmmwv.SetVehicleStepSize(1e-3)
hmmwv.Initialize()
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# Add custom mesh terrain
terrain = veh.RigidTerrain(system)
mesh_file_col = os.path.join('Highway_col.obj')
mesh_file_vis = os.path.join('Highway_vis.obj')
ground_mat = chrono.ChMaterialSurfaceNSC()
ground_mat.SetFriction(0.9)
ground_mat.SetRestitution(0.01)
ground_mat.SetRollingFriction(0.01)
terrain.AddMesh(chrono.ChCoordsysd(), mesh_file_col, mesh_file_vis, ground_mat, 1000, True)
terrain.Initialize()

# Create interactive driver
driver = veh.ChInteractiveDriverIRR(hmmwv, app)
driver.SetSteeringDelta(0.05)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)
driver.SetMaxSteering(0.6)
driver.Initialize()

# Simulation loop parameters
time_step = 1e-3
real_time_step = 1.0 / 50.0
simulation_time = 0.0

while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()

    # Update driver inputs
    driver.Update()

    # Advance simulation
    system.DoStepDynamics(real_time_step)
    simulation_time += real_time_step

    # Update vehicle and terrain
    hmmwv.Update(simulation_time)
    terrain.Synchronize(simulation_time)

    app.EndScene()
    chrono.ChCollisionSystemBase.WaitIfBulletRealTime(0.02)