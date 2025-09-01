import os
import math
import chrono
import chrono.vehicle as veh
import chrono.irrlicht as irr

# =============================================================================
# Initialize simulation environment
# =============================================================================

system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# =============================================================================
# Create HMMWV vehicle
# =============================================================================

# Initialize HMMWV at specified position
init_pos = chrono.ChVectorD(0, 0.5, 0)
hmmwv = veh.HMMWV_Full(system)
hmmwv.SetInitPosition(chrono.ChCoordsysD(init_pos, chrono.QUNIT))
hmmwv.SetTireType(veh.RigidTire)  # Use rigid tires for SCM interaction
hmmwv.Initialize()

# Configure visualization
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# =============================================================================
# Create SCM deformable terrain
# =============================================================================

terrain = veh.SCMDeformableTerrain(system)
terrain.SetPlane(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))

# Configure soil parameters
terrain.SetSoilParameters(
    2e6,        # Bekker Kphi
    0,          # Bekker Kc
    1.1,        # Bekker n exponent
    17,         # Mohr cohesive limit (Pa)
    30,         # Mohr friction angle (degrees)
    1e7,        # Janosi shear coefficient (m)
    2e4,        # Elastic stiffness (Pa/m)
    3e4         # Damping (Pa s/m)
)

# Initialize terrain with height map (flat plane in this case)
terrain.Initialize(40, 40, 0.1)  # 40x40 meter patch with 0.1m resolution

# Enable pressure visualization
terrain.SetPlotType(veh.SCMDeformableTerrain.PLOT_PRESSURE, 0, 30000)
terrain.SetVisualization(True)

# =============================================================================
# Create visualization system
# =============================================================================

app = veh.ChVehicleIrrApp(hmmwv.GetVehicle(), "HMMWV SCM Demo")
app.AddTypicalLights()
app.AddSkyBox()
app.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 1.75), 6.0, 0.5)
app.SetTimestep(0.02)

# =============================================================================
# Initialize driver system
# =============================================================================

driver = app.GetDriver()
driver.Initialize()

# =============================================================================
# Simulation loop
# =============================================================================

time_step = 0.02
while app.GetDevice().run():
    time = system.GetChTime()
    
    # Render scene
    app.BeginScene()
    app.DrawAll()
    app.EndScene()
    
    # Get driver inputs
    driver_inputs = driver.GetInputs()
    
    # Synchronize systems
    driver.Synchronize(time)
    hmmwv.Synchronize(time, driver_inputs)
    terrain.Synchronize(time)
    app.Synchronize("", driver_inputs)
    
    # Advance simulation
    app.Advance(time_step)
    system.DoStepDynamics(time_step)

    # Maintain real-time synchronization
    if not app.GetPaused():
        app.Update()