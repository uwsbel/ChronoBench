# =============================================================================
# BMW 3-Series (E90) – interactive, real-time demo
#
# * Rigid-terrain driving demo using a Sedan vehicle that we will badge as a
#   BMW E90.  (The Chrono “Sedan” template matches the size and axle layout
#   of a compact four-door, so it is a reasonable stand-in.)
# * TMeasy tire model on all four wheels
# * Interactive Irrlicht driver (steer / throttle / brake)
# * Irrlicht visualisation with sky-box, chase camera, lights, logo & terrain
#   texture
# * Uses the SMC contact method – perfectly fine for rigid terrain.
# =============================================================================
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

# -----------------------------------------------------------------------------
# 1. Global Chrono initialisation
# -----------------------------------------------------------------------------
chrono.SetChronoDataPath(chrono.GetChronoDataFile(""))          # core data
veh.SetDataPath(chrono.GetChronoDataFile("vehicle/"))           # vehicle data

# -----------------------------------------------------------------------------
# 2. Create the physical system and set basic parameters
# -----------------------------------------------------------------------------
system          = chrono.ChSystemSMC()          # smooth-contact formulation
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

# Time step parameters --------------------------------------------------------
step_size       = 2.0e-3                        # dynamics time step (s)
render_fps      = 60.0                          # Irrlicht redraw rate (Hz)
render_steps    = int(1.0/(render_fps*step_size))

# -----------------------------------------------------------------------------
# 3. Build the rigid terrain
# -----------------------------------------------------------------------------
terrain         = veh.RigidTerrain(system)

mat_terr        = chrono.ChMaterialSurfaceSMC()
mat_terr.SetFriction(0.9)
mat_terr.SetRestitution(0.01)
mat_terr.SetYoungModulus(2e7)

patch_dimX      = 250.0     # length (m)
patch_dimY      = 250.0     # width  (m)
patch_thick     = 0.1       # thickness (m)

patch = terrain.AddPatch(
    mat_terr,
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
    patch_dimX,
    patch_dimY,
    patch_thick
)

# --- custom texture / colour / logo
patch.SetTexture(chrono.GetChronoDataFile("terrain/textures/tile4.jpg"),
                 tex_scale_x=patch_dimX*0.6,
                 tex_scale_y=patch_dimY*0.6)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))
terrain.Initialize()

# -----------------------------------------------------------------------------
# 4. Vehicle – “Sedan” template, renamed BMW E90, with TMeasy tyres
# -----------------------------------------------------------------------------
init_pos        = chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.40),
                                     chrono.QUNIT)

vehicle         = veh.Sedan(system,
                            fixed=False,
                            brake_type=veh.BrakeType.SHAFTS,
                            contact_method=chrono.ChContactMethod_SMC)

vehicle.SetName("BMW_E90")
vehicle.Initialize(init_pos)

# Visualisation switches -------------------------------------------------------
vehicle.SetChassisVisualizationType   (veh.VisualizationType.MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType.PRIMITIVES)
vehicle.SetSteeringVisualizationType  (veh.VisualizationType.PRIMITIVES)
vehicle.SetWheelVisualizationType     (veh.VisualizationType.MESH)

# --- Replace default tyres by four explicit TMeasy tyres
tire_vis = veh.VisualizationType.MESH

for axle in vehicle.GetAxles():
    for wheel in axle.m_wheels:
        wheel_index = wheel.GetId()
        side        = "L" if wheel_index % 2 == 0 else "R"
        axle_id     = int(wheel_index/2)
        label       = f"TMeasy_A{axle_id}_{side}"
        tire        = veh.TMeasyTire(label)
        tire.SetVisualizationType(tire_vis)
        tire.Initialize(wheel)

# -----------------------------------------------------------------------------
# 5. Irrlicht application & driver (real-time user inputs)
# -----------------------------------------------------------------------------
app = veh.ChWheeledVehicleIrrApp(vehicle,
                                 "BMW E90 sedan – TMeasy tires (interactive)",
                                 irr.dimension2du(1280, 720))

# Visual elements -------------------------------------------------------------
app.SetSkyBox()
app.AddTypicalLogo()
app.AddTypicalLights(chrono.ChVector3df( 150, 100, 200),
                     chrono.ChVector3df(-150,-100, 200),
                     250, 130)
app.SetChaseCamera(chrono.ChVectorD(0.0, 0.5, 0.0), 6.0, 0.4)   # (ptOnChassis, dist, elev)

app.Initialize()

# Interactive GUI driver ------------------------------------------------------
driver = veh.ChIrrGuiDriver(app)
driver.SetSteeringDelta(0.035)      # radians per key press
driver.SetThrottleDelta(0.02)       # throttle increment   per key press
driver.SetBrakingDelta (0.08)       # braking increment    per key press
driver.Initialize()

# -----------------------------------------------------------------------------
# 6. Simulation loop
# -----------------------------------------------------------------------------
step_number = 0
while app.GetDevice().run():

    time = system.GetChTime()

    # Render scene
    if step_number % render_steps == 0:
        app.BeginScene(True, True, chrono.SColor(255, 140, 161, 192))
        app.DrawAll()
        app.EndScene()

    # Collect driver inputs ----------------------------------------------------
    driver_inputs = driver.GetInputs()        # steering, throttle, braking

    # Module synchronisation ---------------------------------------------------
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    app.Synchronize("BMW E90 demo", driver_inputs)

    # Advance all modules ------------------------------------------------------
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    app.Advance(step_size)

    # Integrate dynamics -------------------------------------------------------
    system.DoStepDynamics(step_size)

    step_number += 1