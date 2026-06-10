import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                   # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')               # locate vehicle data files

init_loc = chrono.ChVector3d(-15, 0, 1.2)                              # raised spawn for the hill
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                            # QUNIT — no initial rotation
step_size = 1e-3                                                       # integration step
tire_step_size = step_size                                            # tire substep matches main step

hmmwv = veh.HMMWV_Full()                                               # full HMMWV catalog wrapper
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)                     # SMC for deformable SCM soil
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)                  # no chassis collision shape
hmmwv.SetChassisFixed(False)                                           # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))          # initial pose
hmmwv.SetTireType(veh.TireModelType_TMEASY)                            # SCM needs a non-rigid tire (RIGID won't drive)
hmmwv.SetTireStepSize(tire_step_size)                                  # tire model substep
hmmwv.Initialize()                                                     # build the vehicle

hmmwv.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)    # chassis as primitives
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES) # suspension primitives
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)   # steering primitives
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)            # wheels as mesh
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)             # tires as mesh

system = hmmwv.GetSystem()                                             # the wrapper-owned ChSystem
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)    # REQUIRED before building SCMTerrain
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())                  # report total vehicle mass

terrain = veh.SCMTerrain(system)                                       # deformable Bekker-Wong soil
terrain.SetSoilParameters(
    2e6,    # Bekker_Kphi    — frictional modulus (Pa)
    0,      # Bekker_Kc      — cohesive modulus
    1.1,    # Bekker_n       — exponent
    0,      # Mohr_cohesion  — cohesive limit (Pa)
    30,     # Mohr_friction  — friction angle (deg)
    0.01,   # Janosi_shear   — shear coefficient (m)
    2e8,    # elastic_K      — elastic stiffness (Pa/m)
    3e4,    # damping_R      — vertical damping (Pa·s/m)
)
terrain.AddMovingPatch(                                                # follow only the cells near the vehicle
    hmmwv.GetChassisBody(),                                            # attach to chassis (NOT spindles)
    chrono.ChVector3d(0, 0, 0),                                        # local OOBB centre
    chrono.ChVector3d(5, 3, 1),                                        # OOBB dims (m)
)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)               # false-color sinkage heatmap
terrain.Initialize(veh.GetDataFile("terrain/height_maps/bump64.bmp"),  # heightmap-initialized hill terrain
                   40, 40, -1, 1, 0.02)                                # length, width, hMin, hMax, mesh res
terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)  # dirt texture, UV tiling

tire_rad = hmmwv.GetVehicle().GetAxles()[0].m_wheels[0].GetTire().GetRadius()  # tire radius for cylinders
tire_w = hmmwv.GetVehicle().GetAxles()[0].m_wheels[0].GetTire().GetWidth()     # tire width for cylinders
tire_mat = chrono.ChContactMaterialSMC()                               # SMC contact material for tires
tire_mat.SetFriction(0.9)                                              # tire friction
tire_mat.SetRestitution(0.1)                                           # tire restitution
TIRE_FAMILY = 1                                                        # collision family for tires
for axle in hmmwv.GetVehicle().GetAxles():                             # add a collision cylinder per wheel
    for iw in range(2):
        spindle = axle.m_wheels[iw].GetSpindle()                       # spindle body
        spindle.AddCollisionShape(
            chrono.ChCollisionShapeCylinder(tire_mat, tire_rad + 0.04, tire_w),  # +0.04 m so SCM detects sinkage
            chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(math.pi / 2)),   # roll axis along spindle
        )
        spindle.EnableCollision(True)                                  # enable spindle collision
        sp_cm = spindle.GetCollisionModel()                            # collision model
        sp_cm.SetFamily(TIRE_FAMILY)                                   # tag as tire family
        sp_cm.DisallowCollisionsWith(TIRE_FAMILY)                      # tires don't collide each other
system.GetCollisionSystem().BindAll()                                  # rebuild collision models after shape edits

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                       # vehicle-specific Irrlicht window
vis.SetWindowTitle("SCM Hill Demo")                                    # window title
vis.SetWindowSize(1280, 1024)                                          # window size (px)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.71), 6.0, 0.5)        # chase camera track point/dist/height
vis.Initialize()                                                       # build the device FIRST
vis.AddLogo()                                                          # pychrono logo
vis.AddLightDirectional()                                             # single directional light (vehicle style)
vis.AddSkyBox()                                                        # sky box
vis.AttachVehicle(hmmwv.GetVehicle())                                  # bind the vehicle to the visual system

driver = veh.ChInteractiveDriverIRR(vis)                               # interactive keyboard driver
render_step_size = 1.0 / 20.0                                          # render cadence for the hill demo
driver.SetSteeringDelta(render_step_size / 1.0)                        # steering ramp rate
driver.SetThrottleDelta(render_step_size / 1.0)                        # throttle ramp rate
driver.SetBrakingDelta(render_step_size / 0.3)                         # braking ramp rate
driver.Initialize()                                                    # build the driver

render_steps = math.ceil(render_step_size / step_size)                # physics steps between rendered frames
realtime_timer = chrono.ChRealtimeStepTimer()                          # real-time pacing
step_number = 0                                                        # physics step counter

while vis.Run():                                                       # main real-time loop
    time = hmmwv.GetSystem().GetChTime()                              # current sim time

    if step_number % render_steps == 0:                              # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                               # current driver commands

    driver.Synchronize(time)                                          # advance driver state
    terrain.Synchronize(time)                                         # advance terrain state
    hmmwv.Synchronize(time, driver_inputs, terrain)                  # feed driver + terrain to the vehicle
    vis.Synchronize(time, driver_inputs)                             # update the visual system

    driver.Advance(step_size)                                        # step driver
    terrain.Advance(step_size)                                       # step terrain (deforms soil)
    hmmwv.Advance(step_size)                                         # step the wrapper-owned system
    vis.Advance(step_size)                                           # step the visual system

    step_number += 1                                                 # next step
    realtime_timer.Spin(step_size)                                   # pace to wall clock
