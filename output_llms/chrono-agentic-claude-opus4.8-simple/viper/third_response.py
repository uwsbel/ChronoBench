import os
import pychrono as chrono
import pychrono.robot as robot
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

system = chrono.ChSystemNSC()                                        # NSC system for the rover
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # required BEFORE building SCMTerrain
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # Z-up gravity
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)          # contact envelope
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)            # contact margin

rover = robot.Viper(system)                                          # built-in Viper rover (owns its bodies)
driver = robot.ViperDCMotorControl()                                 # DC-motor steering driver
rover.SetDriver(driver)                                              # MUST precede Initialize
init_pos = chrono.ChVector3d(0, 0.2, 0)                              # spawn just above terrain
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                          # identity orientation (w,x,y,z)
rover.Initialize(chrono.ChFramed(init_pos, init_rot))                # place rover via a ChFramed

terrain = veh.SCMTerrain(system)                                     # deformable Bekker-Wong soft soil
terrain.SetReferenceFrame(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, -0.5),
                                             chrono.QUNIT))          # terrain top plane at z=-0.5
terrain.SetSoilParameters(
    0.2e6,   # Bekker_Kphi   — frictional modulus (Pa)
    0,       # Bekker_Kc     — cohesive modulus
    1.1,     # Bekker_n      — exponent
    0,       # Mohr_cohesion — cohesive limit (Pa)
    30,      # Mohr_friction — friction angle (deg)
    0.01,    # Janosi_shear  — shear coefficient (m)
    4e7,     # elastic_K     — elastic stiffness (Pa/m)
    3e4,     # damping_R     — vertical damping (Pa.s/m)
)
terrain.SetPlotType(veh.SCMTerrain.PLOT_PRESSURE, 0, 20000)          # pressure heatmap overlay
terrain.Initialize(14, 4, 0.02)                                      # length, width, mesh resolution (m)

vis = chronoirr.ChVisualSystemIrrlicht()                             # real-time Irrlicht window
vis.AttachSystem(system)                                             # bind the system
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)                    # Z-up camera convention
vis.SetWindowSize(1280, 720)                                         # window resolution
vis.SetWindowTitle("Viper rover - SCM terrain")                      # window title
vis.Initialize()                                                     # Irrlicht init FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))     # logo
vis.AddSkyBox()                                                      # sky box
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5), chrono.ChVector3d(0, 0, 1))  # eye, target
vis.AddTypicalLights()                                               # default lights
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5),
                       3, 4, 10, 40, 512)                            # pos, aim, radius, near, far, angle, res

time_step = 1e-3                                                     # integration step
render_fps = 50.0                                                    # review video frame rate
render_every = max(1, round(1.0 / (render_fps * time_step)))         # render-cadence (untagged)
while vis.Run():                                                     # plain real-time loop
    driver.SetSteering(0.0)                                          # drive straight (constant steering)
    rover.Update()                                                   # propagate command into the motors
    terrain.Synchronize(system.GetChTime())                          # update terrain at the current time
    vis.BeginScene(); vis.Render(); vis.EndScene()                   # draw the frame
    system.DoStepDynamics(time_step)                                 # advance one physics step
    terrain.Advance(time_step)                                       # advance the deformable terrain
