"""Viper rover on rigid terrain (PyChrono, ChSystemNSC + Bullet collision).

Models PyChrono's built-in six-wheel Viper rover driving on a flat rigid
ground box. The rover owns its chassis/wheels/suspension/motors; a DC-motor
control driver supplies the always-on drive while the steering angle is ramped
smoothly over a specified interval. Visualization is real-time Irrlicht (Z-up).

Expected behavior: the rover rolls forward under its DC drive; as the steering
command ramps from straight toward its limit and back, the rover curves to one
side and then straightens, all without penetrating the ground.
"""

import math

import pychrono as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr

# === Constants === geometry / timing / steering schedule
time_step = 1e-3
sim_end = 14.0
max_steering = math.pi / 6          # practical Viper steering limit (rad)
steer_start = 2.0                   # begin ramping steering in (s)
steer_mid = 7.0                     # fully turned-in / begin ramping out (s)
steer_end = 12.0                    # back to straight (s)
ground_z = -1.0                     # 1 m thick box -> top surface at z=-0.5
init_pos = chrono.ChVector3d(0, 0, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)   # identity (w,x,y,z)


# === System & gravity === NSC + Bullet collision for wheel/terrain contact
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# === Ground === fixed rigid terrain box, top surface at z=-0.5 under the spawn
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, ground_z))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# === Rover === built-in Viper with DC-motor steering driver (system-owned bodies)
rover = robot.Viper(system)
driver = robot.ViperDCMotorControl()
rover.SetDriver(driver)                          # SetDriver BEFORE Initialize
rover.Initialize(chrono.ChFramed(init_pos, init_rot))
chassis_body = rover.GetChassis().GetBody()      # cache: chassis handle reused for logging

# === Visualization === full Irrlicht scene (Initialize first, elements after)
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Viper rover - Rigid terrain")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5), chrono.ChVector3d(0, 0, 1))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5),
                       3, 4, 10, 40, 512)        # pos, aim, radius, near, far, angle, res

# === Main loop === ramp steering over time; rover drives forward under DC motors

try:
    while vis.Run() and system.GetChTime() < sim_end:
        t = system.GetChTime()

        # smooth steering schedule: hold straight, ramp in, then ramp out
        steering = 0.0
        if steer_start < t < steer_mid:
            steering = max_steering * (t - steer_start) / (steer_mid - steer_start)
        elif steer_mid <= t < steer_end:
            steering = max_steering * (steer_end - t) / (steer_end - steer_mid)
        driver.SetSteering(steering)

        rover.Update()        # propagate steering command into the rover motors

        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        system.DoStepDynamics(time_step)
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass

# === Post-processing === assemble review video + timeseries plot (review-only)
