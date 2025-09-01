#
# Viper rover on a rigid terrain – steering ramp example
#
# ---------------------------------------------------------------------
# Prerequisites
#   pip install pychrono
#   (or build Chrono from source with the VEHICLE and IRRLICHT modules)
# ---------------------------------------------------------------------

import math
import pychrono             as chrono
import pychrono.vehicle      as veh
import pychrono.irrlicht     as irr

# 1. -------------------------------------------------------------
#    Basic Chrono initialisation
# ----------------------------------------------------------------
chrono.SetChronoDataPath(chrono.GetChronoDataPath())          # meshes, textures, etc.

system      = chrono.ChSystemSMC()                            # SMC = non-smooth compliant
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))              # Earth gravity


# 2. -------------------------------------------------------------
#    Terrain – a single rigid patch
# ----------------------------------------------------------------
terrain = veh.RigidTerrain(system)

patch_mat = chrono.ChMaterialSurfaceSMC()
patch_mat.SetFriction(0.9)            # friction coefficient
patch_mat.SetRestitution(0.01)
patch_mat.SetYoungModulus(2e7)

patch = terrain.AddPatch(
            patch_mat,
            chrono.ChVectorD(0, 0, 0),                       # location
            chrono.ChVectorD(0, 0, 1),                       # outward normal
            60, 60)                                          # size X, Y  [m]
patch.SetTexture(chrono.GetChronoDataFile("terrain/textures/grass.jpg"), 200, 200)
terrain.Initialize()


# 3. -------------------------------------------------------------
#    The rover
# ----------------------------------------------------------------
init_loc  = chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.4))   # 40 cm above ground
rover     = veh.Viper(system)
rover.Initialize(init_loc)

# Make sure we start with motors enabled
powertrain_speed = 2.0  # [rad/s] nominal wheel rotation speed


# 4. -------------------------------------------------------------
#    A very small “driver”: we are going to change steering
#    over time by scaling left/right wheel speeds
# ----------------------------------------------------------------
class RampSteeringDriver:
    """
    Simple driver that produces a *skid-steer* behaviour by
    scaling left vs. right wheel rotational speeds.

    The steering value goes from 0 (straight) to 'target_steer'
    in a time 'ramp_time'.  After that it stays constant.
    """
    def __init__(self, rover, base_speed, target_steer=0.8, ramp_time=5.0):
        self.rover         = rover
        self.base_speed    = base_speed
        self.target_steer  = max(0.0, min(abs(target_steer), 1.0))  # 0 … 1
        self.ramp_time     = max(ramp_time, 1e-6)
        self.start_time    = 0.0

    def initialize(self, time):
        self.start_time = time

    def synchronize(self, time):
        # how far are we into the steering ramp?
        dt   = time - self.start_time
        frac = min(dt / self.ramp_time, 1.0)
        s    = frac * self.target_steer      # current steering demand (0 … target)
        # simple skid steering: scale wheel speeds
        left_speed  =  self.base_speed * (1.0 + s)
        right_speed =  self.base_speed * (1.0 - s)
        self.rover.SetWheelMotorSpeed(left_speed,  veh.SIDE.LEFT,  powertrain=False)
        self.rover.SetWheelMotorSpeed(right_speed, veh.SIDE.RIGHT, powertrain=False)

driver = RampSteeringDriver(rover, base_speed=powertrain_speed,
                            target_steer=0.8, ramp_time=6.0)
driver.initialize(system.GetChTime())


# 5. -------------------------------------------------------------
#    Real-time visualisation with Irrlicht
# ----------------------------------------------------------------
application = irr.ChIrrApp(system,
                           "Viper rover – rigid terrain",
                           irr.dimension2du(1280, 720))

application.AddTypicalLogo()
application.AddSkyBox()
application.AddTypicalLights(chrono.ChVectorD(60, 60, 100),   # position
                             chrono.ChVectorD(0, 0, 0),       # aim-at
                             300)                             # radiance
# Camera tracking the rover chassis
application.AddChaseCamera(rover.GetChassisBody(),            # target
                           chrono.ChVectorD(0, 0, 1.0),       # relative position
                           5.0,                               # chase distance
                           0.5)                               # min distance
application.SetTimestep(1e-3)                                 # visualisation loop step
application.AssetBindAll()
application.AssetUpdateAll()


# 6. -------------------------------------------------------------
#    Main simulation loop
# ----------------------------------------------------------------
sim_step    = 1e-3                              # integrator step (system)
real_clock  = chrono.ChRealtimeStepTimer()
end_time    = 20.0                              # [s]

print("Starting simulation … (ESC to quit)")
while application.GetDevice().run():

    time = system.GetChTime()
    if time > end_time:
        break

    # Driver computations ----------------------------------------------------
    driver.synchronize(time)

    # Render -----------------------------------------------------------------
    application.BeginScene()
    application.DrawAll()
    application.EndScene()

    # Advance the simulation -------------------------------------------------
    system.DoStepDynamics(sim_step)
    real_clock.Spin(sim_step)                  # Let chrono try to keep real-time pace