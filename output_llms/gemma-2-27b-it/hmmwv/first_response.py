import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np

# ---------------------------------------------------------------------
#
#  Create the simulation system
#
# ---------------------------------------------------------------------

mysystem = chrono.ChSystemNSC()

# Set gravitational acceleration
mysystem.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# ---------------------------------------------------------------------
#
# Create the terrain
#
# ---------------------------------------------------------------------

terrain = chrono.ChRigidTerrain(mysystem, True, True)
patch_size = 100
terrain.AddHeightPatch(chrono.ChVectorD(-patch_size, -patch_size, 0),
                       chrono.ChVectorD(patch_size, -patch_size, 0),
                       chrono.ChVectorD(patch_size, patch_size, 0),
                       chrono.ChVectorD(-patch_size, patch_size, 0),
                       chrono.ChVectorD(0, 0, 0))
terrain.SetTexture(chrono.GetChronoDataPath() + 'terrain/textures/grass.png', 200, 200)

# ---------------------------------------------------------------------
#
# Create the HMMWV vehicle
#
# ---------------------------------------------------------------------

vehicle = chrono.ChVehicleModel()
vehicle.SetSystem(mysystem)

# Chassis
chassis = chrono.ChBodyEasyBox(2.5, 1.2, 0.8, 1000)
chassis.SetPos(chrono.ChVectorD(0, 1.2, 0))
vehicle.AddChassis(chassis)

# Suspension
susp_pos = [chrono.ChVectorD(-1.2, -0.6, 0.5), chrono.ChVectorD(1.2, -0.6, 0.5),
            chrono.ChVectorD(-1.2, -0.6, -0.5), chrono.ChVectorD(1.2, -0.6, -0.5)]
for pos in susp_pos:
    susp = chrono.ChDoubleWishbone(vehicle.GetChassisBody(), pos, chrono.ChVectorD(0, -1, 0),
                                   chrono.ChVectorD(0, 1, 0), chrono.ChVectorD(1, 0, 0),
                                   chrono.ChVectorD(0, 0, 1), 100, 50, 0.2)
    vehicle.AddSuspension(susp)

# Wheels
wheel_pos = [chrono.ChVectorD(-1.2, -0.6, 0.8), chrono.ChVectorD(1.2, -0.6, 0.8),
             chrono.ChVectorD(-1.2, -0.6, -0.8), chrono.ChVectorD(1.2, -0.6, -0.8)]
for pos in wheel_pos:
    wheel = chrono.ChWheel(vehicle.GetChassisBody(), chrono.ChVectorD(0, -1, 0),
                          chrono.ChVectorD(0, 1, 0), chrono.ChVectorD(1, 0, 0),
                          chrono.ChVectorD(0, 0, 1), pos, 0.3, 0.1, 100)
    wheel.SetTMEasyTire(chrono.ChTMEasyTire(0.8, 0.8, 0.5, 0.5, 0.2, 0.1))
    vehicle.AddWheel(wheel)

# ---------------------------------------------------------------------
#
# Set up the driver system
#
# ---------------------------------------------------------------------

driver = chrono.ChDriverSteering(vehicle)

# ---------------------------------------------------------------------
#
# Create the Irrlicht visualization system
#
# ---------------------------------------------------------------------

vis = chronoirr.ChIrrApp(mysystem, "HMMWV Simulation", chronoirr.dimension2du(1280, 720))
vis.AddTypicalLights()
vis.AddSkyBox()
vis.SetCameraPosition(chrono.ChVectorD(0, 5, -10))
vis.SetCameraLookAt(chrono.ChVectorD(0, 1.2, 0))

# ---------------------------------------------------------------------
#
# Simulation loop
#
# ---------------------------------------------------------------------

while vis.GetDevice().run():
    vis.BeginScene()
    vis.Render()
    mysystem.DoStepDynamics(0.02)

    # Handle driver input
    driver.SetSteeringAngle(vis.GetDevice().GetKeyboard()->GetDirKeyState(chrono.ChIrrApp::KEY_LEFT) -
                           vis.GetDevice().GetKeyboard()->GetDirKeyState(chrono.ChIrrApp::KEY_RIGHT))
    driver.SetThrottle(vis.GetDevice().GetKeyboard()->GetDirKeyState(chrono.ChIrrApp::KEY_UP) -
                      vis.GetDevice().GetKeyboard()->GetDirKeyState(chrono.ChIrrApp::KEY_DOWN))
    driver.SetBraking(vis.GetDevice().GetKeyboard()->GetDirKeyState(chrono.ChIrrApp::KEY_SPACE))

    vis.EndScene()