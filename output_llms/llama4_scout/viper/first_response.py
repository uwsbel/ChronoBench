import chrono as ch
import chrono.irrlicht as irr

# 1. Initialize the PyChrono environment and core components
ch.CH_Environment().Set_G_acc(ch.CH_CoordSys(0, 0, -9.81))

# 2. Create the Chrono system with gravity
sys = ch.ChSystem()

# Add ground body
ground = ch.ChBodyEasyTerrain(
    100, 100,  10, 10, 
    ch.GetChronoDataFile("terrain/terrain_heightmap.png"), 
    ch.GetChronoDataFile("terrain/terrain_color.jpg"), 
    False, False, True
)
ground.SetPos(ch.CH_CoordSys(0, 0, -1))
sys.Add(ground)

# 3. Initialize the Viper rover
viper = ch.ChVehicle(ch.GetChronoDataFile("vehicles/viper/Viper.json"))
sys.Add(viper.GetChassisBody())
for wheel in viper.GetWheels():
    sys.Add(wheel)
sys.Add(viper.GetSteering())

# 4. Initialize the driver
driver = ch.ChIrrGuiDriver(viper)
driver.SetSteeringDelta(0.1, 1.0)  # steering change over 1 second

# 5. Create a real-time visualization using Irrlicht
app = irr.ChIrrApp(sys, "Viper Rover Simulation", 800, 600)
app.SetCameraPos(ch.CH_CoordSys(0, -5, 1))
app.SetCameraTarget(ch.CH_CoordSys(0, 0, 0))
app.EnableCollisionVisualization(True)
app.EnableTimestepControl(True)

# Set lighting
app.AddLight(ch.CH_CoordSys(-5, -5, 5), ch.CH_CoordSys(10, 10, 10), ch.color(1, 1, 1))

# Simulation loop
while app.Run():
    # Update the system
    sys.Update()

    # Gradually change steering over 10 seconds
    if app.GetTimestep() < 10:
        driver.SetSteering(0.1 * app.GetTimestep())

    # Render the scene
    app.Render()
    app.Screenshot("viper_rover_sim.png")