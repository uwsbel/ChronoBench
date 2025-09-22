import chrono as ch
import chrono.irrlicht as irr


ch.CH_Environment().Set_G_acc(ch.CH_CoordSys(0, 0, -9.81))


sys = ch.ChSystem()


ground = ch.ChBodyEasyTerrain(
    100, 100,  10, 10, 
    ch.GetChronoDataFile("terrain/terrain_heightmap.png"), 
    ch.GetChronoDataFile("terrain/terrain_color.jpg"), 
    False, False, True
)
ground.SetPos(ch.CH_CoordSys(0, 0, -1))
sys.Add(ground)


viper = ch.ChVehicle(ch.GetChronoDataFile("vehicles/viper/Viper.json"))
sys.Add(viper.GetChassisBody())
for wheel in viper.GetWheels():
    sys.Add(wheel)
sys.Add(viper.GetSteering())


driver = ch.ChIrrGuiDriver(viper)
driver.SetSteeringDelta(0.1, 1.0)  


app = irr.ChIrrApp(sys, "Viper Rover Simulation", 800, 600)
app.SetCameraPos(ch.CH_CoordSys(0, -5, 1))
app.SetCameraTarget(ch.CH_CoordSys(0, 0, 0))
app.EnableCollisionVisualization(True)
app.EnableTimestepControl(True)


app.AddLight(ch.CH_CoordSys(-5, -5, 5), ch.CH_CoordSys(10, 10, 10), ch.color(1, 1, 1))


while app.Run():
    
    sys.Update()

    
    if app.GetTimestep() < 10:
        driver.SetSteering(0.1 * app.GetTimestep())

    
    app.Render()
    app.Screenshot("viper_rover_sim.png")