import chrono as ch
import chrono.irrlicht as irr


ch.ChEngine.Initialize()


app = irr.ChIrrApp()


app.SetTitle("Curiosity Rover on Rigid Terrain")


terrain = ch.ChBodyEasyTerrain(500, 500, 20, 20, "terrain/terrain.obj")
terrain.SetPos(ch.ChVector3d(0, 0, -5))
terrain.SetBodyFixed(True)


terrain_collision = ch.ChCollisionShape(terrain, False)
terrain_collision.SetShapeType(ch.ChCollisionShape.ELLIPSOID)
terrain_collision.SetDimensions(ch.ChVector3d(250, 250, 5))


ch.ChSystemGlobals.GetActiveSystem().Add(terrain)
ch.ChSystemGlobals.GetActiveSystem().Add(terrain_collision)


rover = ch.ChBodyEasyMesh("models/curiosity.obj", 1000)


rover.SetPos(ch.ChVector3d(0, 0, 0.5))


ch.ChSystemGlobals.GetActiveSystem().Add(rover)


motor_driver = ch.ChMotorControlDriver(rover)


motor_driver.SetTorqueLimit(100)
motor_driver.SetSpeedLimit(10)


ch.ChSystemGlobals.GetActiveSystem().Add(motor_driver)


ch.ChSystemGlobals.GetActiveSystem().SetCollisionSystem(ch.ChCollisionSystem.Type.BULLET)
ch.ChSystemGlobals.GetActiveSystem().SetContactMaterial(ch.ChContactMaterial.Type.ANISOTROPIC_ROUGH)


app.SetCameraDistance(10)
app.SetCameraPos(ch.ChVector3d(0, -10, 5))
app.SetLightDirection(ch.ChVector3d(1, 1, 1))
app.EnableShadows(True)


app.Run()

import chrono as ch
import chrono.irrlicht as irr

def main():
    
    ch.ChEngine.Initialize()

    
    app = irr.ChIrrApp()

    
    app.SetTitle("Curiosity Rover on Rigid Terrain")

    
    terrain = ch.ChBodyEasyTerrain(500, 500, 20, 20, "terrain/terrain.obj")
    terrain.SetPos(ch.ChVector3d(0, 0, -5))
    terrain.SetBodyFixed(True)

    
    terrain_collision = ch.ChCollisionShape(terrain, False)
    terrain_collision.SetShapeType(ch.ChCollisionShape.ELLIPSOID)
    terrain_collision.SetDimensions(ch.ChVector3d(250, 250, 5))

    
    ch.ChSystemGlobals.GetActiveSystem().Add(terrain)
    ch.ChSystemGlobals.GetActiveSystem().Add(terrain_collision)

    
    rover = ch.ChBodyEasyMesh("models/curiosity.obj", 1000)

    
    rover.SetPos(ch.ChVector3d(0, 0, 0.5))

    
    ch.ChSystemGlobals.GetActiveSystem().Add(rover)

    
    motor_driver = ch.ChMotorControlDriver(rover)

    
    motor_driver.SetTorqueLimit(100)
    motor_driver.SetSpeedLimit(10)

    
    ch.ChSystemGlobals.GetActiveSystem().Add(motor_driver)

    
    ch.ChSystemGlobals.GetActiveSystem().SetCollisionSystem(ch.ChCollisionSystem.Type.BULLET)
    ch.ChSystemGlobals.GetActiveSystem().SetContactMaterial(ch.ChContactMaterial.Type.ANISOTROPIC_ROUGH)

    
    app.SetCameraDistance(10)
    app.SetCameraPos(ch.ChVector3d(0, -10, 5))
    app.SetLightDirection(ch.ChVector3d(1, 1, 1))
    app.EnableShadows(True)

    
    app.Run()

if __name__ == "__main__":
    main()