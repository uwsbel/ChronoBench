import chrono as ch
import chrono.irrlicht as irr
import chrono.vehicle as veh


ch.set_default_data_file_path("./data")


system = ch.ChSystemSMC()


vehicle = veh.ChHMMWV(system)


vehicle.SetChassisPos({0, 0, 1})


terrain = veh.ChTerrainSCM(system)


soil = terrain.GetSoilModel()
soil.SetType(veh.ChSoilModel.Type.ROTHBERG)
soil.SetParameters(100, 0.3, 1e6, 1e4)


heightMap = []
for i in range(20):
    row = []
    for j in range(20):
        row.append(0.1 * math.sin(i / 5.0) * math.cos(j / 5.0))
    heightMap.append(row)
terrain.Initialize(heightMap)


driver = veh.ChDriver(system, vehicle.GetChassisBody(), vehicle.GetEngine(), vehicle.GetTransmission())


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVerticalAxis(-ch.Vector(0, 0, 1))
vis.SetCameraPosition(ch.Vector(0, -10, 5))
vis.SetLightDirection(ch.Vector(1, 1, 1))


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(1 / 60)

    
    driver.Synchronize(vehicle.GetChassisBody().GetPos(), vehicle.GetChassisBody().GetRot())
    terrain.Synchronize(vehicle.GetChassisBody().GetPos(), vehicle.GetChassisBody().GetRot())
    vis.Synchronize(vehicle.GetChassisBody().GetPos(), vehicle.GetChassisBody().GetRot())