import pychrono as chrono
import pychrono.irrlicht as chronoirr

def main():
    
    my_system = chrono.ChSystemNSC()
    my_system.Set_Gravity(chrono.ChVectorD(0, -9.81, 0))

    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('Epicyclic Gears Simulation')
    vis.SetSymbolscale(0.01)
    vis.SetLightIntensity(0.8)
    vis.SetLightPosition(chrono.ChVectorD(2, 2, 2))
    vis.SetCameraPosition(chrono.ChVectorD(1.5, 1.5, 1.5))
    vis.SetCameraLookatPoint(chrono.ChVectorD(0, 0, 0))
    my_system.SetVisualSystem(vis)

    
    truss = chrono.ChBodyEasyBox(1.0, 0.2, 0.2, 1000, True, True)
    truss.SetPos(chrono.ChVectorD(0, 0, 0))
    truss.SetBodyFixed(True)
    truss.GetVisualShape(0).SetColor(chrono.ChColor(0.4, 0.4, 0.4))
    truss.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
    my_system.Add(truss)

    
    bar_length = 0.3
    bar = chrono.ChBodyEasyBox(bar_length, 0.05, 0.05, 1000, True, True)
    bar.SetPos(chrono.ChVectorD(0.15, 0, 0))  
    bar.SetMaterialSurface(chrono.ChMaterialSurface())
    bar.GetVisualShape(0).SetColor(chrono.ChColor(0.8, 0.8, 0.8))
    my_system.Add(bar)

    
    rev_joint = chrono.ChLinkRevolute()
    rev_joint.Initialize(truss, bar,
                        chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
                        chrono.ChFrameD(chrono.ChVectorD(-0.15, 0, 0), chrono.QUNIT))
    my_system.AddLink(rev_joint)

    
    rev_motor = rev_joint.GetMotor()
    rev_motor.SetVelocity(10)  
    rev_motor.SetTorqueMax(1e6)
    rev_motor.Enable(True)

    
    sun_radius = 0.2
    sun_gear = chrono.ChBodyEasyCylinder(sun_radius, 0.02, 1000, True, True)
    sun_gear.SetPos(chrono.ChVectorD(0, 0, 0))
    sun_gear.SetBodyFixed(True)
    sun_gear.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.2, 0.8))
    my_system.Add(sun_gear)

    
    planet_radius = 0.1
    planet_gear = chrono.ChBodyEasyCylinder(planet_radius, 0.02, 1000, True, True)
    planet_gear.SetPos(chrono.ChVectorD(0.3, 0, 0))
    planet_gear.GetVisualShape(0).SetColor(chrono.ChColor(0.8, 0.2, 0.2))
    my_system.Add(planet_gear)

    
    planet_rev_joint = chrono.ChLinkRevolute()
    planet_rev_joint.Initialize(bar, planet_gear,
                               chrono.ChFrameD(chrono.ChVectorD(0.3, 0, 0), chrono.QUNIT),
                               chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
    my_system.AddLink(planet_rev_joint)

    
    gear_ratio = -planet_radius / sun_radius  
    gear_link = chrono.ChLinkGear()
    gear_link.Initialize(bar, planet_gear,
                        chrono.ChFrameD(chrono.ChVectorD(0.3, 0, 0), chrono.QUNIT),
                        chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
    gear_link.SetGearRatio(gear_ratio)
    my_system.Add(gear_link)

    
    vis.AddAllVisualizationAssets()

    
    application = chronoirr.ChIrrApp(my_system, 'Epicyclic Gears', chronoirr.dimension2du(1024, 768))
    application.SetCamera(chronoirr.ChVectorD(1.5, 1.5, 1.5))
    application.AddAllTypicalLights()
    application.AddAllVisualization()
    application.AssetBind()
    application.AssetUpdate()
    application.SetTimestep(0.001)
    application.SetTryRealtime(True)

    while application.GetDevice().run():
        application.BeginScene(True, True, chrono.ChColor(0.8, 0.8, 0.8))
        application.DrawAll()
        application.DoStep()
        application.EndScene()

if __name__ == '__main__':
    main()

import pychrono as chrono
import pychrono.irrlicht as chronoirr

def main():
    
    my_system = chrono.ChSystemNSC()
    my_system.Set_Gravity(chrono.ChVectorD(0, -9.81, 0))

    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('Epicyclic Gears Simulation')
    vis.SetLightIntensity(0.8)
    vis.SetLightPosition(chrono.ChVectorD(2, 2, 2))
    vis.SetCameraPosition(chrono.ChVectorD(1.5, 1.5, 1.5))
    vis.SetCameraLookatPoint(chrono.ChVectorD(0, 0, 0))
    my_system.SetVisualSystem(vis)

    
    truss = chrono.ChBodyEasyBox(1.0, 0.2, 0.2, 1000, True, True)
    truss.SetPos(chrono.ChVectorD(0, 0, 0))
    truss.SetBodyFixed(True)
    truss.GetVisualShape(0).SetColor(chrono.ChColor(0.4, 0.4, 0.4))
    truss.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
    my_system.Add(truss)

    
    bar_length = 0.3
    bar = chrono.ChBodyEasyBox(bar_length, 0.05, 0.05, 1000, True, True)
    bar.SetPos(chrono.ChVectorD(0.15, 0, 0))  
    bar.SetMaterialSurface(chrono.ChMaterialSurface())
    bar.GetVisualShape(0).SetColor(chrono.ChColor(0.8, 0.8, 0.8))
    my_system.Add(bar)

    
    rev_joint = chrono.ChLinkRevolute()
    rev_joint.Initialize(truss, bar,
                        chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
                        chrono.ChFrameD(chrono.ChVectorD(-0.15, 0, 0), chrono.QUNIT))
    my_system.AddLink(rev_joint)

    
    rev_motor = rev_joint.GetMotor()
    rev_motor.SetVelocity(10)  
    rev_motor.SetTorqueMax(1e6)
    rev_motor.Enable(True)

    
    sun_radius = 0.2
    sun_gear = chrono.ChBodyEasyCylinder(sun_radius, 0.02, 1000, True, True)
    sun_gear.SetPos(chrono.ChVectorD(0, 0, 0))
    sun_gear.SetBodyFixed(True)
    sun_gear.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.2, 0.8))
    my_system.Add(sun_gear)

    
    planet_radius = 0.1
    planet_gear = chrono.ChBodyEasyCylinder(planet_radius, 0.02, 1000, True, True)
    planet_gear.SetPos(chrono.ChVectorD(0.3, 0, 0))
    planet_gear.GetVisualShape(0).SetColor(chrono.ChColor(0.8, 0.2, 0.2))
    my_system.Add(planet_gear)

    
    planet_rev_joint = chrono.ChLinkRevolute()
    planet_rev_joint.Initialize(bar, planet_gear,
                               chrono.ChFrameD(chrono.ChVectorD(0.3, 0, 0), chrono.QUNIT),
                               chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
    my_system.AddLink(planet_rev_joint)

    
    gear_ratio = -planet_radius / sun_radius  
    gear_link = chrono.ChLinkGear()
    gear_link.Initialize(bar, planet_gear,
                        chrono.ChFrameD(chrono.ChVectorD(0.3, 0, 0), chrono.QUNIT),
                        chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
    gear_link.SetGearRatio(gear_ratio)
    my_system.Add(gear_link)

    
    application = chronoirr.ChIrrApp(my_system, 'Epicyclic Gears', chronoirr.dimension2du(1024, 768))
    application.SetCamera(chronoirr.ChVectorD(1.5, 1.5, 1.5))
    application.AddAllTypicalLights()
    application.AddAllVisualization()
    application.AssetBind()
    application.AssetUpdate()
    application.SetTimestep(0.001)
    application.SetTryRealtime(True)

    
    while application.GetDevice().run():
        application.BeginScene(True, True, chrono.ChColor(0.8, 0.8, 0.8))
        application.DrawAll()
        application.DoStep()
        application.EndScene()

if __name__ == '__main__':
    main()