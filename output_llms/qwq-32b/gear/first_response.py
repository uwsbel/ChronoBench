import pychrono as chrono
import pychrono.irrlicht as chronoirr

def main():
    # Create the Chrono system
    my_system = chrono.ChSystemNSC()
    my_system.Set_Gravity(chrono.ChVectorD(0, -9.81, 0))

    # Visualization setup
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('Epicyclic Gears Simulation')
    vis.SetSymbolscale(0.01)
    vis.SetLightIntensity(0.8)
    vis.SetLightPosition(chrono.ChVectorD(2, 2, 2))
    vis.SetCameraPosition(chrono.ChVectorD(1.5, 1.5, 1.5))
    vis.SetCameraLookatPoint(chrono.ChVectorD(0, 0, 0))
    my_system.SetVisualSystem(vis)

    # Create fixed truss (base plate)
    truss = chrono.ChBodyEasyBox(1.0, 0.2, 0.2, 1000, True, True)
    truss.SetPos(chrono.ChVectorD(0, 0, 0))
    truss.SetBodyFixed(True)
    truss.GetVisualShape(0).SetColor(chrono.ChColor(0.4, 0.4, 0.4))
    truss.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
    my_system.Add(truss)

    # Create rotating bar (carrier)
    bar_length = 0.3
    bar = chrono.ChBodyEasyBox(bar_length, 0.05, 0.05, 1000, True, True)
    bar.SetPos(chrono.ChVectorD(0.15, 0, 0))  # Center at (0.15,0,0)
    bar.SetMaterialSurface(chrono.ChMaterialSurface())
    bar.GetVisualShape(0).SetColor(chrono.ChColor(0.8, 0.8, 0.8))
    my_system.Add(bar)

    # Revolute joint between truss and bar (at origin)
    rev_joint = chrono.ChLinkRevolute()
    rev_joint.Initialize(truss, bar,
                        chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
                        chrono.ChFrameD(chrono.ChVectorD(-0.15, 0, 0), chrono.QUNIT))
    my_system.AddLink(rev_joint)

    # Enable motor on the joint to enforce rotation
    rev_motor = rev_joint.GetMotor()
    rev_motor.SetVelocity(10)  # 10 rad/s
    rev_motor.SetTorqueMax(1e6)
    rev_motor.Enable(True)

    # Create sun gear (fixed to truss)
    sun_radius = 0.2
    sun_gear = chrono.ChBodyEasyCylinder(sun_radius, 0.02, 1000, True, True)
    sun_gear.SetPos(chrono.ChVectorD(0, 0, 0))
    sun_gear.SetBodyFixed(True)
    sun_gear.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.2, 0.8))
    my_system.Add(sun_gear)

    # Create planet gear (attached to bar's end)
    planet_radius = 0.1
    planet_gear = chrono.ChBodyEasyCylinder(planet_radius, 0.02, 1000, True, True)
    planet_gear.SetPos(chrono.ChVectorD(0.3, 0, 0))
    planet_gear.GetVisualShape(0).SetColor(chrono.ChColor(0.8, 0.2, 0.2))
    my_system.Add(planet_gear)

    # Revolute joint between bar and planet gear (allowing rotation around Z)
    planet_rev_joint = chrono.ChLinkRevolute()
    planet_rev_joint.Initialize(bar, planet_gear,
                               chrono.ChFrameD(chrono.ChVectorD(0.3, 0, 0), chrono.QUNIT),
                               chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
    my_system.AddLink(planet_rev_joint)

    # Gear constraint between bar and planet gear
    gear_ratio = -planet_radius / sun_radius  # -0.5
    gear_link = chrono.ChLinkGear()
    gear_link.Initialize(bar, planet_gear,
                        chrono.ChFrameD(chrono.ChVectorD(0.3, 0, 0), chrono.QUNIT),
                        chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
    gear_link.SetGearRatio(gear_ratio)
    my_system.Add(gear_link)

    # Visualization setup
    vis.AddAllVisualizationAssets()

    # Simulation loop
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
    # Initialize Chrono system
    my_system = chrono.ChSystemNSC()
    my_system.Set_Gravity(chrono.ChVectorD(0, -9.81, 0))

    # Visualization setup
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('Epicyclic Gears Simulation')
    vis.SetLightIntensity(0.8)
    vis.SetLightPosition(chrono.ChVectorD(2, 2, 2))
    vis.SetCameraPosition(chrono.ChVectorD(1.5, 1.5, 1.5))
    vis.SetCameraLookatPoint(chrono.ChVectorD(0, 0, 0))
    my_system.SetVisualSystem(vis)

    # Create fixed truss (base plate)
    truss = chrono.ChBodyEasyBox(1.0, 0.2, 0.2, 1000, True, True)
    truss.SetPos(chrono.ChVectorD(0, 0, 0))
    truss.SetBodyFixed(True)
    truss.GetVisualShape(0).SetColor(chrono.ChColor(0.4, 0.4, 0.4))
    truss.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
    my_system.Add(truss)

    # Create rotating bar (carrier)
    bar_length = 0.3
    bar = chrono.ChBodyEasyBox(bar_length, 0.05, 0.05, 1000, True, True)
    bar.SetPos(chrono.ChVectorD(0.15, 0, 0))  # Center at (0.15,0,0)
    bar.SetMaterialSurface(chrono.ChMaterialSurface())
    bar.GetVisualShape(0).SetColor(chrono.ChColor(0.8, 0.8, 0.8))
    my_system.Add(bar)

    # Revolute joint between truss and bar (at origin)
    rev_joint = chrono.ChLinkRevolute()
    rev_joint.Initialize(truss, bar,
                        chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
                        chrono.ChFrameD(chrono.ChVectorD(-0.15, 0, 0), chrono.QUNIT))
    my_system.AddLink(rev_joint)

    # Enable motor to enforce rotation
    rev_motor = rev_joint.GetMotor()
    rev_motor.SetVelocity(10)  # 10 rad/s
    rev_motor.SetTorqueMax(1e6)
    rev_motor.Enable(True)

    # Create sun gear (fixed to truss)
    sun_radius = 0.2
    sun_gear = chrono.ChBodyEasyCylinder(sun_radius, 0.02, 1000, True, True)
    sun_gear.SetPos(chrono.ChVectorD(0, 0, 0))
    sun_gear.SetBodyFixed(True)
    sun_gear.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.2, 0.8))
    my_system.Add(sun_gear)

    # Create planet gear (attached to bar's end)
    planet_radius = 0.1
    planet_gear = chrono.ChBodyEasyCylinder(planet_radius, 0.02, 1000, True, True)
    planet_gear.SetPos(chrono.ChVectorD(0.3, 0, 0))
    planet_gear.GetVisualShape(0).SetColor(chrono.ChColor(0.8, 0.2, 0.2))
    my_system.Add(planet_gear)

    # Revolute joint between bar and planet gear (rotation around Z-axis)
    planet_rev_joint = chrono.ChLinkRevolute()
    planet_rev_joint.Initialize(bar, planet_gear,
                               chrono.ChFrameD(chrono.ChVectorD(0.3, 0, 0), chrono.QUNIT),
                               chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
    my_system.AddLink(planet_rev_joint)

    # Gear constraint between bar and planet gear
    gear_ratio = -planet_radius / sun_radius  # -0.5
    gear_link = chrono.ChLinkGear()
    gear_link.Initialize(bar, planet_gear,
                        chrono.ChFrameD(chrono.ChVectorD(0.3, 0, 0), chrono.QUNIT),
                        chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
    gear_link.SetGearRatio(gear_ratio)
    my_system.Add(gear_link)

    # Initialize Irrlicht application
    application = chronoirr.ChIrrApp(my_system, 'Epicyclic Gears', chronoirr.dimension2du(1024, 768))
    application.SetCamera(chronoirr.ChVectorD(1.5, 1.5, 1.5))
    application.AddAllTypicalLights()
    application.AddAllVisualization()
    application.AssetBind()
    application.AssetUpdate()
    application.SetTimestep(0.001)
    application.SetTryRealtime(True)

    # Simulation loop
    while application.GetDevice().run():
        application.BeginScene(True, True, chrono.ChColor(0.8, 0.8, 0.8))
        application.DrawAll()
        application.DoStep()
        application.EndScene()

if __name__ == '__main__':
    main()