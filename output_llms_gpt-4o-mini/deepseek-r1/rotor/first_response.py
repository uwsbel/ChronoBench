import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr
from pychrono import ChVectorD, ChQuaternionD, QUNIT

def main():
    # 1. Initialize system
    system = chrono.ChSystemSMC()
    system.Set_G_acc(ChVectorD(0, 0, 0))  # Zero gravity

    # 2. Create IGA beam
    builder = fea.ChBuilderBeamIGA()
    
    # Beam material properties
    beam_material = fea.ChBeamSectionEulerAdvanced()
    radius = 0.05  # Beam radius (m)
    beam_material.SetDensity(7800)        # kg/m³
    beam_material.SetYoungModulus(2e11)   # Pa
    beam_material.SetGwithPoissonRatio(0.3)
    beam_material.SetCircularSection(radius)
    beam_material.SetRayleighDampingBeta(0.001)  # Damping

    # Build beam along X-axis
    length = 2.0  # Beam length (m)
    num_elements = 10
    start_point = ChVectorD(0, 0, 0)
    end_point = ChVectorD(length, 0, 0)
    builder.BuildBeam(system,                      # System
                      beam_material,               # Section material
                      num_elements,                # Number of elements
                      start_point,                 # Start point
                      end_point,                   # End point
                      chrono.ChVectorD(0, 0, 1))   # Direction for section

    nodes = builder.GetLastBeamNodes()

    # 3. Add rotational motor at first node
    motor_body = chrono.ChBody()
    motor_body.SetBodyFixed(True)
    system.Add(motor_body)

    # Create motor and set rotation function
    motor = chrono.ChLinkMotorRotationAngle()
    rot = ChQuaternionD()
    rot.Q_from_AngAxis(chrono.CH_C_PI / 2, ChVectorD(0, 1, 0))  # Rotate frame
    motor_frame = chrono.ChFrameD(nodes[0].GetPos(), rot)
    motor.Initialize(motor_body, nodes[0], motor_frame)
    motor.SetAngleFunction(chrono.ChFunction_Ramp(0, 5.0))  # 5 rad/s rotation
    system.Add(motor)

    # 4. Add flywheel at center node
    center_index = int(len(nodes)/2)
    center_node = nodes[center_index]
    
    flywheel = chrono.ChBodyEasyCylinder(0.3, 0.1, 7800)  # Radius, height, density
    flywheel.SetPos(center_node.GetPos())
    system.Add(flywheel)

    # Fix flywheel to center node
    link = chrono.ChLinkMateFix()
    link.Initialize(flywheel, center_node)
    system.Add(link)

    # 5. Set up visualization
    application = irr.ChIrrApp(system, "Jeffcott Rotor with IGA Beam", irr.dimension2du(1024, 768))
    application.AddTypicalLights()
    application.AddTypicalCamera(irr.vector3df(1.5, 0.5, 2.0), irr.vector3df(0, 0, 0))
    
    # Visualization assets
    for node in nodes:
        node_vis = fea.ChVisualizationFEAmesh(node)
        node_vis.SetSymbolsScale(0.01)
        node_vis.SetSymbolsColor(chrono.ChColor(1, 0, 0))
        node.AddAsset(node_vis)

    application.AssetBindAll()
    application.AssetUpdateAll()

    # 6. Simulation loop
    application.SetTimestep(0.001)
    application.SetTryRealtime(True)

    while application.GetDevice().run():
        application.BeginScene()
        application.DrawAll()
        application.DoStep()
        application.EndScene()

if __name__ == '__main__':
    main()