from pychrono importchrono as ch
from pychrono.core import ChVector_, ChMatrixSp_
from pychrono.geometric import ChColorAsset
from pychrono.physics import ChSystemParallel
from pychrono.inputoutput import ChVisualization, ChIrrApp
from pychrono.physics import ChSystemChRONO


chrono_system = ChSystemChRONO()


vis_sys = ChVisualization(chrono_system)
vis_sys.SetWindowSize(800, 600)
vis_sys.SetCameraPos(ChVector_(0, 0, 20))
vis_sys.SetCameraTarget(ChVector_(0, 0, 0))
vis_sys.SetCameraUpVector(ChVector_(0, 1, 0))
vis_sys.Initialize()


vis_sys.EnableWindow()


truss = ch.ChLinkTSDP()
truss.Initialize(ch.ChFrameD(ch.ChVector<>(0, 0, 0)), ch.ChFrameD(ch.ChVector<>(0, 0, 10)), 100, 100)
chrono_system.Add(truss)


bar = ch.ChBody()
bar.SetIdentifier(1)
bar.SetType(ch.ChBodyType().SPHERE)
bar.SetMass(1)
bar.SetInertiaXX(ch.ChVector<>(0.01, 0.01, 0.01))
bar.SetPos(ch.ChVector<>(0, 0, 10))
bar.SetPos_dt(ch.ChVector<>(0, 0, 1))
bar.SetRot(ch.ChQuaternion<>(1, 0, 0, 0))
bar.SetBodyFixed(False)
chrono_system.Add(bar)


gear1 = ch.ChBody()
gear1.SetIdentifier(2)
gear1.SetType(ch.ChBodyType().SPHERE)
gear1.SetMass(0.1)
gear1.SetInertiaXX(ch.ChVector<>(0.001, 0.001, 0.001))
gear1.SetPos(ch.ChVector<>(0, 0, 12))
gear1.SetRot(ch.ChQuaternion<>(1, 0, 0, 0))
gear1.SetBodyFixed(False)
chrono_system.Add(gear1)


gear2 = ch.ChBody()
gear2.SetIdentifier(3)
gear2.SetType(ch.ChBodyType().SPHERE)
gear2.SetMass(0.1)
gear2.SetInertiaXX(ch.ChVector<>(0.001, 0.001, 0.001))
gear2.SetPos(ch.ChVector<>(0, 0, 14))
gear2.SetRot(ch.ChQuaternion<>(1, 0, 0, 0))
gear2.SetBodyFixed(False)
chrono_system.Add(gear2)


motor = ch.ChLinkMotorSpeed()
motor.Initialize(bar, gear1, 1, 0.1)
chrono_system.Add(motor)


gear_interaction = ch.ChLinkTSDP()
gear_interaction.Initialize(ch.ChFrameD(ch.ChVector<>(0, 0, 12)), ch.ChFrameD(ch.ChVector<>(0, 0, 14)), 1, 1)
chrono_system.Add(gear_interaction)


material = ChColorAsset()
material.SetDiffuseColor(ChVector_(0.8, 0.8, 0.8))
material.SetSpecularColor(ChVector_(0.2, 0.2, 0.2))
material.SetShininess(10.0)
material.SetTransparency(0.5)
vis_sys.AddAsset(material)


vis_sys.StartVisualization()