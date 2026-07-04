"""Independent oracle for the hmmwv_scm task (stdlib math only, NO Chrono).

Vehicle-terramechanics coupling has no tight closed form, so this task uses the plate_sinkage_scm
posture: a Bekker-Wong-ANCHORED coarse band for the rut depth, cross-turn ordering laws, and
calibrated bands (measured once on the pinned build, frozen, and documented) for the drive
observables. The honest statement: the rut bands are physics-anchored, the speed/distance bands
are calibrated; both are stated in CONTRACT.md.

Bekker anchor (static, single wheel, rigid cylinder on Bekker soil):
  pressure p(z) = (Kc/b + Kphi) z^n over a contact strip A(z) = b * 2 sqrt(2 R z - z^2)
  solve F = p(z) A(z) for z, with F = per-wheel static load = m g / 4.
HMMWV numbers used: m = 2450 kg -> F ~= 6009 N/wheel; tire R = 0.4699 m, width b = 0.254 m.
A moving, multi-pass, slipping wheel cuts DEEPER than the static Bekker value (slip-sinkage and
repeated loading), which the band's upper multiplier absorbs.

Cross-turn laws encoded:
  1. rigid -> SCM: final speed drops (motion resistance); a rut appears.
  2. firm -> soft (Kphi 2e6 -> 5e5): the rut deepens; the soft band's lower edge sits ABOVE the
     firm measurement, so "soil not softened" fails.

Run offline once; kept in-repo for provenance.
Reproduce: conda run -n chronobench python demo_data_10/hmmwv_scm/oracle.py
"""
import json

m_veh, g = 2450.0, 9.81
F_wheel = m_veh * g / 4.0
R, b = 0.4699, 0.254


def bekker_static_sinkage(Kphi, Kc, n):
    lo, hi = 1e-5, R
    for _ in range(200):
        z = 0.5 * (lo + hi)
        area = b * 2.0 * (max(2.0 * R * z - z * z, 0.0)) ** 0.5
        force = (Kc / b + Kphi) * z ** n * area
        if force < F_wheel:
            lo = z
        else:
            hi = z
    return 0.5 * (lo + hi)


z_firm = bekker_static_sinkage(2.0e6, 0.0, 1.1)
z_soft = bekker_static_sinkage(5.0e5, 0.0, 1.1)

out = {
    "per_wheel_load_N": round(F_wheel, 1),
    "firm_Kphi_2e6": {"bekker_static_z": round(z_firm, 4),
                      "rut_band": [round(0.5 * z_firm, 4), round(3.5 * z_firm, 4)],
                      "measured_reference_rut": "see CONTRACT.md (calibrated)"},
    "soft_Kphi_5e5": {"bekker_static_z": round(z_soft, 4),
                      "rut_band_lower_raised_for_separation": True,
                      "measured_reference_rut": "see CONTRACT.md (calibrated)"},
}
print(json.dumps(out, indent=2))
