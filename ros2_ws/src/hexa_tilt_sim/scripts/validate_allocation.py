#!/usr/bin/env python3
"""
Single source of truth for the control-allocation matrix M_PHI.

Builds the 6x6 allocation (Omega^2 -> body wrench [fx,fy,fz,tphi,ttheta,tpsi]) DIRECTLY
from the Gazebo model `sdf/fah_hexa/model.sdf`, by reading each rotor's pose (position +
orientation quaternion => thrust axis) and its MulticopterMotorModel plugin
(motorNumber, turningDirection, motorConstant=kT, momentConstant=kQ/kT). Because both the
simulated plant and the controller allocation come from this one file, they cannot drift.

Usage:
  python3 validate_allocation.py            # print M_PHI / M_PHI_INV C++ + checks
  python3 validate_allocation.py --check    # also diff against params.hpp (nonzero exit on mismatch)

Column i of M_PHI corresponds to fah_hexa motorNumber i.
"""
import argparse
import os
import re
import sys
import xml.etree.ElementTree as ET
import numpy as np

np.set_printoptions(precision=10, suppress=False, linewidth=200)

HERE = os.path.dirname(os.path.abspath(__file__))
SDF = os.path.normpath(os.path.join(HERE, "..", "sdf", "fah_hexa", "model.sdf"))
PARAMS = os.path.normpath(os.path.join(HERE, "..", "include", "hexa_tilt_sim", "params.hpp"))


def quat_axis(qx, qy, qz, qw):
    """Third column of R(q): body-frame direction of the rotor's local +z (thrust axis)."""
    return np.array([2 * (qx * qz + qy * qw),
                     2 * (qy * qz - qx * qw),
                     1 - 2 * (qx * qx + qy * qy)])


def parse_sdf(path):
    root = ET.parse(path).getroot()
    model = root.find("model")
    # rotor link poses
    links = {}
    for link in model.findall("link"):
        name = link.get("name")
        if not name.startswith("rotor_"):
            continue
        pose = link.find("pose")
        vals = [float(v) for v in pose.text.split()]
        pos = np.array(vals[0:3])
        quat = vals[3:7]  # x y z w (rotation_format quat_xyzw)
        links[name] = (pos, quat)
    # motor plugins
    rotors = {}
    for plug in model.findall("plugin"):
        if "MulticopterMotorModel" not in (plug.get("name") or ""):
            continue
        link = plug.find("linkName").text.strip()
        num = int(plug.find("motorNumber").text)
        spin = +1 if plug.find("turningDirection").text.strip() == "ccw" else -1
        kT = float(plug.find("motorConstant").text)
        kQ = kT * float(plug.find("momentConstant").text)
        rotors[num] = (link, spin, kT, kQ)
    return links, rotors


def build_M(links, rotors):
    M = np.zeros((6, 6))
    for num in range(6):
        link, spin, kT, kQ = rotors[num]
        pos, quat = links[link]
        n = quat_axis(*quat)
        M[0:3, num] = kT * n
        M[3:6, num] = kT * np.cross(pos, n) - spin * kQ * n
    return M


def cpp(name, A):
    rows = ["    {" + ", ".join(f"{A[r, c]: .10e}" for c in range(6)) + "}" + ("," if r < 5 else "")
            for r in range(6)]
    return f"static constexpr double {name}[6][6] = {{\n" + "\n".join(rows) + "\n};"


def parse_params_matrix(text, name):
    m = re.search(name + r"\[6\]\[6\]\s*=\s*\{(.+?)\};", text, re.S)
    nums = re.findall(r"[-+]?\d+\.\d+e[-+]?\d+", m.group(1))
    return np.array([float(x) for x in nums]).reshape(6, 6)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    args = ap.parse_args()

    links, rotors = parse_sdf(SDF)
    M = build_M(links, rotors)
    Minv = np.linalg.inv(M)

    fz = M[2, :]
    print(f"[sdf] {SDF}")
    print(f"[check] fz row uniform? spread={fz.max()-fz.min():.2e}  value~{fz.mean():.4e}")
    print(f"[check] det(M_PHI)={np.linalg.det(M):.2e} (nonzero => fully actuated)")
    print(f"[check] M@inv residual={np.max(np.abs(M @ Minv - np.eye(6))):.2e}\n")

    print("// ---- paste into include/hexa_tilt_sim/params.hpp ----")
    print(cpp("M_PHI", M)); print(); print(cpp("M_PHI_INV", Minv))
    print(f"\n// FZ per motor = {M[2,0]:.6e}  ; positive fx entry M_PHI[0][2] = {M[0,2]:.6e}")

    if args.check:
        txt = open(PARAMS).read()
        # relative error (M_PHI_INV entries are ~1e5, so absolute tol would false-alarm on
        # the 10-digit truncation of the C++ literals).
        def rel(a, b):
            return np.max(np.abs(a - b) / (np.abs(b) + 1e-12))
        eM = rel(parse_params_matrix(txt, "M_PHI"), M)
        eI = rel(parse_params_matrix(txt, "M_PHI_INV"), Minv)
        print(f"\n[diff vs params.hpp, relative] M_PHI={eM:.2e}  M_PHI_INV={eI:.2e}")
        if max(eM, eI) > 1e-6:
            print("MISMATCH: regenerate params.hpp from the block above.")
            sys.exit(1)
        print("params.hpp is consistent with the SDF.")


if __name__ == "__main__":
    main()
