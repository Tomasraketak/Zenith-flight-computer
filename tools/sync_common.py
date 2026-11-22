#!/usr/bin/env python3
"""Check (or with --fix refresh) the per-sketch copies of firmware/common/*.h.

The Arduino builder compiles a sketch from a copy of its own folder, so a
`#include "../common/x.h"` does not resolve. Every sketch therefore carries
copies; firmware/common is the original. The Rampa sketch lives in the
separate RampaCode repository; if it is checked out next to this one
(../RampaCode) its copy of zenith_link.h is checked too.
"""
import filecmp
import os
import shutil
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
COMMON = os.path.join(ROOT, "firmware", "common")

SKETCHES = {
    "firmware/Zenith_Master":   ["zenith_link.h", "zenith_params.h", "zenith_flight.h"],
    "firmware/Zenith_Position": ["zenith_link.h", "zenith_kf.h", "zenith_nav.h"],
    "firmware/Ground_Station":  ["zenith_link.h"],
    "../RampaCode/Rampa_MCU":   ["zenith_link.h"],
}


# copies of headers whose original lives in ANOTHER repository (checked when it is next to this one)
FOREIGN = [
    ("../Daidalos_Landing_Rocket_Code/Ariadna_V2/common/ariadna_rampa_link.h", "../RampaCode/Rampa_MCU/ariadna_rampa_link.h"),
]


def main() -> int:
    fix = "--fix" in sys.argv
    bad = 0
    for sk, files in SKETCHES.items():
        d = os.path.normpath(os.path.join(ROOT, sk))
        if not os.path.isdir(d):
            print(f"skip   {sk} (not checked out)")
            continue
        for f in files:
            src, dst = os.path.join(COMMON, f), os.path.join(d, f)
            if os.path.exists(dst) and filecmp.cmp(src, dst, shallow=False):
                print(f"ok     {sk}/{f}")
                continue
            if fix:
                shutil.copyfile(src, dst)
                print(f"FIXED  {sk}/{f}")
            else:
                print(f"STALE  {sk}/{f}")
                bad += 1
    for src, dst in FOREIGN:
        src, dst = os.path.normpath(os.path.join(ROOT, src)), os.path.normpath(os.path.join(ROOT, dst))
        if not (os.path.exists(src) and os.path.isdir(os.path.dirname(dst))):
            continue
        if os.path.exists(dst) and filecmp.cmp(src, dst, shallow=False):
            print(f"ok     {os.path.relpath(dst, ROOT)} (= Daidalos)")
        elif fix:
            shutil.copyfile(src, dst)
            print(f"FIXED  {os.path.relpath(dst, ROOT)}")
        else:
            print(f"STALE  {os.path.relpath(dst, ROOT)} (differs from Daidalos)")
            bad += 1
    if bad:
        print(f"\n{bad} stale copies - run: python3 tools/sync_common.py --fix")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
