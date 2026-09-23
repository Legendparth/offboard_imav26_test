#!/usr/bin/env python3
"""Height above the ground, and whether PX4 actually believes it.

One question, asked the same way everywhere: is EKF2 fusing the downward
rangefinder, and if so how high are we?

WHY THIS IS NOT ONE LINE

    VehicleLocalPosition.dist_bottom_valid looks like the answer and is not.
    On PX4 up to and including v1.17 it is simply the terrain estimate's
    validity:

        EKF2.cpp:1622   lpos.dist_bottom_valid = _ekf.isTerrainEstimateValid();

    With EKF2_HGT_REF = 2 (Range) -- which is what an optical-flow aircraft
    like this one wants, because the ground IS the height datum -- the terrain
    state is not estimated at all. Terrain is pinned to zero and both terrain
    aiding paths are switched off by construction, so that flag can NEVER
    become true however perfectly the rangefinder is working.

    Reading it anyway is how thermal_bench came to report "no rangefinder" for
    an entire bench session on an aircraft whose flight nodes, which ask
    EstimatorStatusFlags instead, could see the rangefinder perfectly.

    So: ask the flags. Fall back to dist_bottom_valid only for a firmware that
    does not publish them at all.

WHY cs_rng_kin_consistent IS NOT OPTIONAL

    It is the switch that actually gates fusion:

        range_height_control.cpp:208
            if (_range_sensor.isDataHealthy()
                && _control_status.flags.rng_kin_consistent) {
                    fuseHaglRng(...);
            }

    With it false no range measurement is ever fused, cs_rng_hgt stays true,
    and the height estimate quietly free-runs on integrated accelerometer
    data -- which is how a vehicle sitting on the floor once reported 2.9 m.
    The flag starts true and can only be re-earned at |vz| > 0.5 m/s, so if it
    is false on the ground the aircraft cannot be trusted until PX4 is
    rebooted or it is moved briskly up and down by hand.

The canonical copy of this logic is OffboardSequence.rangefinder_is_healthy();
this module exists so that nodes which are deliberately NOT flight nodes --
the bench, the detectors -- can ask the same question without inheriting a
class that publishes setpoints.
"""

RANGEFINDER_TOPIC = '/uav_1/fmu/out/estimator_status_flags'


def rangefinder_is_healthy(flags, local_position=None):
    """Is EKF2 actually fusing the downward rangefinder?

    `flags` is the newest EstimatorStatusFlags, or None if that topic is not
    being published. `local_position` is only consulted for the fallback.
    """
    if flags is None:
        lp = local_position
        return lp is not None and lp.dist_bottom_valid
    return ((flags.cs_rng_hgt or flags.cs_rng_terrain)
            and not flags.cs_rng_fault
            and not flags.cs_rng_stuck
            and flags.cs_rng_kin_consistent)


def agl(flags, local_position):
    """Metres above whatever is underneath, or None if it is not trustworthy.

    Over a box that is the height above the BOX TOP, which is the number a
    drop and a pixel-to-metres conversion both want.
    """
    lp = local_position
    if lp is None or not rangefinder_is_healthy(flags, lp):
        return None
    return float(lp.dist_bottom)


def why_no_height(flags, local_position):
    """Which link in the chain is broken, in words.

    "no rangefinder" sends people to look at the sensor when the answer was a
    dead agent, or EKF2 declining to fuse a sensor that is working fine. Each
    of those has a different fix, so each gets its own sentence.
    """
    if local_position is None:
        return ("NO /uav_1/fmu/out/vehicle_local_position AT ALL -- the uXRCE-DDS "
                "agent is not connected to PX4 (agent:=false, or the wrong "
                "serial port/baud)")
    if flags is None:
        return ("no estimator_status_flags, and dist_bottom_valid is false -- "
                "with EKF2_HGT_REF=Range that flag is always false, so this "
                "may well be a rangefinder that is working perfectly")
    if flags.cs_rng_fault:
        return "rangefinder FAULT flagged by EKF2"
    if flags.cs_rng_stuck:
        return "rangefinder STUCK (the same reading over and over)"
    if not flags.cs_rng_kin_consistent:
        return ("rangefinder not kinematically consistent -- EKF2 is refusing "
                "to fuse it. Re-earned only at |vz| > 0.5 m/s: move it briskly "
                "up and down by hand, or reboot PX4")
    if not (flags.cs_rng_hgt or flags.cs_rng_terrain):
        return ("EKF2 is not using the rangefinder for height (check "
                "EKF2_HGT_REF and EKF2_RNG_CTRL)")
    return "rangefinder unhealthy"
