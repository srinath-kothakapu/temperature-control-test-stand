# Verification notes

Findings from writing the requirement-based tests. IDs are local to this file.

## F-1: the plate can coast above the logger range after the 80 °C cutoff (open, simulation only)

ST-02 drives a simulated plate (gain 1.4 °C per % duty, time constant 120 s, 10 s dead time) with a setpoint above the cutoff. The firmware switches the heater off in the first sample at or above 80 °C and keeps it off, as REQ-10 asks. Because of the dead time, heat that is already on its way keeps warming the plate, and in this simulation the peak is about 86.3 °C. The logger accepts at most 85 °C (`frame_build` returns 0 above that), so the last frames of such a run would be dropped.

This is a property of the plant I assumed. The stand is not built, so the real dead time and heater gain are unknown. To do on the stand: measure the coast after a cutoff, then lower `TEMP_CUTOFF_C` or `DUTY_MAX_PCT` if needed.

## F-2: README said "more than 90 %" but the code uses "90 % or more" (fixed)

`safety_update` trips the runaway check at `duty >= RUNAWAY_DUTY_PCT`. README and `docs/hardware.md` said "more than 90 %". TC-12 now pins the behaviour (exactly 90 % counts, 89.9 % does not) and both documents say "90 % or more".

## F-3: the first anti-windup test could not tell the two cases apart (fixed)

I disabled conditional integration on a copy of the code and the old TC-08 still passed, because a separate clamp also limits the integral to the output range. TC-08 now has a case where P alone saturates the output; with conditional integration the integral stays near 0, without it the output stays at 100 % after the error is gone. The disabled copy now fails TC-08.

## Mutation check

Changes made on a copy of the firmware code, each of which a test now catches: `>=` to `>` at the cutoff (TC-10), `>=` to `>` at the runaway duty (TC-12), the sensor fault limit off by one (TC-11, ST-04), the latch removed (TC-10, TC-11), anti-windup removed (TC-08).
