# Verification plan (V-model)

This page says how each requirement of the stand is checked, on which level, and where the evidence is. It follows the V-model: every level of the design on the left has a test level on the right.

```text
Requirements (tests/requirements.csv) ------------------------> Stand tests (HW-xx, not done yet)
   Firmware design (modules, control loop) -------------------> System tests ST-xx (host, simulated plant)
      Module design (decoder, PID, safety, frame) ------------> Integration test IT-01
         Code (firmware/lib/control, analysis/) --------------> Unit tests TC-xx and pytest
```

## Levels in this repo

| Level | Where | What it runs on | Test IDs |
|---|---|---|---|
| Unit | `tests/c/test_control.c`, `tests/test_step_response.py` | PC (gcc, pytest) | TC-01 to TC-13, pytest names |
| Integration | `tests/c/test_control.c` | PC | IT-01 (thermocouple word to UART frame with CRC) |
| System | `tests/c/test_control.c` | PC, control loop against a simulated plant | ST-01 to ST-05 |
| Stand | `docs/test-procedure.md` | the real stand | HW-01 to HW-04, **not done yet** |

The system tests use `stand_step()`, which repeats the order of operations of `loop()` in `firmware/src/main.cpp` with the real values from `config.h`. `main.cpp` itself needs the board and is not covered by the host tests.

## Process

1. **Requirements** are numbered in `tests/requirements.csv` (one line each, with level and how it is verified).
2. **Test cases** carry the requirement IDs they check: in `test_control.c` the table at the bottom, in pytest `@pytest.mark.req("REQ-15")`.
3. **Run:** `make -C tests/c && ./tests/c/test_control` and `python -m pytest tests`. CI runs both on every push.
4. **Report:** `python tools/trace.py` builds `docs/traceability.md` with PASS/FAIL per requirement. CI fails when a test fails, a requirement has no test, a test names an unknown requirement, or the committed matrix is out of date.
5. **Defects:** a failure or a design finding gets a GitHub issue and, when fixed, a commit that references it. Open findings are listed in `docs/verification-notes.md`.

## How to add a requirement

Add a line to `tests/requirements.csv`, add or tag a test with its ID, run `python tools/trace.py --c-results c_results.txt --junit pytest_results.xml` (after the two test commands), and commit `docs/traceability.md`.

## Limits

- PASS means passed on a PC. Several tests run against a simple first-order plant with dead time that I chose; they check the code, not the real heater.
- The tests check the control library. The Arduino-specific code (SPI, PWM, watchdog, pins) is verified only on the stand.
- A requirement counts as verified when at least one test covers it. That does not prove the tests are complete.
