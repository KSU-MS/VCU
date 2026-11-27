# vcu_control

kmslap's launch / traction controller, vendored from
`python-data-analysis/controller/` at `40e0100` (KSU-MS python-data-analysis,
"Gear ratio plausibility check"). That's where it's designed and tuned: its
accel sim runs this same C code in the loop against a tire and driveline
model (`kmslap/vcu/firmware.py`).

Local changes, so they can go back upstream:

- **Wheel speeds over CAN.** `ws_teeth = 0` means wheel rpm comes decoded
  over CAN (`ws_rpm`, `ws_rpm_new` at the end of `vcu_inputs_t`) instead of
  as input-capture edges: this car's VCU has no wheel speed inputs, the AN1
  nodes send rpm. The gear ratio check does the same measurement from
  integrated speeds in that mode.

Used through `include/traction_control.hpp`; see
`docs/traction-control.md`.
