# clean_uav_core

This package is the clean-room glue layer for phase-1 and later stages.

## Intended responsibilities
- odom adapter (ground-truth first, VINS later)
- mode / commander layer for no-RC workflows
- minimal launch entry for EGO + px4ctrl demo
- later stage emergency / fail-safe interfaces

## Phase-1 status
Current phase has moved beyond pure scaffolding.

Implemented runtime glue:
- `scripts/px4_param_bootstrap.py`
- `scripts/truth_odom_adapter.py`
- `scripts/takeoff_land_trigger.py`
- `launch/phase1_truth_odom.launch`
- `launch/phase1_minimal_demo.launch`
- `launch/phase1_fullstack.launch`
- `config/phase1_px4ctrl_no_rc.yaml`

Current scope:
- single UAV
- truth odom first
- `px4ctrl` in `no_RC` mode
- PX4 `COM_RCL_EXCEPT=4` bootstrap for OFFBOARD no-RC SITL
- EGO preset target path
- no XTDrone communication layer
