# Campaign names

Campaigns are defined only in `experiments/configs/campaigns.yaml`; the
extraction pipeline reads its dataset lists from the same file
(`st_gat/pipeline/config.py`).

## Convention

`nominal`, or `<imu|tl>_<mode>[_<param>]`:
- `<mode>` names the fault shape (`bias`, `scale`, `stuck`, `oscillate`,
  `unknown`, `blackout`, `conf`).
- `<param>`, if the mode has one tunable value, is a zero-padded mnemonic:
  `imu_bias_008` = 0.08 rad/s, `tl_conf_030` = severity 0.30.
- Arm B data goes under `<campaign>_armB` (the runner adds the suffix).

The name is a mnemonic only. Every parameter is copied into each trial's
`metadata.json` (`fault`) and the fault injector's `fault_log.jsonl`
startup line; analysis reads them from there.

## Old → new (data collected before 2026-11, deleted)

`docs/findings.md` and the archive refer to the old names.

| Old | New | Fault |
|---|---|---|
| `nom_v11` (+ `baseline_all`) | `nominal` | none |
| `imu_fault_s1` | `imu_bias_003` | gyro bias 0.03 rad/s, 20 s on / 10 s off |
| `imu_fault_s3` | `imu_bias_008` | gyro bias 0.08 rad/s, 15 s on / 10 s off |
| `imu_fault_scale` | `imu_scale` | gyro x1.8 in turn zones |
| `imu_fault_stuck` | `imu_stuck` | gyro frozen in turn zones |
| `tl_fault_s2` | `tl_oscillate` | GREEN / original, 5 s period |
| `tl_fault_s3` | `tl_unknown` | UNKNOWN |
| `tl_fault_s4` | `tl_blackout` | no signal |
| `tl_fault_fixed_030/050/070` | `tl_conf_030/050/070` | confidence x0.7 / x0.5 / x0.3 |
| `tl_fault_ramp`, `imu_fault_ramp` | dropped | ramps confound severity with time in zone; the modes remain in `fault_injector.py` |
| `nom_v5`, `nom_v7`, `obs_*` | dropped | other speeds and obstacle scenarios are out of scope |
