# Measured synthetic 2.5D benchmark (NOT original-paper replication)

`success` means reaching goal with the shared oracle supervisor enabled. `unsafe_plan` means at least one segment of the proposed global path violates simulator ground-truth limits; it does **not** mean the robot executed an unsafe action. Timing measured on the current container, not embedded UGV hardware.

| Terrain | Planner | n | Success | Unsafe proposal | Safety-stop | Median global planning (ms) | Mean planned length (m) |
|---|---|---:|---:|---:|---:|---:|---:|
|flat|geometry|5|1.00|0.00|0.00|0.77|14.44|
|flat|terrain|5|1.00|0.00|0.00|14.56|14.44|
|slope|geometry|5|0.00|1.00|1.00|0.43|14.44|
|slope|terrain|5|1.00|0.00|0.00|35.51|20.41|
|rough|geometry|5|0.00|1.00|1.00|0.45|14.44|
|rough|terrain|5|1.00|0.00|0.00|21.50|17.27|
|hazard|geometry|5|0.00|1.00|1.00|0.52|14.44|
|hazard|terrain|5|1.00|0.00|0.00|20.05|16.96|
|unknown|geometry|5|0.00|1.00|1.00|0.69|14.44|
|unknown|terrain|5|1.00|0.00|0.00|19.60|18.26|
