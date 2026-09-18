# data_logging

`simulation.launch.py` records the CHR topics automatically. Each run creates a
timestamped bag under `~/Desktop/ar_ws/src/data_logging/bags`.

After stopping the simulation, open a new terminal and convert the bag:

```bash
aw
cd ~/Desktop/ar_ws/src/data_logging/bags
python3 ../bag_to_csv.py rosbag2_YYYY_MM_DD-HH_MM_SS/
```

The CSV is created inside the bag directory as
`rosbag2_YYYY_MM_DD-HH_MM_SS/rosbag2_YYYY_MM_DD-HH_MM_SS.csv`. In MATLAB,
run `matlab/chr_bag_plot.m` and select that CSV file. Three dashboards are
created in order: Palletrone flight control, arm control, and CHR whole-body/TCP
control. The plots include PX4-calibrated PWM, DOB and nominal/final torque,
four arm torque definitions, absolute TCP sensor tracking, and DLS diagnostics.
Motor output is shown as 0--100% throttle using the firmware's 1100--1900 us
normalization range; the current firmware safety clamp therefore appears at 80%.
