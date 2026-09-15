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
run `matlab/chr_bag_plot.m` and select that CSV file.
