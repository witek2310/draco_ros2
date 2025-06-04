To run just compressiing and decompressing use:
```
ros2 launch draco_ros2 run_draco.py csv_folder_path:=./metrics \
    pointcloud_topic:=/kitti/velodyne \
    encoding_speed:=10 \
    decoding_speed:=10 \
    quantization_bits:=14 \

```

Encodin and decoding speed become slower with decreasing number. 