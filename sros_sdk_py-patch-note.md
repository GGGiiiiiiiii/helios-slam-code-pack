# sros_sdk_py 补充说明（不在本包内）

当前版本 **不需要** 修改 `/home/admin/sros_sdk_py/`。

雷达数据使用 SDK 默认的 `loc_laser_points` 推送模式（`set_laser_point_upload` + 回调重建 LaserScan）。

若曾添加过 `fetch_sensor_samples()`（lidar2d 实验），可删除；不影响 loc 模式。
