"""SROS 移动任务下发（扩展 SDK 未暴露的字段）。"""

from sros_sdk_py import main_pb2


def send_move_to_station(
    srp_client,
    movement_no: int,
    station_id: int,
    *,
    no_rotate: bool = False,
    avoid_policy=main_pb2.MovementTask.OBSTACLE_AVOID_WAIT,
) -> None:
    """下发 MT_MOVE_TO_STATION；no_rotate=True 时设置 DS_NO_ROTATE（到站不对准站点角度）。"""
    protobuf = srp_client._srp._protobuf
    srp = srp_client._srp

    def _move_to_station(seq, no, station_id):
        movement_task = main_pb2.MovementTask(
            no=no,
            type=main_pb2.MovementTask.MT_MOVE_TO_STATION,
            stations=[station_id],
            avoid_policy=avoid_policy,
        )
        if no_rotate:
            movement_task.dst_station_type = main_pb2.MovementTask.DS_NO_ROTATE
        protobuf._send_command_msg(
            seq, main_pb2.CMD_NEW_MOVEMENT_TASK, movement_task=movement_task
        )

    srp._run_sync_threadsafe(_move_to_station, movement_no, station_id)
