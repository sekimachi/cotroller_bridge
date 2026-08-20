#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Int32  
from std_msgs.msg import Int8   


from imrc_messages.msg import BallInfo
from imrc_messages.msg import Circle
from imrc_messages.msg import ConpanelBuzzerControl
from imrc_messages.msg import ConpanelLedControl
from imrc_messages.msg import GeneralCommand
from imrc_messages.msg import LCU
from imrc_messages.msg import LedControl
from imrc_messages.msg import LedData
from imrc_messages.msg import MissBallInfo
from imrc_messages.msg import MotorControl
from imrc_messages.msg import PCU
from imrc_messages.msg import RU
from imrc_messages.msg import RobotActionProgress

from imrc_messages.srv import BrockOperate 
from imrc_messages.srv import GoalPosition
from imrc_messages.srv import IndexSelect
from imrc_messages.srv import ResetMissBall
from imrc_messages.srv import StringRequest   

from imrc_messages.action import BallColor
from imrc_messages.action import BoxCommand
from imrc_messages.action import LinearMove
from imrc_messages.action import Rotate
from imrc_messages.action import TiltAdjustment


# ==========================================
# name ごとの設定たち
# ==========================================
CONFIG = {
    "/brock_conf": {
        "msg_type": Float32,
        "fields": [("data", float)],
    },

    "/brock_color": {
        "msg_type": String,
        "fields": [("data", str)],
    },

    "/brock_YOLO": {
        "msg_type": bool,
        "fields": [("True/False", bool)],
    },

    "/power_state": {
        "msg_type": PCU,
        "fields": [
            ("relay_state", int),
            ("unit_index", int),
            ("mode", str),
        ],
    },
    "/ball_info": {
        "msg_type": BallInfo,
        "fields": [
            ("detected", bool),
            ("dx", int),
            ("dy", int),
            ("depth_cm", float),
        ],
    },
    "/circle": {
        "msg_type": Circle,
        "fields": [
            ("x", float),
            ("y", float),
            ("r", float),
        ],
    },

    "/conpanel_buzzer_control": {
        "msg_type": ConpanelBuzzerControl,
        "fields": [
            ("count", int),
            ("isloop", bool),
        ],
    },

    "/conpanel_led_control": {
        "msg_type": ConpanelLedControl,
        "fields": [
            ("led_index", int),
            ("led_state", bool),
        ],
    },

    "/general_command": {
        "msg_type": GeneralCommand,
        "fields": [
            ("target", str),
            ("param", int),
        ],
    },
    "/LCU": {
        "msg_type": LCU,
        "fields": [
            ("led_id", int),
            ("led_color", str),
            ("led_brightness", float),
            ("led_mode", str),
            ("duration", float),
        ],
    },
    "/led_Control": {
        "msg_type": LedControl,
        "fields": [
            ("led_id", int),
            ("led_color", str),
            ("led_brightness", float),
            ("led_mode", str),
            ("duration", float),
        ],
    },
    "/led_data": {
        "msg_type": LedData,
        "fields": [
            ("led_index", int),
            ("led_color_red", int),
            ("led_color_green", int),
            ("led_color_blue", int),
            ("led_mode", int),
            ("blink_duration", float),
        ],
    },
    "/miss_ball_info": {
        "msg_type": MissBallInfo,
        "fields": [
            ("miss_red", int),
            ("miss_blue", int),
            ("miss_yellow", int),
        ],
    },
    "/motor_control": {
        "msg_type": MotorControl,
        "fields": [
            ("target", str),
            ("param", str),
        ],
    },
    "/pcu": {
        "msg_type": PCU,
        "fields": [
            ("relay_state", int),
            ("unit_index", int),
            ("mode", str),
        ],
    },
    "/ru": {
        "msg_type": RU,
        "fields": [
            ("relay_no", int),
            ("relay_state", int),
            ("unit_index", int),
            ("mode", str),
        ],
    },
    "/robot_action_progress": {
        "msg_type": RobotActionProgress,
        "fields": [
            ("target", str),
            ("param", str),
            ("state", str),
        ],
    },


}

# Service ごとの設定たち
SERVICE_CONFIG = {
    "BrockOperate": {
        "srv_type": BrockOperate,
        "fields": [
            ("color", str),
        ],
    },

    "GoalPosition": {
        "srv_type": GoalPosition,
        "fields": [
            ("position", str),
        ],
    },

    "IndexSelect": {
        "srv_type": IndexSelect,
        "fields": [
            ("selection", int),
        ],
    },

    "ResetMiss_ball": {
        "srv_type": ResetMissBall,
        "fields": [
            ("color", str),
        ],
    },

    "StringRequest": {
        "srv_type": StringRequest,
        "fields": [
            ("target", str),
        ],
    },
        
}

# Action ごとの設定たち
ACTION_CONFIG = {
    "BallColor": {
        "action_type": BallColor,
        "fields": [
            ("color", str),
        ],
    },

    "BoxCommand": {
        "action_type": BoxCommand,
        "fields": [
            ("color", str),
            ("moveforward",bool)
        ],
    },

    "LinearMove": {
        "action_type": LinearMove,
        "fields": [
            ("target_x", float),
            ("target_y", float),
            ("target_yaw", float),
            ("skip_yaw",bool),
        ],
    },

    "Rotate": {
        "action_type": Rotate,
        "fields": [
            ("mode", str),
            ("angle", float),
        ],
    },

    "TiltAdjustment": {
        "action_type": TiltAdjustment,
        "fields": [
            ("direction_x", str),
            ("distance_x", float),
            ("direction_y", str),
            ("distance_y", float),
            ("angle_direction", str),
            ("angle", float),
        ],
    },
}


# ==========================================
# ノード♡
# ==========================================
class Cotroller(Node):
    def __init__(self):
        super().__init__("control_json_relay")

        self._cj_publishers = {}

        # Service Clientを保存するお辞書
        self._cj_clients = {}

        # Action Clientを保存するお辞書
        self._cj_action_clients = {}

        self.subscription = self.create_subscription(String,"cotroll_json",self.on_control_json,10,)

        self.get_logger().info(f'control_json_relay: "{"cotroll_json"}" をいっぱいブリッジするよ！')

    # ==========================================
    # "cotroll_json" 受信時のコールバック
    # ==========================================
    def on_control_json(self, msg: String):
        # JSONとしてパース
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSONの解析に失敗しました: {e} (raw=\"{msg.data}\")")
            return

        name = data.get("name")
        values = data.get("value", [])

        # ★ Actionの場合
        if name in ACTION_CONFIG:
            self.on_action_request(name, values)
            return

        # ★ Serviceの場合
        if name in SERVICE_CONFIG:
            self.on_service_request(name, values)
            return

        if name not in CONFIG:
            self.get_logger().warn(f"未登録のnameです: {name}")
            return

        config = CONFIG[name]
        fields = config["fields"]

        if len(values) < len(fields):
            self.get_logger().error(
                f"{name}: valueの数が足りません (必要{len(fields)}, 受信{len(values)})"
            )
            return

        # メッセージを組み立てる
        ros_msg = config["msg_type"]()
        for (field_name, field_type), raw_value in zip(fields, values):
            try:
                converted = field_type(raw_value)
            except (ValueError, TypeError) as e:
                self.get_logger().error(
                    f'{name}.{field_name} の変換に失敗しました: "{raw_value}" -> '
                    f"{field_type.__name__} ({e})"
                )
                return
            setattr(ros_msg, field_name, converted)

        # Publish（初回のみPublisherを作成してキャッシュしる）
        if name not in self._cj_publishers:
            self._cj_publishers[name] = self.create_publisher(config["msg_type"], name, 10)

        self._cj_publishers[name].publish(ros_msg)
        self.get_logger().info(f"{name} <- {ros_msg}")

    #  ==========================================
    #  Serviceを呼び出す
    #  ==========================================
    def on_service_request(self, name, values):

        config = SERVICE_CONFIG[name]
        fields = config["fields"]

        if len(values) < len(fields):
            self.get_logger().error(
                f"{name}: valueの数が足りません "
                f"(必要{len(fields)}, 受信{len(values)})"
            )
            return

        # Service Clientを初回のみ作成
        if name not in self._cj_clients:
            self._cj_clients[name] = self.create_client(
                config["srv_type"],
                name
            )

            self.get_logger().info(
                f"Service Clientを作成しました: {name}"
            )

        client = self._cj_clients[name]

        # Serviceが存在するか確認
        if not client.service_is_ready():
            self.get_logger().warn(
                f"Serviceがまだ利用できません: {name}"
            )
            return

        # Requestを作成
        request = config["srv_type"].Request()

        # JSONのvalueをRequestへ設定
        for (field_name, field_type), raw_value in zip(fields, values):
            try:
                converted = field_type(raw_value)
            except (ValueError, TypeError) as e:
                self.get_logger().error(
                    f'{name}.{field_name} の変換に失敗しました: "{raw_value}" -> '
                    f"{field_type.__name__} ({e})"
                )
                return

            setattr(request, field_name, converted)

        # Serviceを非同期で呼び出す
        future = client.call_async(request)

        #  結果が返ってきたときの処理
        future.add_done_callback(
            lambda future: self.on_service_response(name, future)
        )

        self.get_logger().info(
            f"{name} -> {request}"
        )

    # ==========================================
    # ServiceのResponseを受け取る
    # ==========================================
    def on_service_response(self, name, future):

        try:
            response = future.result()

        except Exception as e:
            self.get_logger().error(
                f"{name} Service呼び出しに失敗しました: {e}"
            )
            return

        self.get_logger().info(
            f"{name} <- {response}"
        )

        # Responseの中身を取得
        self.get_logger().info(
            f"success = {response.success}"
        )

        self.get_logger().info(
            f"distance = {response.distance}"
        )

    #  ==========================================
    #  Actionを呼び出す（結果だけ受け取る）
    #  ==========================================
    def on_action_request(self, name, values):

        config = ACTION_CONFIG[name]
        fields = config["fields"]

        if len(values) < len(fields):
            self.get_logger().error(
                f"{name}: valueの数が足りません "
                f"(必要{len(fields)}, 受信{len(values)})"
            )
            return

        # Action Clientを初回のみ作成
        if name not in self._cj_action_clients:
            self._cj_action_clients[name] = ActionClient(
                self,
                config["action_type"],
                name,
            )

            self.get_logger().info(
                f"Action Clientを作成しました: {name}"
            )

        client = self._cj_action_clients[name]

        # Action Serverが存在するか確認（見つからなければ待たずに諦める）
        if not client.server_is_ready():
            self.get_logger().warn(
                f"Action Serverがまだ利用できません: {name}"
            )
            return

        # Goalを作成
        goal_msg = config["action_type"].Goal()

        # JSONのvalueをGoalへ設定
        for (field_name, field_type), raw_value in zip(fields, values):
            try:
                converted = field_type(raw_value)
            except (ValueError, TypeError) as e:
                self.get_logger().error(
                    f'{name}.{field_name} の変換に失敗しました: "{raw_value}" -> '
                    f"{field_type.__name__} ({e})"
                )
                return

            setattr(goal_msg, field_name, converted)

        # Goalを非同期で送信（フィードバックは受け取らない）
        send_goal_future = client.send_goal_async(goal_msg)

        send_goal_future.add_done_callback(
            lambda future: self.on_action_goal_response(name, future)
        )

        self.get_logger().info(
            f"{name} -> {goal_msg}"
        )

    # ==========================================
    # Goalが受理されたかどうかを受け取る
    # ==========================================
    def on_action_goal_response(self, name, future):

        try:
            goal_handle = future.result()

        except Exception as e:
            self.get_logger().error(
                f"{name} Goal送信に失敗しました: {e}"
            )
            return

        if not goal_handle.accepted:
            self.get_logger().warn(
                f"{name} Goalが拒否されました"
            )
            return

        self.get_logger().info(
            f"{name} Goalが受理されました"
        )

        # 結果だけ受け取る
        result_future = goal_handle.get_result_future()

        result_future.add_done_callback(
            lambda future: self.on_action_result(name, future)
        )

    # ==========================================
    # Actionの最終結果を受け取る
    # ==========================================
    def on_action_result(self, name, future):

        try:
            result = future.result().result

        except Exception as e:
            self.get_logger().error(
                f"{name} 結果の取得に失敗しました: {e}"
            )
            return

        self.get_logger().info(
            f"{name} <- {result}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = Cotroller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()