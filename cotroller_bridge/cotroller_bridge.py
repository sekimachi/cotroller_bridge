#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Int32  
from std_msgs.msg import Int8   


from imrc_messages.msg import PCU
from imrc_messages.msg import BallInfo
from imrc_messages.msg import Circle
from imrc_messages.msg import ConpanelBuzzerControl
from imrc_messages.msg import ConpanelLedControl


from imrc_messages.srv import BrockOperate 
from imrc_messages.srv import GoalPosition
from imrc_messages.srv import IndexSelect
from imrc_messages.srv import ResetMissBall
from imrc_messages.srv import StringRequest   

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



}

# Service ごとの設定たち
SERVICE_CONFIG = {
    "/brock_operate": {
        "srv_type": BrockOperate,
        "fields": [
            ("color", str),
        ],
    },

    "/goal_position": {
        "srv_type": GoalPosition,
        "fields": [
            ("position", str),
        ],
    },

    "/index_select": {
        "srv_type": IndexSelect,
        "fields": [
            ("selection", int),
        ],
    },

    "/reset_miss_ball": {
        "srv_type": ResetMissBall,
        "fields": [
            ("color", str),
        ],
    },

    "/string_request": {
        "srv_type": StringRequest,
        "fields": [
            ("target", str),
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

        # ★ Responseの中身を取得
        self.get_logger().info(
            f"success = {response.success}"
        )

        self.get_logger().info(
            f"distance = {response.distance}"
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