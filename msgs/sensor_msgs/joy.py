from utils.websocket_manager import WebSocketManager
from typing import List, Any

class Joy:
    def __init__(self, ws_manager: WebSocketManager, topic: str = "/joy"):
        self.publisher = ws_manager
        self.topic = topic

    def publish(self, axes: List[float], buttons: List[int]):
        msg = {
            "axes": axes,
            "buttons": buttons
        }
        message = {
            "op": "publish",
            "topic": self.topic,
            "msg": msg
        }
        self.publisher.send(message)
        return True

    def subscribe(self):
        subscribe_msg = {
            "op": "subscribe",
            "topic": self.topic
        }
        self.publisher.send(subscribe_msg)
        raw = self.publisher.receive_binary()
        if not raw:
            return None
        import json
        try:
            msg = json.loads(raw)
            if "msg" in msg:
                return json.dumps(msg["msg"], indent=2, ensure_ascii=False)
            return json.dumps(msg, indent=2, ensure_ascii=False)
        except Exception as e:
            print(f"[JointState] Failed to parse: {e}")
            return None
