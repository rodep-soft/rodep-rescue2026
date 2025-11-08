#!/usr/bin/env python3
import websocket
import json

def test_rosbridge():
    print("Connecting to ws://localhost:9090...")

    try:
        ws = websocket.create_connection('ws://localhost:9090', timeout=2)
        print("✅ Connected successfully!")

        # Send a simple command to move to home pose
        move_msg = {
            "op": "publish",
            "topic": "/move_to_pose",
            "msg": {
                "data": "home"
            }
        }
        ws.send(json.dumps(move_msg))
        print("✅ Sent move command: home")

        ws.close()
        print("✅ Connection closed successfully")
        print("\n🎉 rosbridge is working! Ready for Flutter UI!")
        return True

    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    test_rosbridge()
