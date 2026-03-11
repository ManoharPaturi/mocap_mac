import zmq
import json
import msgpack

print("🎧 Firing up the wiretap on 10.28.109.228:6001...")

ctx = zmq.Context()
s = ctx.socket(zmq.SUB)
s.connect("tcp://10.28.109.228:6001")
s.setsockopt_string(zmq.SUBSCRIBE, "")

print("⏳ Waiting for a single frame from the Windows PC...")
msg = s.recv()
print(f"✅ Intercepted packet! Size: {len(msg)} bytes")

try:
    data = json.loads(msg.decode('utf-8'))
    print("📝 Format: JSON")
except:
    data = msgpack.unpackb(msg, raw=False)
    print("📦 Format: MSGPACK")

print(f"\n🔑 TOP-LEVEL KEYS: {list(data.keys())}")

for k, v in data.items():
    if k == 'image' or k == 'frame_jpeg':
        print(f" 🖼️ {k}: <IMAGE BYTES OMITTED>")
    elif isinstance(v, list):
        print(f" 📊 {k}: List with {len(v)} items")
        if len(v) > 0:
            print(f"    Sample: {str(v[0])[:80]}...")
    elif isinstance(v, dict):
        print(f" 📚 {k}: Dict with keys {list(v.keys())}")
    else:
        print(f" 📄 {k}: {v}")