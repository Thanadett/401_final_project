# โปรโตคอลแชร์ตำแหน่ง (Evader ↔ Pursuer) ผ่าน UDP

เอกสารนี้ให้ทีมคนไล่ (Pursuer) อ่าน **ไม่ต้องใช้ SSH และไม่ต้องรู้รหัสผ่าน Pi ของอีกทีม**
แต่ละทีมรันโปรแกรมรับ/ส่งบนหุ่นของตัวเอง แล้วส่งข้อความ UDP เล็ก ๆ หากันผ่าน Wi-Fi วงเดียวกัน

## 1. พอร์ตและความถี่

| ทิศทาง | ปลายทาง | พอร์ต |
|---|---|---|
| Evader (เรา) → Pursuer | IP ของ Pi คนไล่ | **UDP 47001** |
| Pursuer → Evader (เรา) | IP ของ Pi เรา | **UDP 47002** |

- ส่ง **10 Hz** (ทุก 0.1 s) หนึ่ง datagram = หนึ่ง JSON object
- ถ้าไม่ได้รับข้อความเกิน 0.6 s ฝั่งเราจะกลับไปใช้ LiDAR หาคนไล่เอง (ไม่หยุดหุ่น)

## 2. ระบบพิกัด (สำคัญที่สุด)

ใช้ **พิกัดสนาม (arena frame)** ร่วมกัน: มุมสนามที่ตกลงกันเป็น (0, 0) หน่วยเมตร แกน x ตามขอบหนึ่ง แกน y ตามอีกขอบ
yaw เป็นเรเดียน (0 = หันตามแกน x, ทวนเข็มนาฬิกาเป็นบวก)

แต่ละทีมวัดจุดเริ่มของหุ่นตัวเองในพิกัดนี้ แล้วแปลง odometry เป็นพิกัดสนามก่อนส่ง:

```
x_arena   = start_x + cos(start_yaw)*odom_x - sin(start_yaw)*odom_y
y_arena   = start_y + sin(start_yaw)*odom_x + cos(start_yaw)*odom_y
yaw_arena = start_yaw + odom_yaw
```

วิธีนี้ไม่ต้องจับคู่สแกน LiDAR ระหว่างหุ่น (scan matching) เลย

## 3. ข้อความที่เราส่งให้คนไล่ (port 47001)

```json
{"team": "evader", "seq": 17, "t": 1760000000.12, "frame": "arena",
 "x": 1.52, "y": 0.84, "yaw": 1.57, "vx": 0.18, "vy": 0.02,
 "odom_x": 0.70, "odom_y": 0.12, "odom_yaw": 0.0}
```

| ฟิลด์ | ความหมาย |
|---|---|
| `x`, `y`, `yaw` | ตำแหน่งเราในพิกัดสนาม |
| `vx`, `vy` | ความเร็วเรา (m/s, พิกัดสนาม, กรองแล้ว) ใช้ทำ intercept ได้เลย |
| `odom_x/y/yaw` | ค่าเดียวกันใน /odom ของเราเอง (เผื่อทีมคนไล่อยากแปลงพิกัดเอง) |
| `seq`, `t` | ลำดับข้อความ และเวลา (Unix seconds) ไว้ทิ้งข้อความเก่า |

**ไม่มีตำแหน่ง Goal ในข้อความ** (ตามกติกา คนไล่ไม่รู้ Goal)

## 4. ข้อความที่คนไล่ส่งมาให้เรา (port 47002)

แบบแนะนำ:
```json
{"team": "pursuer", "frame": "arena", "x": 2.10, "y": 1.40, "yaw": 3.14}
```

ฝั่งเรารับได้หลายรูปแบบ (ใช้ได้ทันทีถ้าโค้ดเดิมส่งแบบใดแบบหนึ่งอยู่แล้ว):
- `{"x":..,"y":..,"theta":..}` หรือ `"heading"` แทน `yaw`
- `{"pose": {"x":..,"y":..,"yaw":..}}` (หรือคีย์ `pursuer`, `chaser`, `peer`, `robot`)
- `{"position": [x, y]}`
- ข้อความธรรมดา `"x,y,yaw"`

ถ้าทีมคนไล่ส่ง **odometry ของตัวเอง** แทนพิกัดสนาม ให้เราเปิดด้วย
`peer_frame:=peer_odom peer_start_x:=.. peer_start_y:=.. peer_start_yaw:=..` (ตำแหน่งเริ่มของคนไล่)

## 5. ตัวอย่างโค้ดฝั่งคนไล่ (Python ล้วน ไม่ต้องใช้ ROS ก็ทดสอบได้)

```python
import json, socket
EVADER_IP = "192.168.1.50"          # IP ของ Pi ฝั่ง Evader

rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
rx.bind(("0.0.0.0", 47001)); rx.setblocking(False)
tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_my_pose(x, y, yaw):        # เรียกทุก 0.1 s (พิกัดสนาม)
    tx.sendto(json.dumps({"team": "pursuer", "frame": "arena",
                          "x": x, "y": y, "yaw": yaw}).encode(), (EVADER_IP, 47002))

def latest_evader():                # คืนข้อความล่าสุด หรือ None
    last = None
    while True:
        try: last = json.loads(rx.recv(4096))
        except BlockingIOError: return last
```

## 6. วิธีรันฝั่งเรา

```bash
ros2 launch ram_bug robot.launch.py start_x:=0.3 start_y:=0.3 start_yaw:=0.0 \
     goal_x:=2.5 goal_y:=2.6 peer_host:=<IP Pi คนไล่>
ros2 topic echo /ram_bug/peer_status        # "ok, last packet 0.1 s ago" = ได้รับแล้ว
```

เช็คการเชื่อมต่อโดยไม่ใช้ ROS (ใช้แทน tb3_connect.py แบบ SSH เดิม):
```bash
python3 src/ram_bug/tools/tb3_connect.py --peer <IP Pi คนไล่>                       # บน Pi เรา
python3 src/ram_bug/tools/tb3_connect.py --peer <IP Pi เรา> --role pursuer --x 1.5 --y 1.5   # จำลองฝั่งคนไล่
```

ความเข้ากันได้กับสคริปต์เดิมของอีกทีม: ถ้ายังอยากใช้ไฟล์ `/tmp/tb3_peer.json` ให้ใส่ `peer_file:=/tmp/tb3_peer.json`
(แต่แนะนำ UDP เพราะไม่ต้องให้อีกทีม SSH เข้าหุ่นเรา)

## 7. เช็คลิสต์ก่อนแข่ง (ทั้งสองทีม)

- [ ] หุ่นทั้งสองตัวต่อ Wi-Fi วงเดียวกัน และ `ping` หากันได้
- [ ] ROS_DOMAIN_ID ต่างกัน (เช่น 10 / 20) เพื่อไม่ให้ topic `/scan` `/cmd_vel` ชนกัน — UDP นี้ไม่ผ่าน ROS จึงไม่กระทบ
- [ ] ตกลงมุม (0, 0) และทิศแกน x ของสนามร่วมกัน
- [ ] ทดสอบ: เข็นหุ่นคนไล่ไปที่ (1.5, 1.5) แล้วดูว่า `/ram_bug/peer_pose` ได้ประมาณ (1.5, 1.5)
- [ ] ไฟร์วอลล์ไม่บล็อก UDP 47001/47002 (`sudo ufw allow 47001:47002/udp` ถ้าเปิด ufw อยู่)
