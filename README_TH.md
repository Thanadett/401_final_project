# ram_bug: RAM-Bug บน TurtleBot3 Burger (ROS 2 Jazzy)

แพ็กเกจ ROS 2 ที่นำอัลกอริทึม RAM-Bug จากเกมจำลองมาใช้กับหุ่นจริง
หุ่นของเราเป็น **ฝ่ายหนี**: รับจุด Goal แล้ววิ่งไปให้ถึง โดยหลบหุ่นคนไล่ ใช้ **LiDAR + odometry อย่างเดียว**

```
ram_bug_ws/src/ram_bug/
├── ram_bug/
│   ├── eg_core.py           อัลกอริทึม RAM-Bug (ชุดเดียวกับเกมจำลอง)
│   ├── brain.py             แปลง LaserScan → แผนที่ตาราง, ตัดสินใจ, คุมล้อ (ไม่มี ROS)
│   ├── ram_bug_node.py      ROS node: /scan /odom /goal_pose → /cmd_vel
│   └── trap_chaser_node.py  คนไล่ทดสอบใน Gazebo (ใช้เฉพาะตอนจำลอง)
├── launch/
│   ├── sim.launch.py        Gazebo: สนาม 3×3 m + Burger + คนไล่แบบกับดัก
│   └── robot.launch.py      หุ่นจริง
├── config/ram_bug.yaml      ค่าตั้งต่าง ๆ (ความเร็ว, ระยะจับ, ...)
├── config/best_params.json  ค่าน้ำหนักที่จูนจากการจำลอง
├── config/ram_bug.rviz      หน้าจอ RViz
├── worlds/arena_1..5        สนามสุ่ม 5 แบบ (.sdf + .json)
├── tools/make_world.py      สร้างสนามสุ่มใหม่
└── test/                    ทดสอบโดยไม่ต้องมี ROS
```

## 1. ติดตั้ง (Ubuntu 24.04 + ROS 2 Jazzy)

```bash
sudo apt install ros-jazzy-turtlebot3 ros-jazzy-turtlebot3-simulations ros-jazzy-ros-gz
# ถ้าไม่มี ros-jazzy-turtlebot3-simulations ให้ build จาก source (branch jazzy):
#   git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git  (ใส่ใน src ของ workspace)
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc

cp -r ram_bug_ws ~/          # หรือวาง src/ram_bug ลงใน workspace ที่มีอยู่
cd ~/ram_bug_ws
colcon build --symlink-install
source install/setup.bash
```

## 2. ทดสอบใน Gazebo ก่อน

```bash
ros2 launch ram_bug sim.launch.py                         # สนาม arena_1, คนไล่แบบทีมเพื่อน (intercept/chase + แชร์ตำแหน่ง UDP)
ros2 launch ram_bug sim.launch.py share:=false            # ไม่แชร์ตำแหน่ง ใช้ LiDAR หาคนไล่อย่างเดียว
ros2 launch ram_bug sim.launch.py chaser_mode:=trap       # คนไล่แบบกับดัก (รอที่ Goal)
ros2 launch ram_bug sim.launch.py world:=arena_3          # สนามอื่น
ros2 launch ram_bug sim.launch.py chaser_mode:=pursuit    # คนไล่วิ่งเข้าหาตั้งแต่ต้น
ros2 launch ram_bug sim.launch.py goal_x:=nan             # รอให้กด 2D Goal Pose ใน RViz
```

- คนไล่เป็นทรงกระบอกสีแดง โหมด `friend` (ค่าเริ่มต้น) เลียนแบบโค้ดทีมเพื่อน: ไกลกว่า 0.8 m ดักหน้า (ตำแหน่ง + ความเร็ว × เวลา),
  ใกล้กว่านั้นวิ่งเข้าหาตรง ๆ (hysteresis 1.0 m) และส่งตำแหน่งตัวเองทาง UDP พอร์ต 47002 เหมือนหุ่นจริง
- `chaser_mode:=trap` รอจนเราเหยียบ particle รอบ Goal (ลูกศรชมพูใน RViz) แล้วจึงวิ่งเข้าหา
- ดูผล: `ros2 topic echo /ram_bug/mode` (โหมดของเรา) และ `ros2 topic echo /chaser/state` (`CAUGHT` = โดนจับ)
- สร้างสนามใหม่: `python3 src/ram_bug/tools/make_world.py --seed 7` แล้ว `colcon build` อีกครั้ง

## 3. รันบนหุ่นจริง

**แบบง่ายสุด (ไม่ต้องวัดจุดเริ่ม):** สนามต้องมีผนังล้อมรอบ วางหุ่นตรงไหนก็ได้ในสนาม แล้ว
```bash
ros2 launch ram_bug robot.launch.py share:=false cmd_vel_stamped:=true rviz:=true
```
อย่าจับหุ่นประมาณ 1-2 วินาที หุ่นจะหาผนังเองแล้วขึ้นว่า `auto_start: robot is at ...`
จากนั้นกด **2D Goal Pose** ใน RViz (Fixed Frame = `arena`) หุ่นจะวิ่งไปเลย
(มุมสนามที่เป็น (0, 0) หุ่นเลือกเอง จึงใช้คู่กับการแชร์ตำแหน่งไม่ได้ ถ้าจะแชร์ต้องใส่ start เอง)

**ระบบพิกัด:** ใช้มุมสนามเป็นจุด (0, 0) วัดเป็นเมตร แกน x ไปตามขอบหนึ่ง แกน y ไปตามอีกขอบ

1. วางหุ่นในสนาม แล้ววัดตำแหน่ง `start_x`, `start_y` และทิศที่หุ่นหัน `start_yaw`
   (0 = หันตามแกน x, 1.5708 = หันตามแกน y, 3.1416 = หันสวนแกน x)
2. บนหุ่น (Raspberry Pi):
   ```bash
   ros2 launch turtlebot3_bringup robot.launch.py
   ```
3. บนหุ่นหรือคอมที่ใช้ `ROS_DOMAIN_ID` เดียวกัน:
   ```bash
   ros2 launch ram_bug robot.launch.py start_x:=1.5 start_y:=0.2 start_yaw:=1.5708 goal_x:=1.5 goal_y:=2.7
   ```
   หรือเว้น `goal_x`, `goal_y` ไว้ แล้วกด **2D Goal Pose** ใน RViz (`rviz:=true`)
4. **แชร์ตำแหน่งกับทีมคนไล่** ใส่ `peer_host:=<IP ของ Pi คนไล่>` เพิ่มในคำสั่งข้อ 3
   โหนด `peer_link` จะส่งตำแหน่งเรา (UDP 47001) และรับตำแหน่งคนไล่ (UDP 47002) บน Pi ของเราเอง
   **ไม่ต้องให้รหัสผ่านหรือ SSH กับอีกทีม** รายละเอียดรูปแบบข้อความอยู่ใน `PROTOCOL_TH.md` (ส่งไฟล์นี้ให้ทีมเพื่อน)
   - ถ้าได้ตำแหน่งจาก UDP: ใช้ตำแหน่งนั้นแทนการเดาจาก LiDAR (`/ram_bug/mode` จะขึ้น `chaser from shared`)
   - ถ้าข้อความขาดเกิน 0.6 s: กลับไปใช้ LiDAR หาคนไล่เองอัตโนมัติ (`chaser from lidar`)
   - เช็ค: `ros2 topic echo /ram_bug/peer_status`

**ต้องเปิดโหนดนี้ใหม่ทุกรอบ** หุ่นต้องอยู่ที่จุดเริ่มตอนเปิด `turtlebot3_bringup` เพราะ odometry เริ่มนับจากตรงนั้น

### เช็คลิสต์ก่อนแข่ง
- [ ] `ros2 topic hz /scan` ได้ราว 5 Hz และ `ros2 topic echo /odom` ขยับเมื่อเข็นหุ่น
- [ ] ใส่ `start_x`, `start_y`, `start_yaw` ตรงกับที่วางจริง (ผิดไป 10 cm = Goal ผิดไป 10 cm)
- [ ] เปิด RViz ดูว่า **แผนที่สีเทาซ้อนตรงกับจุด LiDAR** ถ้าเอียง แปลว่า `start_yaw` ผิด
- [ ] ทดลองเดินไป Goal โดยไม่มีคนไล่ 1 รอบ
- [ ] `/ram_bug/peer_status` ขึ้น `ok` และ `/ram_bug/peer_pose` ตรงกับตำแหน่งคนไล่จริง (ถ้าใช้การแชร์)
- [ ] ถ้าหุ่นไม่ขยับเลย: ลอง `cmd_vel_stamped:=false` หรือ `true` (โหนดพยายามเดาให้เองอยู่แล้ว)

## 4. ค่าที่ควรปรับหน้างาน (`config/ram_bug.yaml`)

| ค่า | เริ่มต้น | ปรับเมื่อ |
|---|---|---|
| `capture_r` | 0.30 | กติการะยะจับเปลี่ยน |
| `max_lin` / `max_ang` | 0.20 / 2.5 | หุ่นส่ายหรือเลี้ยวไม่ทัน |
| `inflate_cells` | 1 (0.1 m) | หุ่นเฉี่ยวกล่อง → 2, ผ่านช่องแคบไม่ได้ → 0 |
| `robot_max_cells`, `robot_max_extent` | 4, 3 | หุ่นมองกล่องเล็กเป็นคนไล่ → ลดลง / มองคนไล่ไม่เห็น → เพิ่ม |
| `time_limit` | 75 s | ตามกติกา |

## 5. ทดสอบโดยไม่ต้องมี ROS (ทำแล้วในเครื่องผม)

```bash
python3 src/ram_bug/test/test_brain_offline.py     # หุ่นแบบล้อจริง + LiDAR จำลอง 360 ลำ
python3 src/ram_bug/test/test_brain_offline.py --chaser Friend             # คนไล่แบบทีมเพื่อน + แชร์ตำแหน่ง
python3 src/ram_bug/test/test_brain_offline.py --chaser Friend --no-share  # เทียบกับ LiDAR อย่างเดียว
python3 src/ram_bug/test/test_node_mock.py         # เช็คว่า node รับ/ส่งข้อความถูก
python3 src/ram_bug/test/test_peer_link.py         # เช็ค UDP รับ/ส่ง และรูปแบบข้อความ
```

ผลทดสอบ offline:

| คนไล่ | แชร์ตำแหน่ง UDP | LiDAR อย่างเดียว |
|---|---|---|
| ทีมเพื่อน (intercept/chase, เริ่มห่าง 1-3 m หันหาเรา) 20 รอบ | **ถึง Goal 5-6/20** | ถึง Goal 2-3/20 |
| กับดัก (รอที่ Goal) 15 รอบ | - | ถึง Goal 14/15 |
| วิ่งตรงเข้าหา 15 รอบ | - | ถึง Goal 10/15 |

- การแชร์ตำแหน่งช่วยได้ชัด (ราว 2 เท่า) เพราะรู้ตำแหน่งคนไล่ตลอด แม้ถูกกล่องบัง
- คนไล่ของทีมเพื่อนเก่งมากเมื่อเริ่มอยู่ **ระหว่างเรากับ Goal** (ความเร็วเท่ากัน + ดักหน้า) รอบที่แพ้ส่วนใหญ่เป็นแบบนี้
- ในเกมจำลองแบบตาราง (tune.py ฝึกกับคนไล่แบบเพื่อน) ได้ success 62% ค่าที่ฝึกแล้วอยู่ใน `config/best_params.json`
  (ค่าเดิมที่ฝึกกับคนไล่แบบกับดักเก็บไว้ที่ `config/best_params_trap.json`)

## ข้อจำกัด / สิ่งที่ยังไม่ได้ทดสอบ
- **ยังไม่ได้รันใน ROS 2 / Gazebo จริง** เครื่องที่ผมใช้ติดตั้ง ROS ไม่ได้ จึงทดสอบเฉพาะตรรกะด้วยตัวจำลองของผมเอง
  ถ้ารันแล้วเจอ error ส่งข้อความ error มาได้เลย
- odometry ของหุ่นจริงคลาดสะสม (ล้อลื่น) ในสนาม 3 m ยังพอใช้ได้ แต่ถ้า Goal คลาดเกิน 10 cm ควรเช็คการวางหุ่นตอนเริ่ม
- ถ้าคนไล่ยืนชิดกล่องตั้งแต่ต้น LiDAR จะแยกไม่ออกว่าเป็นหุ่น จนกว่ามันจะขยับ (ปัญหานี้หายไปเมื่อแชร์ตำแหน่ง)
- รูปแบบข้อความ UDP จริงของทีมเพื่อนยังไม่รู้แน่ (ไฟล์ tb3_connect.py ที่ได้มาเป็นแค่ตัวเช็ค SSH)
  ตัวรับของเรารองรับหลายรูปแบบแล้ว แต่ควรให้ทีมเพื่อนส่งตาม `PROTOCOL_TH.md`
