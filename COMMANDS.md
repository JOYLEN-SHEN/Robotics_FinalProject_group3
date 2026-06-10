# Kullanışlı Komutlar

ROS2 Humble · Multi-Robot Warehouse Simulation

---

## Gereksinimler

```bash
# ROS2 Humble kurulu olmalı
source /opt/ros/humble/setup.bash

# Workspace build
cd ~/multi_robot_warehouse_ws
colcon build --symlink-install

# Her yeni terminalde:
source ~/multi_robot_warehouse_ws/install/setup.bash
```

---

## Tam Simülasyonu Başlatma

### Normal başlatma (Nav2 + RViz2 dahil)
```bash
ros2 launch warehouse_gazebo full_simulation.launch.py
```

### Sadece Gazebo + robotlar (Nav2 olmadan)
```bash
ros2 launch warehouse_gazebo full_simulation.launch.py \
  launch_nav2:=false launch_fleet:=false launch_dashboard:=false
```

### RViz2 olmadan başlat
```bash
ros2 launch warehouse_gazebo full_simulation.launch.py \
  launch_rviz:=false launch_fleet:=false launch_dashboard:=false
```

### Özel harita dosyasıyla başlat
```bash
ros2 launch warehouse_gazebo full_simulation.launch.py \
  map:=/tam/yol/harita.yaml
```

**Başlatma sırası (otomatik):**
- 0s  → Gazebo dünyası + RViz2 + TF relay
- 5s  → 4 robot spawn edilir
- 12s → Nav2 başlar (4 robot için ayrı ayrı)

---

## Harita Oluşturma (SLAM)

Yeni bir harita çıkarmak istersen bu adımları takip et.

### Adım 1 — Gazebo'yu Nav2 olmadan başlat
```bash
ros2 launch warehouse_gazebo full_simulation.launch.py \
  launch_nav2:=false launch_fleet:=false launch_dashboard:=false launch_rviz:=false
```

### Adım 2 — SLAM başlat (robot_1 ile)
```bash
ros2 launch warehouse_navigation slam.launch.py robot_name:=robot_1
```

### Adım 3 — Robotu sürerek depoyu tara
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap /cmd_vel:=/robot_1/cmd_vel
```

> Klavye kontrolleri: `i` ileri, `,` geri, `j`/`l` dönüş, `k` dur

### Adım 4 — Haritayı kaydet
```bash
# Harita doğrudan src/warehouse_navigation/maps/ klasörüne kaydedilir
ros2 launch warehouse_navigation save_map.launch.py

# Farklı isimle kaydetmek için:
ros2 launch warehouse_navigation save_map.launch.py \
  map_name:=yeni_harita output_dir:=/home/adil/haritalar
```

Kaydedilen dosyalar: `warehouse_map.pgm` (görüntü) + `warehouse_map.yaml` (meta)

---

## Navigasyon Hedefi Gönderme

### RViz2 üzerinden (önerilen)
1. RViz2'de **"2D Goal Pose"** aracını seç (toolbar)
2. Harita üzerinde hedefe tıkla ve yönü belirlemek için sürükle
3. Varsayılan olarak `robot_1`'e gönderir
4. Farklı robot için: Tool Properties panelinde topic'i `/robot_2/goal_pose` yap

### Komut satırından
```bash
# robot_1'i (-10, 7) konumuna gönder
ros2 action send_goal /robot_1/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: -10.0, y: 7.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

### Navigasyonu iptal et
```bash
ros2 action cancel /robot_1/navigate_to_pose
```

---

## Tek Robot Spawn (Test için)

```bash
# Gazebo çalışırken tek robot ekle
ros2 launch warehouse_gazebo spawn_single_robot.launch.py \
  robot_name:=robot_1 x:=-13.0 y:=7.0 yaw:=0.0
```

---

## Debug Komutları

### Sistem durumu kontrol
```bash
# Çalışan node'ları listele
ros2 node list | grep robot_1

# Tüm topic'leri listele
ros2 topic list | grep robot_1

# Nav2 lifecycle durumu
ros2 lifecycle get /robot_1/amcl
ros2 lifecycle get /robot_1/controller_server
```

### AMCL (lokalizasyon) kontrol
```bash
# Robot_1'in tahmin edilen pozisyonu
ros2 topic echo /robot_1/amcl_pose --once

# AMCL partikülleri (yayın var mı?)
ros2 topic hz /robot_1/particlecloud
```

### TF (koordinat dönüşümleri) kontrol
```bash
# map → robot_1/base_link zinciri var mı?
ros2 run tf2_ros tf2_echo map robot_1/base_link

# Tüm frame'leri göster (frames.pdf oluşturur)
ros2 run tf2_tools view_frames
```

### Costmap kontrol
```bash
# Local costmap yayın var mı?
ros2 topic hz /robot_1/local_costmap/costmap

# Scan verisi geliyor mu?
ros2 topic hz /robot_1/scan
```

### Navigasyon kontrol
```bash
# Plan yayınlanıyor mu?
ros2 topic hz /robot_1/plan

# Velocity komutu gidiyor mu?
ros2 topic echo /robot_1/cmd_vel --once
```

---

## Robot Başlangıç Pozisyonları

| Robot   | X     | Y    | Yaw (rad) | Konum    |
|---------|-------|------|-----------|----------|
| robot_1 | -13.0 | +7.0 | 0.0       | Sol üst  |
| robot_2 | +13.0 | +7.0 | π (3.14)  | Sağ üst  |
| robot_3 | -13.0 | -7.0 | 0.0       | Sol alt  |
| robot_4 | +13.0 | -7.0 | π (3.14)  | Sağ alt  |

---

## RViz2 Display'leri

| Display              | Topic                            | Açıklama                   |
|----------------------|----------------------------------|----------------------------|
| Map                  | `/robot_1/map`                   | Depo haritası              |
| LaserScan robot_N    | `/robot_N/scan`                  | Lidar taraması (renk kodu) |
| Path robot_N         | `/robot_N/plan`                  | Planlanan yol              |
| LocalCostmap robot_N | `/robot_N/local_costmap/costmap` | Yakın engel haritası       |
| Particles robot_N    | `/robot_N/particlecloud`         | AMCL partikülleri          |
| RobotModel robot_N   | `/robot_N/robot_description`     | 3D robot modeli            |

**Renk kodlaması:** robot_1=kırmızı, robot_2=yeşil, robot_3=mavi, robot_4=sarı

---

## Önemli Config Dosyaları

| Dosya                                               | Açıklama                          |
|-----------------------------------------------------|-----------------------------------|
| `warehouse_navigation/config/nav2_params.yaml`      | Nav2 parametreleri (tüm robotlar) |
| `warehouse_navigation/config/slam_params.yaml`      | SLAM Toolbox parametreleri        |
| `warehouse_navigation/config/rviz_multi_robot.rviz` | RViz2 görünüm ayarları            |
| `warehouse_navigation/maps/warehouse_map.yaml`      | Mevcut depo haritası              |
| `warehouse_gazebo/worlds/warehouse.world`           | Gazebo dünya dosyası              |

---

## Sık Karşılaşılan Sorunlar

**Nav2 başlamıyor / lifecycle timeout:**
- Gazebo'nun tamamen yüklendiğinden emin ol
- `ros2 topic hz /clock` çıktısı olmalı (sim clock çalışıyor mu?)

**AMCL lokalize olamıyor (partiküller saçılmış):**
- RViz'de "2D Pose Estimate" ile robotun gerçek konumunu işaretle
- `ros2 topic echo /robot_1/amcl_pose --once` ile pozisyonu kontrol et

**Robot hedefe gitmiyor:**
- `ros2 topic echo /robot_1/cmd_vel --once` — velocity komutu geliyor mu?
- `ros2 topic hz /robot_1/plan` — plan üretiliyor mu?
- Global costmap'te başlangıç/hedef nokta engel içinde olabilir

**RViz'de harita görünmüyor:**
- Fixed Frame `map` olmalı
- TF relay çalışıyor mu: `ros2 node list | grep tf_relay`
- `ros2 topic hz /robot_1/map` ile harita yayınlanıyor mu kontrol et

**Costmap boş görünüyor (beyaz):**
- `ros2 topic hz /robot_1/scan` — lidar veri gönderiyor mu?
- `ros2 topic info /robot_1/local_costmap/costmap --verbose` ile QoS uyuşmazlığı var mı kontrol et
