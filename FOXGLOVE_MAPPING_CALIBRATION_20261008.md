# Foxglove + Livox MID-360S: kalibrasi, pemetaan 3D, navigasi tampilan (audit 2026-10-08)

Workspace: `/media/sirobo/HDD/slam3d-livox-mid360s`. ROS2 Humble, Ubuntu 22.04, MID-360S lewat `enp4s0`.

## Aplikasi yang berbeda

1. **Foxglove resmi**: Foxglove Bridge ROS2 (`ros-humble-foxglove-bridge` 3.5.0) read-only di `127.0.0.1:8765`, layanan `mid360-foxglove.service`. Sambungkan dari Foxglove dengan **Foxglove WebSocket** di `ws://localhost:8765`.
2. **Dashboard internal**: C++ ROS2 + HTTP canvas di `http://localhost:5001/`, layanan `mid360-dashboard.service`. Ini **bukan** Foxglove asli. Kini dapat drag pan, orbit, zoom, source-select.
3. **RViz2**: viewer ROS2 desktop terpisah.

Dari PC Windows ke sirobo, SSH tunneling:

```bash
ssh -L 8765:127.0.0.1:8765 -L 5001:127.0.0.1:5001 sirobo
```

Lalu Foxglove desktop/web di Windows dapat terhubung `ws://localhost:8765`; browser dashboard `http://localhost:5001`. Koneksi SSH perlu tetap aktif. Jangan mengekspos WebSocket 8765 secara publik tanpa autentikasi/TLS. Layanan default hanya bind loopback dan dibatasi ke **read-only**; tidak dapat teleoperate, memanggil service atau mengubah parameter.

## Foxglove import layout

`foxglove/layouts/MID360S_mapping_calibration.json`

Di Foxglove: **Layouts → Import from file**; pilih JSON. Source sudah dicocokkan dengan format layout pada referensi lokal `husarion--foxglove-docker` dan `foxglove--diy-dvr-panel`. Layout memuat panel 3D, plot IMU akselerasi, plot gyro, Raw Messages diagnostik dan odometri. **Belum diuji impor ke akun Foxglove pengguna**; kompatibilitas UI mungkin berubah antarversi.

Sumber data yang direkomendasikan:
- `/livox/lidar`: sensor PointCloud2 mentah, frame `livox_frame`.
- `/livox/imu`: sensor_msgs/Imu, akselerasi dalam satuan **g**, angular velocity rad/s.
- `/lio/cloud_registered`: scan terdaftar oleh FAST-LIO P3, frame `lio_odom`.
- `/lio/odometry`, `/lio/path`: estimasi odometri lokal.
- `/p4/optimized_map`, `/p4/optimized_path`: peta/lintasan dikoreksi GTSAM P4, frame `map`.
- `/mid360/map_preview`: preview peta P4 ter-voxelize, bukan peta resolusi penuh.
- `/diagnostics`, `/tf`: kesehatan sistem dan transform.

Jangan memperlihatkan PointCloud2 dari beberapa frame sekaligus tanpa transform TF yang valid. Bila P4 belum menerbitkan `map`, ubah Display Frame ke `livox_frame` untuk data mentah atau `lio_odom` untuk FAST-LIO. P4 dan FAST-LIO tidak boleh bersaing menerbitkan global TF.

## Kontrol 3D

**Foxglove resmi**: drag biasa = pan ground plane; Alt+drag = pan bidang layar dengan vertikal; Shift+drag atau klik kanan+drag = orbit/rotasi; scroll = zoom; tombol 1 = reset kamera; W/A/S/D menggeser; 3 = toggle 2D/3D. Sumber: https://docs.foxglove.dev/docs/visualization/panels/3d

**Dashboard 5001**: drag biasa = pan; Shift+drag atau klik kanan+drag = rotasi; tombol **Drag: Orbit** mengubah default; wheel = zoom; **Fit/Reset** mengatur skala/bounding box; **Top View** untuk overhead; pilihan layer P4/FAST-LIO/LiDAR raw. Pan/orbit diuji dengan browser Chrome otomatis (PASS).

## Kalibrasi dan kesehatan IMU

File status yang tetap benar (belum terukur secara fisik):

`config/extrinsics.yaml` → `status: uncalibrated`, transform `base_link -> livox_frame` sementara identity/zero.

Tidak boleh otomatis mengubah status menjadi calibrated hanya karena LiDAR, RViz, Foxglove atau FAST-LIO terlihat berjalan.

**Baseline dari MCAP sensor nyata**: `results/calibration/imu_baseline_bag_20261008.yaml`. Baseline 1.029 sampel/5,09 detik memenuhi tes sensor diam, mean |acc| 0,989984 g dan gyro 0,009111 rad/s, tetapi belum cukup untuk kalibrasi multi-orientasi, temporal offset, mounting extrinsic, atau evaluasi suhu jangka panjang.

**Kalibrasi ulang IMU dengan sensor hidup dan benar-benar diam selama ≥15 detik**:

```bash
cd /media/sirobo/HDD/slam3d-livox-mid360s
source env.sh
tools/calibrate_mid360_imu.sh --confirm-stationary
```

Script menghasilkan YAML baru dengan gyro mean/std, acc mean/std, dan status stationary; gagal bila data atau ambang tidak terpenuhi. Tidak menimpa `config/extrinsics.yaml`.

**Extrinsic fisik (wajib)**: ukur koordinat posisi [x,y,z] dan orientasi roll/pitch/yaw sensor relatif terhadap pusat `base_link` robot dengan konvensi `base_link -> livox_frame`, lalu validasi terhadap geometri dinding/permukaan datar dan gaya gravitasi. LI-Init dari referensi awal adalah ROS1/custom-message; jangan menjalankannya tanpa port ROS2 PointCloud2 yang diverifikasi. Sensor MID-360S memiliki IMU internal, tetapi base_link tetap perlu diukur.

**Timestamp nyata**: ROS2 PointCloud2 mempunyai field `timestamp` FLOAT64 nanodetik absolut pada tiap titik. Cuplikan cloud dalam MCAP mencakup ±100 ms per frame; header cloud mengacu waktu titik awal, sedangkan penerimaan rosbag sekitar 103–104 ms lebih lambat karena akumulasi scan. IMU diterima sekitar 0,1–0,3 ms setelah timestampnya. Selisih penerimaan cloud ini **bukan** bukti bahwa sensor LiDAR vs IMU memiliki offset waktu fisik 100 ms; jangan menggeser `time_offset_lidar_to_imu` tanpa audit sinkronisasi jam.

## Arsitektur pemetaan yang tepat

`/livox/lidar` **mentah dengan field waktu tiap titik** + `/livox/imu` → FAST-LIO ROS2 → `/lio/odometry` + `/lio/cloud_registered` → P4 descriptor/ICP/GTSAM iSAM2 → `/p4/optimized_map` → `/mid360/map_preview` → 5001/Foxglove.

Jangan memasukkan `/mid360/points_filtered` sebagai input FAST-LIO karena voxelization membuang timing per-point, sehingga deskew menjadi salah.

P4 mengoptimasi lintasan dengan loop closure jika observasi lintasan kembali lagi, bukan membangun peta lingkungan utuh dari LiDAR statis. Kualitas yang perlu diuji: drift saat berhenti, jumlah keyframe, candidate loop vs yang diterima/dibatalkan, kesesuaian ICP, 3D wall/floor alignment, ketebalan dinding, overlap pada revisits, jump ketika loop ditutup, kemampuan replay menghasilkan peta yang sama, dan performa CPU/RAM.

**Profil parameter** (tuning kandidat, bukan klaim terverifikasi):
- Baseline: `config/stage3_fast_lio.yaml`, `src/slam3d_pose_graph/config/stage4.yaml`.
- Kandidat detail indoor: `config/profiles/fast_lio_indoor_detail.yaml`, `config/profiles/p4_indoor_detail.yaml`. Lebih banyak titik/keyframe, voxel peta lebih kecil. Memakai CPU/memori dan menambah peluang matching ambigu; kalibrasi dan uji A/B tetap wajib.

Cara beralih ke detail indoor tanpa menjalankan dua odometri/graf:

```bash
systemctl --user stop mid360-dashboard.service
cd /media/sirobo/HDD/slam3d-livox-mid360s
tools/run_mid360_detail_mapping.sh
# Setelah pengujian selesai:
# tekan Ctrl+C lalu:
systemctl --user start mid360-dashboard.service
```

Script menolak bila layanan baseline masih aktif atau stream sensor LiDAR/IMU belum tersedia; tidak mengaktifkan motor. Detail map disimpan di folder `results/detail_mapping_<timestamp>/maps`.

Untuk menghasilkan **peta nyata yang layak diuji**, sensor harus dibawa bergerak perlahan dan stabil melewati lintasan yang memiliki overlap: lihat fitur pada beberapa sudut, hindari rotasi di tempat terus-menerus, kembali ke koridor atau objek awal, lalu tunggu P4 menerima loop verifikasi ICP. Jangan mencampur peta pengujian statis dengan peta untuk navigasi.

## Pengujian data nyata per 2026-10-08

Rekaman `datasets/live_mid360s/first_live_20261008_1450` berisi 74 point clouds dan 1496 pesan IMU selama ~7,48 detik. Ini dataset hampir **statis**, tidak ada lintasan loop.

FAST-LIO replay baseline: 71 pose, 0 nonmonotonik, 0 jump, akumulasi lintasan 0,27638 m, displacement 0,04307 m dan rentang Z 0,04694 m; P4 menghasilkan satu keyframe, 0 loop closure, PCD baseline hanya 438 poin setelah downsampling.

File uji ini sengaja ditempatkan terpisah:
`results/live_mid360s/stationary_bag_test/`.

Peta berisi satu keyframe bukan peta SLAM final; jangan gunakan untuk P5 relocalization atau Nav2. Uji replay profil detail tersedia di `results/live_mid360s/detail_profile_bag_test/maps` dan harus diperiksa per hasil.

**Status terakhir sensor**: pada pemeriksaan 18:15–18:17, IP 192.168.1.197 tidak menjawab ping dan topic live 0 Hz. Driver process yang ada tidak sama dengan sensor powered on. Browser dashboard benar menampilkan OFFLINE. Untuk mapping bergerak sekarang, nyalakan PSU dan verifikasi stream lagi.

## Referensi yang dipakai

- Source ROS2 `FAST_LIO_SLAM_Livox` untuk FAST-LIO, per-point timing dan IKD-Tree; `FAST_LIO_SAM_ROS2` dan `FAST_LIO_LC` untuk integrasi GTSAM/loop matching.
- `FAST_LIO_SLAM/SC-PGO` sebagai referensi konsep Scan Context, bukan dipasang ROS1 langsung ke ROS2.
- `kiss-icp` sebagai baseline LiDAR-only dan kontrol pembanding.
- `foxglove--foxglove-sdk` dan `foxglove--ros-foxglove-bridge` untuk bridge; layout JSON mengadaptasi struktur layout dari referensi lokal Husarion dan DIY DVR.
- Penjelasan resmi pan/orbit: https://docs.foxglove.dev/docs/visualization/panels/3d
- Pengaturan koneksi ROS2 Foxglove: https://docs.foxglove.dev/docs/getting-started/frameworks/ros2

**Acceptance**: Web UI, pan/orbit, P3 replay dari hardware dan P4 export, IMU baseline, Foxglove WebSocket, source separation: OK. Peta lingkungan 3D dengan akurasi fisik dan loop closure pada rute nyata: **PENDING**.

## Hasil A/B parameter dan regression terakhir

- Baseline P3 + P4 dari MCAP sensor hampir diam: 1 keyframe, 0 loop; PCD 438 titik, path drift terakumulasi 0,276m (71 pose).
- Profil high-detail dari MCAP yang sama: 1 keyframe, 0 loop; PCD 820 titik (peningkatan kepadatan ~87%). Ini bukan bukti akurasi yang meningkat; belum ada lintasan bergerak/ground truth atau loop closure.
- Fixture P4 terpisah domain ROS2 198 dan port 5002: 60 keyframe, 1 loop terverifikasi, 0 ditolak, 10.531 titik map terkoreksi; GET /api/points?source=p4 menampilkan 2.633 titik frame map; GET /api/points?source=lio menampilkan 2.414 titik frame lio_odom. Semua fixture berhenti normal dengan exit 0.
- Dashboard 5001 dan Foxglove Bridge 8765 tetap aktif; port fixture 5002 dibebaskan.
- Kalibrasi base_link ke livox_frame masih UNCALIBRATED dan LiDAR belum menghasilkan stream baru pada pengecekan terakhir. Jangan gunakan map fixture atau stationary map untuk navigasi fisik.
