# Audit Metode SLAM MID-360S — presisi dan efisiensi CPU/RAM (2026-10-08)

Workspace implementasi: `/media/sirobo/HDD/slam3d-livox-mid360s`
Korpus referensi: `/media/sirobo/HDD/Referensi-slam3d` (150 repository ber-Git ditemukan, tersebar di beberapa kategori). Audit ini membandingkan referensi yang paling langsung relevan dengan **1× MID-360S + IMU internal** dan sistem ROS2 Humble yang sudah berfungsi; bukan klaim seluruh 150 repo sudah benchmark penuh.

## Keputusan arsitektur

Dengan **hanya LiDAR + IMU internal**, gunakan:
1. `livox_ros_driver2` xfer_format=0, raw `/livox/lidar` PointCloud2 bertimestamp per titik, `/livox/imu` (acceleration g, angular velocity rad/s).
2. `fast_lio` P3 menggunakan error-state iterated Kalman + iKD-tree, point-time deskew dan local LIO odometry. Jangan membuang field timestamp sebelum estimator.
3. `slam3d_pose_graph` P4 untuk keyframe, *Scan Context-style* descriptor candidate, ICP geometric verification (fitness **dan sekarang overlap**), GTSAM/iSAM2 robust Huber loop factors, map cloud mengikuti pose global optimized, penyimpanan PCD+CSV.
4. `slam3d_mid360_dashboard` dan Foxglove Bridge menjadi viewer diagnostics dan ekspor; **bukan** estimator/motion-control.
5. Benchmark ulang source P3/P4 pada ROS_DOMAIN_ID khusus; jangan memperlakukan fixture sebagai pengukuran ketelitian sensor fisik.

Skema: `LiDAR + IMU -> P3 FAST-LIO /lio/odometry + /lio/cloud_registered -> P4 iSAM2 loop closure /p4/optimized_map + optimized_path -> PCD/CSV/visualisasi`.

Jangan mengganti FAST-LIO dengan repo algoritma yang lebih berat secara otomatis; jumlah bintang GitHub bukan bukti kecocokan timing, sensor frame, IMU unit, atau kompilasi ROS2.

## Hasil pembacaan referensi terpilih

| Referensi lokal | Aspek yang berguna | Keputusan |
|---|---|---|
| `02_LIDAR_INERTIAL_LIO/FAST_LIO`, `FAST_LIO_SLAM_Livox` | IEKF, ikd-Tree, LiDAR + IMU solid-state, deskew | Dipakai sebagai P3 dengan ROS2 MID-360S timestamp parser khusus |
| `02_LIDAR_INERTIAL_LIO/faster-lio` | iVox/hash voxel untuk mempercepat map lookup; contoh `mid360.yaml` | Kandidat P3 tahap lanjut jika profil nyata menunjukkan CPU terbatas. Jangan mengganti ROS2 parser langsung |
| `02_LIDAR_INERTIAL_LIO/Point-LIO` | Estimasi LiDAR–IMU pada motion agresif | Pembanding riset setelah port ROS2/per-point timing tervalidasi |
| `02_LIDAR_INERTIAL_LIO/FAST_LIO_SAM_ROS2`, `FAST_LIO_LC` | LIO + GTSAM/loop closure | Ide pemisahan LIO real-time dari peta global di P4 |
| `02_LIDAR_INERTIAL_LIO/LIO-SAM_MID360_ROS2` | MID360 ROS2 LIO-SAM | README menyebut repo bukan paket mandiri, butuh paket induk; bukan drop-in replacement |
| `07_LOOP_CLOSURE_PLACE_RECOGNITION/Fast-LIO2_SC_EGO_ROS2`, Scan Context | Penemuan ulang lokasi dan loop closure dengan LiDAR non-repetitive MID360 | P4 menggunakan kandidat descriptor dengan validasi ICP fitness+overlap dan pagar koreksi |
| `06_GRAPH_OPTIMIZATION_BACKEND/gtsam` | iSAM2 incremental pose graph, robust factors | Sudah dipakai; pembatas Intel oneTBB ditambahkan untuk efisiensi |
| `10_CALIBRATION_INITIALIZATION/LiDAR_IMU_Init` | Kalibrasi offset waktu dan LiDAR-to-IMU extrinsic tanpa kamera/GNSS | Konsep disarankan, tetapi upstream ROS1/CustomMsg belum aman digunakan langsung untuk PointCloud2 ROS2 |
| `01_LIDAR_ODOMETRY_SLAM/kiss-icp` | LiDAR-only baseline dan evaluasi drift tanpa IMU | Sudah tersedia sebagai pembanding P2 |
| `08_SEMANTIC_DYNAMIC_SCENE/ERASOR`, `removert` | Filtering objek dinamis untuk map statis | Kandidat offline; jangan hilangkan data mentah sebelum P3 tanpa uji |
| `02_LIDAR_INERTIAL_LIO/glim_ros2` | Optimisasi faktor dan dense mapping lebih advanced | Kandidat riset A/B di komputer dengan RAM besar, bukan otomatis lebih ringan |

**Catatan kalibrasi penting:** file contoh Faster-LIO MID360 menggunakan `extrinsic_T=[-0.011,-0.02329,0.04412]`, sedangkan referensi FAST_LIO_SLAM_Livox menyebut nilai yang lain. Itu bukan kalibrasi untuk unit MID-360S serial milik pengguna. P3 saat ini identitas/0 sebagai baseline, `config/extrinsics.yaml` (base_link→livox_frame) juga tetap `uncalibrated`. Kedua jenis extrinsic tersebut **berbeda** dan tidak boleh dicampur.

## Audit kelemahan yang ditemukan dan diimplementasikan

1. **FAST-LIO P3 akumulasi peta tak terbatas:** opsi `publish.map_en: true` sebelumnya menggabungkan point cloud setiap 1 detik tanpa batas, meskipun P4 juga membuat corrected map. Di konfigurasi utama dan detail, `publish.map_en: false` kini default. Jika map P3 debug diaktifkan, buffer dibatasi `publish.max_debug_map_points: 120000`; interval preview 5 s dan perhitungan diabaikan bila tak ada subscriber. Gunakan `/p4/optimized_map` sebagai authoritative corrected map.
2. **P4 map cap belum global:** sampling lama mereset stride pada tiap keyframe, sehingga jumlah titik sementara dapat melewati plafon. Kini menggunakan indeks global, anggaran `max_map_points` berlaku untuk gabungan semua keyframe. PCD tetap divoxelize sebelum publikasi.
3. **P4 loop ICP kurang pemeriksaan geometrik:** setelah ICP konvergen, kini menghitung proporsi titik yang mempunyai tetangga target dalam jarak terukur, `loop_min_overlap_ratio=0.30` dan `loop_overlap_distance_m=0.65`. Selain fitness, correction translation/rotation, descriptor dan minimum jarak waktu, loop harus memenuhi overlap. Nilai ini konservatif **candidate** untuk uji lapangan, bukan universal optimum.
4. **P4 memproses peta terlampau sering:** global map kini hanya diregenerasi jika sedikitnya `map_publish_min_new_keyframes=3` sejak pembuatan terakhir atau saat loop baru diterima. Service `/p4/save_map` tetap memaksa pembuatan snapshot saat diminta.
5. **Thread explosion pada GTSAM oneTBB:** `OMP_NUM_THREADS=4` saja tidak cukup. `tbb::global_control(max_allowed_parallelism=4)` sekarang hidup selama node P4 aktif. FAST-LIO dan P4 launch mengatur OMP=4 dan BLAS=1.
6. **Kapasitas keyframe:** batas `max_keyframes=1500` adalah batas sesi saat ini. Saat penuh, P4 menolak keyframe baru; diagnostik sekarang harus berstatus ERROR agar pemetaan berhenti tidak tersamarkan sebagai map sehat. Perlu potong sesi/submap untuk lintasan yang melewati batas, jangan mengklaim map otomatis tak terbatas.
7. **Monitoring:** /diagnostics P4 ditambah jumlah titik input/sampled/final, waktu build map, rasio overlap dan waktu perhitungan, serta jumlah loop ditolak karena overlap. Dashboard Foxglove tetap menampilkan data raw/LIO/P4 secara terpisah.

## Hasil uji yang dapat diulang

**Lingkungan:** RDC Xeon E5-2680v4 (56 logical CPU), RAM 62 GiB; ROS2 Humble, PCL 1.12 ROS ABI konsisten, GTSAM+iSAM2 dan oneTBB.

**Fixture P4 lintasan tertutup:** 60 keyframe, 1 loop closure terverifikasi, PCD tersimpan dan proses exit 0 sebelum dan sesudah optimasi.
- Profil sebelum TBB limiter: 66 thread, RSS sampel sekitar 63–75 MB, CPU proses sampel sekitar 9–13% pada interval tengah.
- Profil setelah TBB limiter=4: **14 thread**, RSS sekitar 60–67 MB, CPU proses sekitar 4–7% pada interval sampel tengah. Hasil ini pada fixture; generalisasi untuk rute yang sangat besar membutuhkan benchmark terpisah.
- Uji map global cap=5000 dengan voxel 0.10 m: output PCD **4.645 titik**, di bawah batas; 1 loop valid, kedua proses exit 0. Folder: `results/benchmark/map_cap_5000`.

**MCAP asli MID-360S, hampir diam:** `datasets/live_mid360s/first_live_20261008_1450` (74 clouds + 1496 IMU, ±7,48 detik). Setelah optimasi, P3/P4 replay keluar normal: P3 **71 pose**, 10 Hz selama stream, tidak ada jumps/timestamp mundur; P4 menghasilkan 1 keyframe, 0 loop. Map disimpan ke `results/benchmark/live_bag_after_optimizer_20261008/maps`.
- Profil replay: FAST-LIO kira-kira **148–153 MB RSS**, P4 **58–60 MB RSS**. Jumlah thread P3 ~13, P4 ~12–15.
- FAST-LIO masih menghasilkan akumulasi lintasan **~0,28 m** dan rentang Z **~0,041 m** selama rekaman sensor hampir diam. Ini menunjukkan kebutuhan penyetelan/kalibrasi dan **bukan** presisi sentimeter yang dapat diklaim.
- Basis IMU: `results/calibration/imu_baseline_bag_20261008.yaml`, 1029 sampel, norma akselerasi 0,989984 g, gyro 0,009111 rad/s. Itu hasil statistik IMU diam, bukan penyelesaian LiDAR/IMU extrinsic.

## Yang belum bisa dibuktikan dengan sensor mati / data pendek

Pada pemeriksaan jaringan terakhir, MID360S 192.168.1.197 belum merespons ARP/ping dan tidak menerbitkan stream baru. **Belum ada data gerak berulang yang mencakup loop nyata**; akurasi geometri, error translasi absolut dan robustness multipath belum terukur.

Untuk membuktikan presisi fisik dengan hanya sensor ini:
1. Nyalakan PSU dan verifikasi kedua topic `/livox/lidar` 10 Hz dan `/livox/imu` 200 Hz.
2. Rekam IMU diam minimal 15 s, cek gyro bias, acceleration noise, dan temperatur. Jangan menormalisasi data dua kali: Livox IMU accelerometer menggunakan *g* dan FAST-LIO internally menormalisasi mean.
3. Kalibrasi offset waktu dan **LiDAR→IMU** pada gerakan translasi/rotasi 3D yang cukup; LI-Init diperlukan port/adapter ROS2 sebelum dapat dijadikan pipeline otomatis.
4. Ukur terpisah **base_link→livox_frame**, tinggi sensor dan roll/pitch/yaw pemasangan.
5. Rekam beberapa rute 30–120 s indoor dengan overlap cukup, radius/geometry terukur, kembali ke titik awal, termasuk koridor berulang dan objek bergerak.
6. Replay baseline vs detail dan (opsional) Faster-LIO port dengan dataset sama: bandingkan trajectory vs jarak fisik, net stationary drift, Z span, loop accepted/rejected, point overlap, PCD planar wall thickness dan wall/floor alignment, CPU single-core, RSS P95/peak, odom jitter, dropped frames, map build latency, serta keselamatan saat tracking hilang.
7. Lakukan ekspor PCD dan bandingkan ukuran fisik (pita ukur atau survei sederhana) untuk mengukur error dimensi. Tanpa ground truth eksternal tak mungkin menyatakan presisi absolut dari SLAM sendiri.
8. Jangan gunakan hasil `/p4/optimized_map` untuk Nav2 AGV sampai kalibrasi, loop validation, E-STOP hardware dan navigasi geometri fisik disetujui.

**Kesimpulan:** implementasi sensor-only sudah konsisten secara arsitektur, pemborosan CPU/RAM utama telah dikurangi dan diuji; ketelitian fisik masih **belum terbukti**. Catat nilai CPU/RAM di atas sebagai hasil pada fixture/replay, bukan janji produksi.
