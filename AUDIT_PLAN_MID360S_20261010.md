# Audit bug dan rencana percepatan SLAM 3D Livox MID-360S

Tanggal audit: 10 Oktober 2026 (UTC+7).
Proyek: /media/sirobo/HDD/Project/slam3d-livox-mid360s
Perangkat: Sirobo, diakses melalui RDC sirobo-rdc2.
Jenis pekerjaan: audit kode, inspeksi runtime baca-saja, dan rencana perbaikan. Tidak ada patch kode produksi, restart layanan, pengubahan konfigurasi sensor, atau perintah motor dalam audit ini.

## 1. Kesimpulan

Prioritas pertama adalah memperbaiki jalur instalasi yang rusak setelah proyek berpindah folder, kemudian memulihkan aliran sensor dan memperbaiki waktu/TF/covariance. Menaikkan publish_freq sebelum itu tidak menyelesaikan masalah utama.

Ada tiga sumber kelambatan yang berbeda:
1. Saat audit, input LiDAR dan IMU benar-benar tidak mengirim pesan.
2. Jalur komputasi memiliki pekerjaan berulang, logging, antrean tanpa batas, serta bagian OpenMP yang tidak diaktifkan.
3. Tampilan web mengambil data jauh lebih lambat daripada konfigurasi LiDAR: sekitar 0,56 Hz untuk live/LIO dan 0,28 Hz untuk peta P4 dalam kondisi permintaan HTTP cepat.

Target pengembangan yang masuk akal adalah 10 Hz yang stabil lebih dulu, lalu menguji 20 Hz dengan latency terukur dan kualitas peta tetap terjaga. Dukungan publikasi driver sampai 100 Hz bukan bukti bahwa SLAM akan berjalan 100 Hz atau sensor menghasilkan lebih banyak titik per detik.

## 2. Bukti runtime

Pengamatan topic dilakukan dengan subscriber sementara, tanpa memublikasikan data uji ke domain robot. Durasi pengamatan berhasil: 15,024 detik.

| Topic | Pesan diterima | Frekuensi teramati | Publisher ditemukan |
|---|---:|---:|---:|
| /livox/lidar | 0 | 0 Hz | 1 |
| /livox/imu | 0 | 0 Hz | 1 |
| /lio/odometry | 0 | 0 Hz | 1 |
| /lio/cloud_registered | 0 | 0 Hz | 1 |
| /p4/optimized_odometry | 0 | 0 Hz | 1 |
| /p4/optimized_map | 0 | 0 Hz | 1 |

Bukti pendukung:
- /diagnostics: Livox PointCloud dan IMU berstatus ERROR/no data; expected_cloud_hz=10 dan expected_imu_hz=200.
- P4: keyframes=0, loops=0, menunggu odometri dan cloud yang tersinkron.
- GET http://127.0.0.1:5001/api/status: sensor_online=false, imu_online=false, cloud_messages=0, imu_messages=0, extrinsics_status=uncalibrated.
- enp4s0: link UP, carrier=1, speed=100 Mb/s; host memiliki 192.168.1.10/24.
- Neighbor 192.168.1.197: FAILED. Ini menunjukkan resolusi tetangga ke IP konfigurasi gagal; belum membuktikan penyebabnya pasti sensor mati. Periksa daya, sambungan/switch/VLAN, IP aktual, dan pengiriman UDP.
- Port Ethernet 100 Mb/s sesuai spesifikasi MID-360S; angka tersebut sendiri bukan indikasi bottleneck.
- Perangkat yang teramati: 2 x Xeon E5-2680 v4, 28 core fisik/56 logical CPU, RAM sekitar 62 GiB. Ketersediaan banyak core tidak membuat setiap algoritme otomatis paralel.
- Proses yang berjalan masih menampilkan argumen path lama. /proc/PID/exe menunjuk binary yang kini berada di folder Project. Proses lama dapat bertahan setelah perpindahan folder, tetapi itu tidak membuktikan service bisa di-start ulang.
- ./build.sh --help gagal dengan exit 127 karena target wrapper tidak ada.
- source env.sh menulis error file tidak ada, tetapi exit 0 dan tetap mencetak pesan sukses.
- Pemeriksaan AST 38 file Python launch/tools berhasil. Ini hanya pemeriksaan sintaks, bukan bukti launch berhasil.
- Delapan CMakeLists paket tidak memperlihatkan registrasi CTest/ament test melalui pemeriksaan add_test/ament_add/BUILD_TESTING. Ada fixture/synthetic launch; jangan menyimpulkan proyek sama sekali tidak memiliki pengujian.

## 3. Cakupan dan kekuatan bukti

Kode inti delapan paket diperiksa: livox_ros_driver2, livox_mid360_manager, fast_lio, kiss_icp, slam3d_pose_graph, slam3d_relocalizer, slam3d_nav2_bridge, dan slam3d_mid360_dashboard. Juga diperiksa wrapper, konfigurasi, launch, service pengguna, flag build, halaman web, dan server tuning.

Snapshot analisis memuat 43 file/9.466 baris; beberapa dokumen, konfigurasi, service dan launch dibaca langsung di luar snapshot itu. Daftar file beserta hash ada di lampiran. Kode vendor besar seperti seluruh Eigen/PCL/GTSAM/IKFoM/ikd-Tree dan backend riset external tidak diaudit baris demi baris.

Status bukti:
- RUNTIME: diuji atau diobservasi langsung pada sesi ini.
- STATIS: jalur kode yang bermasalah terlihat jelas, tetapi skenario pemicunya belum semuanya dijalankan.
- KONDISIONAL: cacat muncul ketika mode, input, atau parameter tertentu dipakai.
- OPTIMASI: pekerjaan yang bisa dikurangi; besarnya kenaikan performa belum diukur.

Karena sensor tidak mengalir, laporan ini tidak mengklaim benchmark live, akurasi lapangan, semua bug sudah ditemukan, atau sistem sudah bebas bug.

Semua referensi file relatif berikut berakar pada folder proyek di atas. Nomor baris merujuk snapshot audit.

## 4. Temuan bug dan robustness

### F01 — Jalur build, environment, service, dan tools tidak mengikuti perpindahan folder
Prioritas: P0. Bukti: RUNTIME + STATIS.

env.sh baris 2 memuat /media/sirobo/HDD/pixhawk/tools/env_unified.sh, dan build.sh baris 3 menjalankan build_unified.sh di lokasi itu. Direktori tersebut tidak ada. tools/run_mid360_dashboard.sh baris 3 dan service mid360-dashboard.service/mid360-foxglove.service memakai /media/sirobo/HDD/slam3d-livox-mid360s, yang juga tidak ada. Tools kalibrasi, Foxglove dan tuning masih memiliki root lama.

Dampak: terminal baru tidak mendapat environment yang benar; build gagal; restart/reboot berisiko menghentikan layanan yang saat ini masih bertahan. env.sh memberi sukses palsu karena echo terakhir mengembalikan exit 0.

Perbaikan: satu root yang diturunkan dari lokasi skrip atau konfigurasi eksplisit; tentukan sumber workspace gabungan yang benar; gagal secara eksplisit jika dependency tidak ada. Regenerasi service dan install pada path baru. Jangan membuat symlink untuk menutupi semua referensi tanpa menyelesaikan sumber konfigurasi.
Uji: source pada shell bersih, resolusi ros2 pkg prefix, build bersih, pemeriksaan ExecStart, lalu restart terencana dan health topic.

### F02 — Buffer statistik FAST-LIO bisa ditulis melewati batas
Prioritas: P0. Bukti: STATIS.

src/fast_lio/src/laserMapping.cpp baris 69–73 menetapkan MAXN=720000. Callback baris 288/305 dan 317/346 menambah scan_count lalu menulis s_plot11[scan_count] tanpa batas, termasuk ketika runtime_pos_log_enable=false. Statistik lain pada 1107–1118 juga tidak memiliki guard ketika logging aktif.

Dampak: undefined behavior/corruption setelah 720.000 cloud diterima: sekitar 20 jam pada 10 Hz, 10 jam pada 20 Hz, dan 4 jam pada 50 Hz. Ini perhitungan ambang, bukan klaim crash live sudah diobservasi.

Perbaikan: ring buffer terbatas atau agregasi statistik online; hindari penyimpanan per-frame saat logging tidak aktif.
Uji: ASan/UBSan dengan penghitung mendekati kapasitas dan replay panjang.

### F03 — Array fitur tetap 100.000 tidak mengikuti ukuran cloud
Prioritas: P1. Bukti: STATIS, KONDISIONAL.

laserMapping.cpp baris 79/98 memakai res_last dan point_selected_surf berukuran 100000; loop pencocokan baris 701 dan 748 mengikuti feats_down_size. Pemeriksaan sekitar 1039 hanya menolak jumlah terlalu kecil. Tidak ada batas atas yang konsisten untuk semua buffer.

Dampak: input besar, akumulasi scan, atau parameter voxel sangat kecil dapat menghasilkan akses melewati array. Normal MID-360S 10 Hz tidak otomatis mencapai ambang ini.
Perbaikan: buffer dinamis yang di-resize bersama, atau penolakan/downsampling eksplisit sebelum akses.
Uji: cloud sintetis dengan lebih dari 100.000 fitur setelah voxel.

### F04 — TF FAST-LIO dikirim tanpa timestamp
Prioritas: P0. Bukti: STATIS.

laserMapping.cpp publish_odometry, baris 660–670: TransformStamped diberi frame dan pose tetapi tidak diberi trans.header.stamp. Timestamp default tetap nol.

Dampak: lookup TF pada waktu sensor dapat gagal atau menghasilkan data stale ketika publish_odom_tf=true. Mode P6 yang mematikan TF FAST-LIO dan menggunakan odom_base_bridge menghindari jalur ini, tetapi mode mapping P3/P4 default masih mengaktifkannya.

Perbaikan: gunakan timestamp odometri yang sama, lalu verifikasi satu publisher per transform.
Uji: TF lio_odom→frame yang benar harus memiliki stamp scan terakhir dan lookup pada waktu cloud berhasil.

### F05 — Covariance odometri tertinggal satu frame dan bloknya tertukar
Prioritas: P1. Bukti: STATIS.

laserMapping.cpp baris 647 memublikasikan odometri sebelum mengisi covariance pada 648–658. Selain itu, use-ikfom.hpp baris 13–14 mendefinisikan urutan pos lalu rot, sementara kode publisher menukar bloknya.

Dampak: pesan pertama memiliki covariance default; pesan berikut memakai covariance sebelumnya; ketidakpastian posisi dan rotasi dipertukarkan. Fusi downstream menerima kualitas pose yang keliru.

Perbaikan: susun covariance dari urutan state yang benar sebelum publish; konversikan perturbasi rotasi ke konvensi covariance ROS secara eksplisit.
Uji: covariance diagonal dengan nilai posisi/rotasi yang berbeda harus muncul pada enam elemen yang benar pada pesan pertama dan selanjutnya.

### F06 — Pose IMU diberi label frame LiDAR
Prioritas: P1. Bukti: STATIS, KONDISIONAL saat extrinsic tidak identity.

laserMapping.cpp set_posestamp memakai state_point.pos/rot, yaitu state IMU. Publisher menamai child livox_frame. Di 1073 posisi LiDAR dihitung sebagai pos_lid tetapi tidak digunakan untuk odometri. publish_frame_body pada 554 mengubah titik LiDAR ke frame IMU, lalu pada 561 menamainya livox_frame.

Dampak: setelah kalibrasi LiDAR–IMU nonzero, pose/cloud/TF tidak lagi merepresentasikan frame yang sama. P4/P5/P6 mengasumsikan odometri adalah pose sensor livox_frame.

Perbaikan: pilih kontrak imu_link atau livox_frame; transformasikan pose, cloud dan covariance secara konsisten. Pisahkan extrinsic internal LiDAR–IMU dari mounting sensor–base_link.
Uji: extrinsic translasi dan rotasi nonzero; cloud dunia harus sama melalui dua jalur transform yang ekuivalen.

### F07 — Reset waktu LiDAR meninggalkan antrean timestamp lama
Prioritas: P0. Bukti: STATIS.

laserMapping.cpp baris 290–294 dan 318–322 membersihkan lidar_buffer ketika timestamp mundur, tetapi tidak membersihkan time_buffer dan lidar_pushed. sync_packages pada 393–394 mengambil cloud dan waktu dari dua antrean tersebut.

Dampak: setelah replay mundur/restart sensor, cloud baru dapat dipasangkan dengan waktu lama; pending scan dan riwayat IMU/filter juga bisa berasal dari epoch sebelumnya.

Perbaikan: transaksi reset untuk seluruh pasangan cloud–timestamp, pending measurement, riwayat IMU, estimator dan state publikasi; tangani clock jump eksplisit.
Uji: urutan waktu 100, 101, lalu 1, 2; tidak ada pemasangan lintas epoch.

### F08 — Antrean internal tidak memiliki batas umur/kapasitas yang memadai
Prioritas: P1. Bukti: STATIS.

FAST-LIO menambahkan lidar_buffer/time_buffer/imu_buffer tanpa limit di 302–303, 343–344, 377. Driver juga menambahkan raw_packet_queue_ di pub_handler.cpp:144 dan antrean IMU pada lidar_imu_data_queue.cpp:48 tanpa limit.

Dampak: bila salah satu stream berhenti atau pemrosesan tertinggal, RAM/latency dapat terus bertambah. QoS depth ROS tidak membatasi deque yang sudah disalin ke dalam aplikasi.

Perbaikan: kapasitas dan batas usia eksplisit, counter overflow, watchdog stream, serta reset/recovery yang memahami kontinuitas IMU. Jangan membuang sampel IMU sembarangan saat scan masih membutuhkan interval tersebut.
Uji: cloud terus masuk saat IMU berhenti, kebalikannya, dan overload CPU; RAM dibatasi, status degraded jelas, pemulihan tidak mengejar backlog lama.

### F09 — File log gagal dibuka tetapi FILE* tetap dipakai
Prioritas: P1. Bukti: STATIS, dengan path build lama terobservasi.

laserMapping.cpp:927 fopen tanpa pemeriksaan; destructor:972 fclose(fp) tanpa guard; dump_lio_state_to_log memakai fprintf. build/fast_lio/fast_lio_runtime_paths.hpp masih menunjuk folder build lama yang hilang.

Dampak: null FILE* dapat memicu undefined behavior saat shutdown atau logging setelah binary dijalankan di layout baru.
Perbaikan: output log di runtime directory yang dapat ditulis; RAII; validasi open; jangan tutup/tulis pointer null.
Uji: direktori log hilang/tidak writable dan shutdown normal.

### F10 — Cabang self time-sync IMU salah satuan
Prioritas: P1. Bukti: STATIS, KONDISIONAL.

laserMapping.cpp:359–362 membentuk rclcpp::Time(timediff_lidar_wrt_imu + get_time_sec(...)). Ekspresi tersebut dalam detik, sedangkan konstruktor numerik tunggal Time menerima nanodetik. Cabang aktif jika time_sync_en=true dan selisih cukup besar; konfigurasi saat audit false.

Dampak: timestamp IMU berubah orde besaran dan sinkronisasi gagal.
Perbaikan: gunakan helper konversi detik ke nanodetik yang konsisten; uji offset positif/negatif. Jangan mengaktifkan self-sync sebelum kontrak clock diverifikasi.

### F11 — Kontrak pesan IMU tidak sesuai konvensi ROS umum
Prioritas: P1 untuk integrasi. Bukti: STATIS + dokumentasi driver lokal.

lddc.cpp:355–367 meneruskan acceleration mentah dalam g, tidak mengonversinya ke m/s². Orientation tidak disediakan tetapi orientation_covariance[0] tidak ditandai -1. frame_id IMU juga selalu livox_frame, tidak mengikuti frame_id konfigurasi.

Dampak: FAST-LIO lokal memiliki normalisasi sendiri sehingga temuan ini tidak berarti semua estimasi FAST-LIO salah skala. Namun konsumen sensor_msgs/Imu umum dapat menafsirkan unit dan ketersediaan orientation secara salah.

Perbaikan: kontrak topic raw yang jelas dan output SI standar; konversi covariance dengan kuadrat faktor skala; sesuaikan diagnostik/kalibrasi/consumer bersamaan agar tidak melakukan konversi dua kali.
Uji: diam mendekati 9,80665 m/s² pada topic SI, gyro rad/s, orientation tidak tersedia ditandai benar.

### F12 — Nama topic di YAML tidak di-remap ke driver
Prioritas: P2. Bukti: STATIS.

src/livox_mid360_manager/launch/sensor.launch.py:90–91 membaca point_topic dan imu_topic. Node driver:102–118 tidak menerima remappings; nama konfigurasi hanya dipakai monitor.

Dampak: mengubah topic di YAML membuat monitor pindah sementara publisher tetap pada nama default.
Perbaikan: remap driver dan propagasikan nama ke seluruh consumer.
Uji: ubah kedua topic dalam konfigurasi uji; publisher dan semua consumer harus mengikuti.

### F13 — KISS-ICP gagal pada timestamp kosong atau seragam
Prioritas: P1. Bukti: STATIS + REPRODUKSI terisolasi.

src/kiss_icp/src/Utils.hpp:111–122 dereference hasil minmax_element tanpa pemeriksaan kosong dan membagi dengan max-min tanpa guard.

Reproduksi memakai fungsi asli yang diekstrak, dikompilasi C++17:
- vektor kosong: UBSan melaporkan null-reference dan ASan menunjukkan SEGV.
- [42,42]: dua nilai -nan, finite=0.
LeakSanitizer tidak cocok dengan lingkungan eksekusi awal; pengulangan kasus seragam dengan detect_leaks=0 mengonfirmasi NaN. Ini uji fungsi, bukan crash node produksi.

Perbaikan: tangani kosong, satu titik, NaN, range nol; pilih deskew off/skip scan dengan diagnostik yang eksplisit.
Uji: semua edge case dan data normal tidak merusak pipeline.

### F14 — KISS-ICP memakai identity ketika transform tidak tersedia
Prioritas: P1, KONDISIONAL.

OdometryServer.cpp:58–74 mengembalikan Sophus::SE3d() setelah lookup gagal. PublishOdometry tetap melabelkan hasil sebagai base_frame yang diminta.

Dampak: konfigurasi base_frame berbeda dari frame cloud bisa menghasilkan pose dengan label benar tetapi transform salah.
Perbaikan: tunda/tolak output sampai TF mounting tersedia; cache transform static tervalidasi.
Uji: hilangkan TF extrinsic pada launch uji; tidak boleh ada output yang diam-diam mengasumsikan identity.

### F15 — Peta P4 awal dan sisa keyframe dapat tidak pernah dipublikasikan
Prioritas: P2. Bukti: STATIS.

pose_graph_node.cpp:250–258 mensyaratkan pertambahan keyframe minimal map_publish_min_new_keys_ (3). Bila baru ada 1–2 keyframe atau sesi berhenti dengan tambahan 1–2 keyframe sesudah publikasi terakhir, timer tidak membangun peta baru. Layanan save_map memaksa publikasi, sehingga export tidak otomatis memiliki cacat yang sama.

Perbaikan: publish pertama segera; tambah batas usia dirty-map dan flush saat sesi berhenti.
Uji: 1 keyframe, 2 keyframe, lalu berhenti; preview tetap muncul dan mencakup keyframe terakhir.

### F16 — Map preview tidak menerima peta latched saat start terlambat
Prioritas: P2. Bukti: STATIS.

P4 publisher memakai transient_local; map_preview.cpp:42 memakai subscription SensorDataQoS yang volatile. Node yang baru start setelah peta terbit tidak meminta sampel historis.

Dampak: preview kosong sampai publikasi P4 berikutnya, yang bisa tidak terjadi saat robot diam.
Perbaikan: subscription reliable/transient_local/depth 1 untuk topic peta snapshot.
Uji: start map_preview setelah satu peta P4 terbit, tanpa menggerakkan robot; peta harus diterima.

### F17 — P5 reload tidak menghapus state waktu/antrean
Prioritas: P1 untuk replay/restart. Bukti: STATIS.

relocalizer_node.cpp:165–180 me-reset state/path, tetapi tidak last_stamp_, odoms_, clouds_. process:339 menolak stamp <= last_stamp_.

Dampak: reload setelah bag diulang ke waktu sebelumnya dapat menolak semua scan sampai waktu lama terlampaui; antrean sebelum reload bisa diproses setelah reload.

Perbaikan: reset epoch terpusat, antrean dan timestamp; deteksi clock jump; jangan campur measurement sebelum/sesudah map reload.
Uji: reload + replay rewind, lalu relocalize lagi tanpa restart proses.

### F18 — Persyaratan tracking berturut-turut tidak direset setelah timeout
Prioritas: P1. Bukti: STATIS.

localization_stop_gate.cpp:66 menaikkan consecutive_tracking_, tetapi reset hanya di callback status non-TRACKING:68. Ketika STATUS_TIMEOUT/ODOMETRY_TIMEOUT/TF_STALE terjadi di tick, arm dibatalkan tetapi counter lama dipertahankan.

Dampak: satu status TRACKING baru setelah putus dapat memenuhi syarat “dua pesan berturut-turut” menggunakan sejarah lama. Operator arm dan pemeriksaan TF tetap ada; temuan ini bukan klaim bypass seluruh guard.

Perbaikan: reset counter stabilitas pada gangguan yang relevan, hitung hanya epoch pemulihan baru.
Uji: TRACKING stabil → timeout → satu TRACKING; arm masih ditolak sampai syarat pemulihan baru terpenuhi.

### F19 — Diagnostik guard dapat mengatakan READY saat command timeout
Prioritas: P2. Bukti: STATIS.

localization_stop_gate.cpp:184–199 menghitung reason dari inhibitReason sebelum pemeriksaan command. Cabang command timeout menerbitkan COMMAND_TIMEOUT tetapi publishDiagnostics(reason) dapat masih menerima READY.

Dampak: status topic dan /diagnostics bertentangan pada siklus tersebut, walaupun output kecepatan sudah nol.
Perbaikan: satu effective_state final untuk status, diagnostics, output, dan latched fault.
Uji: command timeout pada saat tick diagnostik; semua kanal harus menyatakan fault yang sama.

### F20 — Covariance tidak ditransformasikan bersama pose
Prioritas: P1 untuk fusi downstream. Bukti: STATIS.

odom_base_bridge.cpp:80–82 menyalin pose covariance sensor lalu mengubah pose menjadi base_link. pose_graph_node.cpp publishCorrected juga menyalin covariance sebelum mengubah pose ke frame map.

Dampak: ketika extrinsic atau koreksi rotasi nontrivial, covariance tetap milik pose/frame semula; lever-arm dan cross-covariance diabaikan.
Perbaikan: propagasi menggunakan Jacobian transform pose yang benar. Pisahkan uncertainty odometri dan correction pose graph; jangan mengganti dengan angka optimistis tanpa dasar.
Uji: transform 90° dan mounting offset nonzero pada covariance anisotropik.

### F21 — Generator kalibrasi menerima baseline gagal/NaN
Prioritas: P2. Bukti: STATIS.

calibration_prepare.cpp membaca stationary_ok tetapi hanya menuliskannya kembali; tidak menolak false. Pemeriksaan acc_norm<=0 tidak menolak NaN. imu_stationary_stats juga tidak menyaring sampel nonfinite pada input.

Dampak: executable langsung dapat menghasilkan file berlabel generated/calibrated input dari baseline yang tidak layak. tools/calibrate_mid360_imu.sh sudah memeriksa stationary_ok, tetapi tidak semua jalur CLI melewati skrip tersebut.

Perbaikan: finite checks, syarat stationary_ok=true, jumlah/durasi/gap dan kewajaran gravitasi; output atomik hanya setelah lolos.
Uji: false, NaN, data kosong, sensor berhenti sebagian durasi, dan baseline diam valid.

### F22 — Opsi --prefix ditimpa YAML dan pengecekan jaringan mengabaikan netmask
Prioritas: P2. Bukti: STATIS.

livox_network_setup.cpp:89 membaca --prefix, kemudian:106–108 menimpanya dari YAML. interface_has_ip hanya membandingkan alamat IP tanpa netmask.

Dampak: perintah --prefix tidak memiliki prioritas yang dijanjikan; alamat /16 dapat dilaporkan cocok dengan permintaan /24.
Perbaikan: tandai opsi CLI yang eksplisit, prioritaskan CLI, periksa prefix/mask aktual.
Uji: YAML /24 + CLI /16 dan interface dengan IP sama/mask berbeda.

### F23 — Timeout HTTP 3 detik tidak berlaku pada pembacaan sinkron
Prioritas: P2. Bukti: RUNTIME + STATIS.

web_dashboard.cpp:260–269 menggunakan beast::tcp_stream.expires_after(3s) bersama http::read sinkron. Dokumentasi Boost menyatakan deadline stream berlaku pada operasi asynchronous, bukan blocking I/O.

Uji satu koneksi lokal parsial dibiarkan 4,2 detik lalu dilengkapi; server masih menjawab HTTP 200. Socket uji ditutup setelah selesai. Tidak dilakukan pengujian massal/penjenuhan worker.

Dampak: client yang macet bisa menahan worker; destructor menunggu join worker sehingga shutdown dapat tersendat.
Perbaikan: async_read/async_write dengan deadline dan pembatalan socket; shutdown menutup semua sesi; respons 503 saat kapasitas habis.
Uji: satu client parsial, slow response, dan shutdown harus selesai dalam deadline.

## 5. Hambatan performa dan perbaikan

### O01 — OpenMP tertaut, tetapi loop utama tidak diparalelkan
laserMapping.cpp:697–700 membungkus pragma OpenMP dalam #ifdef MP_EN. CMakeLists hanya mendefinisikan MP_PROC_NUM=3. flags.make aktif memperlihatkan -fopenmp dan -O3, tetapi tidak -DMP_EN.

Tindakan: aktifkan compile definition secara eksplisit ketika OpenMP ditemukan; audit akses data loop; jadikan jumlah thread parameter dengan batas. Benchmark 1/2/4/8 thread. Menyetel OMP_NUM_THREADS saja tidak membuka blok #ifdef yang sudah dibuang compiler. Jangan mengklaim speedup tertentu sebelum profiling.

### O02 — Data tampilan dibatasi di server dan browser
web_dashboard.cpp:79: live cloud disampling paling cepat setelah >0,45 detik; :92: LIO setelah 0,8 detik.
web/index.html:175: status() hanya dipanggil setiap 1.800 ms. Fetch point ada di fungsi itu, dengan interval minimum 1.200 ms untuk live/LIO dan 2.600 ms untuk P4.

Akibat normal pada HTTP cepat:
- live/LIO: sekitar sekali per 1,8 detik = 0,56 Hz.
- P4 fetch: sekitar sekali per 3,6 detik = 0,28 Hz.
- P4 backend sendiri dibangun tiap 5 detik bila jumlah keyframe cukup.
requestAnimationFrame menggambar ulang point set yang sama; tidak menambah frekuensi data baru.

Tindakan: pisahkan polling status 1 Hz dan stream point 5–10 Hz; idealnya gunakan Foxglove Bridge atau WebSocket binary; sequence/stamp di payload; one request in flight; redraw hanya ketika data/kamera berubah; gunakan WebGL bila titik banyak. Peta global boleh tetap 0,2–1 Hz, sementara scan live/pose tampil cepat.

### O03 — ICP, optimasi graph, dan build peta memblokir callback dalam satu executor
P4 memanggil processPair/ICP/iSAM2 melalui callback subscriber, timer publishMap dan layanan save pada spin tunggal. P5 juga melakukan global search sejumlah seed pada callback yang sama dengan status timer.

Tindakan: pisahkan penerimaan/sinkronisasi, worker backend dan snapshot map; batasi pekerjaan per siklus. P5 health/watchdog harus tetap responsif ketika relocalization mahal. MultiThreadedExecutor saja tanpa callback group, ownership, dan proteksi state tidak cukup.

### O04 — Batas output peta bukan batas biaya seluruh pemetaan
P4 publishMap mentransformasikan seluruh cloud setiap keyframe tersimpan, lalu baru mengambil stride sampai 250.000 titik output. Seluruh local cloud tetap disimpan sampai 1.500 keyframe. detectLoop membandingkan descriptor terhadap kandidat sepanjang riwayat; descriptorDistance memeriksa 30 shift x 1.200 sel per kandidat. calculateEstimate/path juga dibuat kembali setiap keyframe.

Tindakan: submap/incremental voxel map; transform hanya keyframe baru sebelum loop correction; sample sebelum transform jika memang preview; spatial index untuk kandidat; cache descriptor norm; worker graph terpisah; export map penuh dipisahkan dari preview. Berikan metrik jumlah titik tersimpan, keyframe, waktu ICP, graph dan map-build.

### O05 — Duplikasi sorting, alokasi, salinan, dan mutex
- mid360_handler preprocess.cpp:559 menyortir waktu titik; IMU_Processing.hpp:223 menyortir lagi sebelum deskew.
- lddc.cpp:259–274 membangun vector per titik tanpa reserve lalu memcpy ke message.
- pub_handler.cpp:144/234 menyalin paket ke/dari queue.
- ProcessCartesianHighPoint:387–388 mengunci mutex per titik.

Tindakan: satu kontrak sort dengan flag/validasi ordering; reserve/reuse buffer; move paket bila ownership memungkinkan; batch lock per paket; tulis output teralokasi sekali. Verifikasi urutan timestamp dan isi titik sebelum/sesudah optimasi. Tidak mengganti timestamp point menjadi satu timestamp frame.

### O06 — Logging tetap aktif sebagian meskipun dinonaktifkan
laserMapping.cpp:1049–1050 menulis fout_pre dengan endl di luar if(runtime_pos_log), sehingga melakukan flush per scan ketika file tersedia.

Tindakan: seluruh log debug tunduk pada flag, buffered/asynchronous logging, rotasi ukuran. Statistik ringkas tetap tersedia tanpa dump matriks setiap frame.

### O07 — Path historis disiarkan berulang
FAST-LIO path.poses tumbuh tanpa cap pada :684. Dua evaluator stage3/stage4 default aktif dan masing-masing menerbitkan seluruh path sampai 20.000 pose pada setiap odometri; P5 juga menerbitkan seluruh path.

Tindakan: path untuk visualisasi dibatasi berdasarkan jarak/waktu dan dipublikasikan 1 Hz atau on-demand, simpan trajectory penuh terpisah untuk evaluasi. Matikan evaluator di profil production bila tidak diperlukan. CloudFilter preview juga sebaiknya berhenti memproses bila tidak ada subscriber.

### O08 — P5 preprocessing dan pencarian global masih boros
Pada P5, transform cloud dan dua voxel filter berlangsung sebelum pemeriksaan icp_update_stride. ROI menelusuri seluruh map untuk setiap kandidat dan ICP membangun target berulang. Mengurangi frekuensi ICP belum mengurangi semua biaya preprocessing.

Tindakan: jalur propagasi pose ringan dipisahkan dari ICP; coarse map disiapkan sekali; ROI memakai spatial index; cache local target; coarse processing hanya pada frame refinement/global search. Ukur time-to-relocalize dan kualitas kandidat.

## 6. Batas desain/akurasi yang perlu diselesaikan

- extrinsics.yaml saat audit masih uncalibrated; LiDAR–IMU di YAML FAST-LIO identity. Kedua kalibrasi berbeda dan harus dibuktikan terpisah.
- Saat time_type tidak sinkron, GetEthPacketTimestamp di pub_handler.cpp:260–269 memakai waktu host saat callback, bukan clock akuisisi sensor. Ini jalur desain yang memasukkan jitter transport/scheduling ke waktu measurement. Gunakan PTP/GPS bila tersedia dan valid; bila memetakan clock sensor ke host sendiri, tangani offset/drift/clock jump dengan telemetry.
- P5 global search hanya mencoba maksimal sejumlah pose trajectory tersimpan (default 16), tanpa pencarian yaw menyeluruh. Tidak ada jaminan relocalization dari lokasi/yaw sembarang. Perlu retrieval descriptor + kandidat posisi/yaw, atau initial pose operator.
- Batas P4 1.500 keyframe memang disengaja dan sudah menghasilkan diagnostics error ketika penuh; ini bukan bug tersembunyi. Untuk misi panjang perlu submap/session rollover atau pemuatan kembali, bukan sekadar menaikkan angka tanpa batas RAM.
- Batas titik preview/export yang sekarang ada memengaruhi densitas. Voxel dan stride tidak bisa dinilai hanya dari tampilan “lebih rapat”; bandingkan drift, ketebalan dinding, overlap, dan obstacle retention dengan data sama.
- FOV 360° sensor tidak berarti semua konfigurasi algoritme bernama fov_degree boleh langsung diubah ke 360. Telusuri pemakaian parameter dan batas algoritmenya.
- PCD-to-OccupancyGrid menggunakan floor_z/height band tetap. Nilai harus cocok terhadap frame peta dan medan; uji lantai miring, obstacle rendah, dan unknown space sebelum Nav2.

## 7. Frame rate: arti dan target

Spesifikasi resmi MID-360S: 200.000 titik/detik first return, 10 Hz tipikal, Ethernet 100 BASE-TX. Driver ROS resmi mendokumentasikan publish_freq hingga 100 Hz. Mengubah publish_freq membagi stream menjadi jendela berbeda, tidak mengalikan jumlah titik fisik per detik.

Perkiraan teoritis sebelum invalid-point filtering, range filter, stride, voxel dan overhead:

| Publish cloud | Durasi frame nominal | Titik/frame pada 200k titik/s | Sesudah stride 3, sebelum voxel |
|---|---:|---:|---:|
| 10 Hz | 100 ms | 20.000 | ~6.667 |
| 20 Hz | 50 ms | 10.000 | ~3.333 |
| 50 Hz | 20 ms | 4.000 | ~1.333 |
| 100 Hz | 10 ms | 2.000 | ~667 |

Angka ini bukan pengukuran MID-360S pengguna. Paket, return mode dan kondisi adegan memengaruhi realisasi.

Rekomendasi bertahap:
1. Stabilkan input 10 Hz + IMU sesuai rate aktual yang didukung sensor (konfigurasi monitor saat ini mengharapkan 200 Hz).
2. Perbaiki TF/time/frame/covariance, lalu profile seluruh pipeline.
3. Uji 20 Hz dengan timestamp dan scan interval yang konsisten; sinkronkan publish_freq, diagnostics.expected_cloud_hz dan konfigurasi lain yang relevan. MID360 handler memakai timestamp titik, sehingga perubahan scan_rate saja bukan pengatur rate nyata.
4. Pada 20 Hz, nilai point_filter_num=1/2/3 dibandingkan menggunakan rekaman adegan yang sama; pilih berdasarkan latency DAN kualitas.
5. 50/100 Hz hanya eksperimen setelah 20 Hz lolos. Scan yang sangat jarang dapat memperburuk observability/registration meskipun Hz tampak lebih tinggi.
6. Jika consumer membutuhkan pose 50–100 Hz, pertimbangkan propagasi IMU antar-koreksi LiDAR dengan timestamp/covariance dan status prediksi yang jujur. Itu bukan LiDAR update atau koreksi SLAM baru 100 kali/detik.

Target awal berikut adalah acceptance yang diusulkan, bukan hasil yang sudah dicapai:

| Lapisan | Target awal | Ukuran keberhasilan |
|---|---|---|
| Driver | 10 Hz, kemudian 20 Hz | Rate >=95% target saat steady; gap/drop terhitung; tidak ada backlog meningkat |
| FAST-LIO 10 Hz | Mengikuti scan masuk | P95 proses <70 ms; P99 <100 ms |
| FAST-LIO 20 Hz | Mengikuti scan masuk | P95 proses <35 ms; P99 <50 ms |
| Freshness | Terukur dengan clock tervalidasi | Pisahkan usia sejak awal/akhir scan, waktu antre, dan waktu proses |
| P4 loop/graph | Background sesuai keyframe | Tidak menahan jalur penerimaan/odometri |
| Peta global | 0,2–1 Hz/on-change | Pertama dan dirty-tail tetap terbit; tidak rebuild tanpa perubahan |
| Preview scan | 5–10 Hz | Sequence baru dan usia data terlihat; bukan sekadar render FPS |
| Render browser | 30–60 FPS bila perangkat mampu | CPU browser wajar, tanpa antrean request |
| Soak | Minimal 24 jam | Melewati ambang lama 720k frame; RAM stabil dan tidak ada error sanitizer pada pengujian terkait |

## 8. Rencana perbaikan berurutan

### Tahap A — Buat proyek dapat dibangun dan dijalankan kembali
Cakupan: F01, F09, inventaris satu workspace aktif.
- Simpan snapshot konfigurasi, lokasi binary aktif dan service; versi sumber saat ini belum berada dalam root Git.
- Tetapkan lokasi canonical yang benar dan perbaiki root relative/environment.
- Sinkronkan build/install/service; pastikan tidak ada path build lama tertanam untuk logging.
- Tambahkan preflight untuk keberadaan paket, config, dependency dan writeability.
Gate: shell bersih bisa memuat environment, build berjalan pada path yang benar, paket/config yang di-resolve sesuai manifest. Restart runtime dilakukan sesudah perbaikan siap dan tidak mengganggu sesi robot aktif.

### Tahap B — Pulihkan sensor dan rekam baseline nyata
Cakupan: jaringan, input stream dan waktu.
- Periksa IP/power/link hingga sensor benar-benar merespons; jangan ubah konfigurasi jaringan remote secara membabi buta.
- Rekam 60 detik diam dan rute tertutup dengan gerak/yaw, timestamp per titik dan IMU.
- Ukur raw Hz, point/s, point/frame, arrival jitter, queue age, packet drop, scan duration dan clock mode.
- Catat CPU tiap thread/node dan RSS; jangan simpulkan bottleneck dari persen CPU global.
Gate: input terus mengalir dengan waktu monotonic dan data cukup untuk replay yang identik.

### Tahap C — Selesaikan correctness sebelum menaikkan rate
Cakupan: F02–F11, F13–F14, F17–F20.
- Buffer bounded, reset epoch, timestamp TF, konversi satuan waktu.
- Kontrak IMU/LiDAR/base_link dan propagasi covariance.
- Guard data finite/ukuran input; recovery P5 dan state gate.
- Pisahkan unit IMU raw/SI dan perbarui consumer.
Gate: uji kasus waktu mundur, stream putus, parameter extrinsic nonzero, timestamp kosong/seragam dan long-run lulus.

### Tahap D — Turunkan latency CPU
Cakupan: O01, O03–O08.
- Tambah histogram waktu per tahap dan antrean sebelum mengubah algoritme.
- Aktifkan OpenMP terukur; hilangkan flush debug dan sort ganda.
- Kurangi salinan/alokasi dan lock per titik.
- P4/P5 worker terpisah, submap/incremental update, path throttling.
Gate: data replay yang sama menunjukkan penurunan P95/P99, tanpa kenaikan drift atau penurunan loop quality yang tidak diterima.

### Tahap E — Perbaiki visualisasi dan operasional
Cakupan: F12, F15–F16, F21–F23, O02.
- Preview latched, first-map/dirty-tail publish, topic remapping.
- Stream tampilan 5–10 Hz; status 1 Hz; payload sequence/timestamp/frame.
- HTTP deadline sebenarnya, guard layanan save, parsing/config validation.
Gate: late subscriber langsung menerima peta, data tampilan baru terukur, HTTP idle selesai sesuai deadline dan semua error terlihat jelas.

### Tahap F — Benchmark 10/20 Hz dan penerimaan lapangan
Matriks:
- Rate 10 dan 20 Hz.
- Thread 1/2/4/8; ambil kombinasi yang paling stabil, bukan yang paling banyak.
- Profil voxel sekarang dibanding profil detail; stride 1/2/3.
- Headless, dashboard, Foxglove, recording aktif/nonaktif.
- Diam, lurus, belok cepat, area minim fitur, rute tertutup, reconnect, dan sesi panjang.
- Ukur drift terhadap referensi/pengukuran fisik yang tersedia, ATE/RPE hanya jika ground truth/alignment didefinisikan.

Gate final: 20 Hz dipilih hanya jika processing deadline, queue age, kontinuitas dan kualitas peta lolos. Jika tidak, pertahankan 10 Hz dengan latency rendah dan preview lebih cepat; laporan benchmark tetap mencatat penyebabnya.

## 9. Daftar regression test yang perlu dibuat

| Uji | Temuan yang ditutup | Ekspektasi |
|---|---|---|
| Root dipindah + shell bersih | F01/F09 | Tidak ada referensi lama; kegagalan dependency menghasilkan exit nonzero |
| 720k+ callback dan 100k+ fitur | F02/F03 | Tidak ada out-of-bounds; memori terbatas |
| TF timestamp + extrinsic nonzero | F04/F06/F20 | TF, pose, cloud dan covariance konsisten |
| Covariance diagonal sentinel | F05 | Posisi/rotasi dan frame pesan tidak tertukar/terlambat |
| Clock jump dan bag rewind | F07/F10/F17 | Reset satu epoch, tidak memproses backlog dari epoch lama |
| Salah satu stream putus | F08/F18 | Queue dibatasi, stop/degraded, recovery butuh bukti baru |
| IMU SI dan orientation unavailable | F11 | Unit dan covariance marker benar |
| Topic custom | F12 | Driver, monitor, SLAM mengikuti remap |
| Timestamp kosong/seragam/nonfinite | F13 | Tidak crash/NaN; deskew fallback jelas |
| TF KISS hilang | F14 | Tidak memublikasikan pose base palsu |
| 1–2 keyframe lalu diam | F15 | Peta pertama/terakhir tetap terbit |
| Subscriber peta start terlambat | F16 | Menerima snapshot latched |
| Guard command timeout | F19 | Output nol dan semua status konsisten |
| Baseline kalibrasi invalid | F21 | Tidak menghasilkan “kalibrasi valid” |
| Prefix CLI vs YAML | F22 | CLI menang; mask diperiksa |
| HTTP parsial + shutdown | F23 | Deadline bekerja, worker tidak tertahan |
| Performa identik-replay | O01–O08 | P95/P99 membaik dengan kualitas terjaga |

Fixture ROS harus berjalan pada domain terisolasi dari robot. Jangan menginjeksikan /cmd_vel atau cloud sintetis ke domain live untuk pengujian.

## 10. Sumber teknis primer

1. Livox MID-360S specs: https://www.livoxtech.com/mid-360s/specs
2. Livox ROS Driver 2, parameter publish_freq: https://github.com/Livox-SDK/livox_ros_driver2
3. Boost.Beast basic_stream, bagian Blocking I/O: https://www.boost.org/latest/libs/beast/doc/html/beast/ref/boost__beast__basic_stream.html
4. ROS REP-145, unit dan covariance IMU: https://ros.org/reps/rep-0145.html

Sumber online diakses pada sesi audit 10 Oktober 2026. Temuan proyek berasal dari kode dan runtime lokal yang dibaca pada sesi ini.

## Lampiran: snapshot teks yang ditelaah

Hash berikut untuk salinan teks analisis, dengan normalisasi newline saat materialisasi; bukan attestation binary atau fingerprint keseluruhan repository. File yang diperiksa langsung (misalnya service dan runtime YAML) tidak semuanya tercantum di sini.

| File | Baris | SHA256 snapshot teks |
|---|---:|---|
| config/extrinsics.yaml | 16 | e915e80bbcd8e917812ab8dff4e9efc428e51c5d65bab6549f922942577b2bb3 |
| src/fast_lio/CMakeLists.txt | 69 | 2a2f48dc7f380afa40013373c234ddeaff8e451dcf86ca125e4bf3a46c0f37be |
| src/fast_lio/config/runtime_paths.hpp.in | 2 | fe302d5a3528c36c07f1bf87eda05407fd9d83fa95e7cc6d825bdcf0cd51eecf |
| src/fast_lio/include/use-ikfom.hpp | 126 | cde90f11353f9e2e56dfe49b7dfe790b47d11b706b366519a6a9c0a06b0e872d |
| src/fast_lio/src/IMU_Processing.hpp | 379 | 2b786c50c255a118fe95c9b017ef13af9e27a79b0624a718d4e192c3e5ca138d |
| src/fast_lio/src/laserMapping.cpp | 1218 | 0cb1229da12d914be23dfea69dc51b6efd378446ceffd05b9f4adb7081dfaf36 |
| src/fast_lio/src/preprocess.cpp | 1072 | 4bdaeab59cd980befda2488f2bc67601da1196dd736f151a960d01518dbf4fbd |
| src/kiss_icp/src/OdometryServer.cpp | 260 | c9f6096afd1c3dc694b802276d89d856ac231bc540aa466ce6d78c2f72dddab3 |
| src/kiss_icp/src/Utils.hpp | 227 | eca7da18c78449d4d7d2d515b9f592947728c4ec43acab1ec2dbc17f48676b32 |
| src/livox_mid360_manager/launch/stage3.launch.py | 76 | f38bfff82c9ba80265adc9fc6c8b93271edeacd3175a0bfec0c8a757798a0a37 |
| src/livox_mid360_manager/launch/stage4.launch.py | 65 | 07b8befdf76da9a685e9c63f0aedd256f1f1bd324309ad614037ecd1b448b074 |
| src/livox_mid360_manager/src/calibration_prepare.cpp | 114 | 52ce40c4cedb156e80c4270127a6cc8c31dd2ef72e51d965062ff2621eb523e1 |
| src/livox_mid360_manager/src/imu_stationary_stats.cpp | 218 | d185dd2dc36662b2553cb8d9507bbc4decebdf4939443b5b320f306d5afdbb8b |
| src/livox_mid360_manager/src/livox_diagnostics.cpp | 283 | a75598dffabe6888d6bf1c3df9c730389cd66917e9f4249a2d479c7d00344587 |
| src/livox_mid360_manager/src/livox_discover.cpp | 308 | 39bd894547fb71f2d0f5819f764c82393ef5b5ab656734fbd8ef24a7e634f293 |
| src/livox_mid360_manager/src/livox_network_setup.cpp | 141 | 97f7730e37c3a55ab575afee4f820381900ab92a630c8c37e6f9efc9f6bb31e9 |
| src/livox_mid360_manager/src/stage2_odom_evaluator.cpp | 405 | 6fac218ef2425e454e1658aba41a5dc8436b85523aac72529d70f0b516fa247f |
| src/livox_ros_driver2/src/comm/ldq.cpp | 150 | d34edd199784fb01453ede9e2651fad333d6128fbf0ee300453b47acd4203736 |
| src/livox_ros_driver2/src/comm/lidar_imu_data_queue.cpp | 70 | 2d7854d0aaefd4c841b5fa49fb7811ec59329587103f7692edee8db0c7113614 |
| src/livox_ros_driver2/src/comm/pub_handler.cpp | 505 | a07421385eae22cf0113d32885162d9c0e9f0487f76f0d28121ef0b14e5ed8dd |
| src/livox_ros_driver2/src/comm/pub_handler.h | 138 | e84316b5c3f6952e25b611bd605ad46fd370d344b0eab6a0aa1f27dc83211374 |
| src/livox_ros_driver2/src/lddc.cpp | 470 | 184d669c6e6a87ca626a285700c4927c81df41088d3dd42e54e768f4c4e4debf |
| src/livox_ros_driver2/src/lds_lidar.cpp | 212 | 4ba80b226c79ba9e1d7fd6e919acd4325da5ad138f7e0991f784aa6a94885106 |
| src/livox_ros_driver2/src/livox_ros_driver2.cpp | 157 | 6ea3b771d639eb3842ab0520d32041f88d55c1fc23afbcdf521e34f5bae6efe6 |
| src/slam3d_mid360_dashboard/launch/mapping.launch.py | 50 | 4f3190879fb72d0a1aeffa1a29203fddf2b430287ef48c90ed2b782437f51890 |
| src/slam3d_mid360_dashboard/src/cloud_filter.cpp | 71 | 506b48d7eba38ded3115c354b0f4ccdc7ef2b1e5540d6d9b1fdd1ffd64faadd0 |
| src/slam3d_mid360_dashboard/src/map_preview.cpp | 119 | 9449950a9baf93b0e84eea0114537a30035af733a23948c4dc963d7e8f01b3c3 |
| src/slam3d_mid360_dashboard/src/web_dashboard.cpp | 351 | 75a51a80de0a0287029289d5876d952d399c383431ade20da6d0471241945b18 |
| src/slam3d_mid360_dashboard/web/index.html | 176 | 8e6498618d4248ba6e16d39bc22268032dbefa97dfaedb5325b2d9169298b03b |
| src/slam3d_nav2_bridge/launch/bridge_only.launch.py | 72 | d848eceee33e693101333ecdc53a6c95e975580c427b4729dc18ffbffeb0e080 |
| src/slam3d_nav2_bridge/launch/stage6.launch.py | 75 | 4943a4fc21e8356795494d4a0c7b74e205bd19938859d7056dbfe8a65c5ca247 |
| src/slam3d_nav2_bridge/src/localization_stop_gate.cpp | 251 | 41dd10db47b06f65d899656019df25e8beaf9595a92fce49940fb69ff43dd11c |
| src/slam3d_nav2_bridge/src/odom_base_bridge.cpp | 139 | a6b1b5ccbaf7841c70696f414f054075553b0e01da61f46a58a5514050ad4700 |
| src/slam3d_nav2_bridge/src/pcd_occupancy_map.cpp | 100 | 77afd11e9d656bda614892afc337c9bf5bb3245b652245bc1a52748df139706c |
| src/slam3d_pose_graph/config/stage4.yaml | 35 | 283fe40bd4826cd976981c9f27da36a53c027a429a378641f036d804273d3903 |
| src/slam3d_pose_graph/src/pose_graph_node.cpp | 568 | cbd82d8a4f396f39f568c81c37cf4731772e94b036daca4338a3876ec8683017 |
| src/slam3d_relocalizer/config/stage5.yaml | 32 | d7cab3cf6aefde70bd5382959418f0f0b5394dd973a116a8992b2f9d0ba76fdc |
| src/slam3d_relocalizer/src/relocalizer_node.cpp | 552 | 420faf0851083d49f227cd97745a65971562cab35749b6f65005e5b7f423485a |
| tools/calibrate_mid360_imu.sh | 30 | b16c59d5354b32bc555c9734b29f93f2eeb36079caacd0e8729d63b541cc1078 |
| tools/run_mid360_dashboard.sh | 21 | a6a1a1276eb0feb264d5766a471790ceaab445ea239fd4f996723707f3876da8 |
| tools/run_mid360_detail_mapping.sh | 31 | c94ca77adc4210d88bda45a711bf5d1c065e0207c521eb4b4a0ecc352b8d5117 |
| tools/run_mid360_foxglove.sh | 7 | d2b868032926f4215e444f0f27be3806377102f3142d2409aef8e5028657b101 |
| tools/stock_px4_tuning_server.py | 105 | 8e21478a4206cc371027e97abc40ea7862433df4533f6e878b3ca30f9f322dda |
