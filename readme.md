# TB3 ROS2 FSM Waypoint Patrol (Gazebo)

Project ini mengimplementasikan **Finite State Machine (FSM)** untuk navigasi sederhana TurtleBot3: robot berpindah-pindah **waypoint**, dan melakukan **obstacle avoidance** berbasis aksi “belok lalu maju” menggunakan timer.

---

## Konsep FSM

FSM memodelkan perilaku robot sebagai kumpulan **state** (mode kerja).  
Pada setiap loop (20 Hz), robot:
1) mengecek **event/condition** (goal tercapai, obstacle terdeteksi, timer habis, dll)
2) jika perlu melakukan **transition** (pindah state)
3) menjalankan **action** sesuai state saat ini (publish `cmd_vel`)

---

## State Diagram

![Image](https://github.com/user-attachments/assets/67154e4f-a065-4e1c-be90-9a71d4396d49)

---

## Definisi Elemen FSM

### 1) State (Kotak pada diagram)
- **START** : robot berhenti (awal sistem)
- **TURN** : robot berputar untuk menghadap ke waypoint (align heading)
- **GO** : robot maju menuju waypoint sambil koreksi arah
- **AVOID_TURN** : robot berputar menghindari obstacle (arah kiri/kanan dipilih)
- **AVOID_GO** : robot maju sebentar untuk menjauh dari obstacle
- **DONE** : robot berhenti, lalu pindah ke waypoint berikutnya

### 2) Transition (Panah pada diagram)
Panah menunjukkan perpindahan dari satu state ke state lain, misalnya:
- START → TURN
- TURN → GO
- GO → DONE
- GO/TURN → AVOID_TURN
- AVOID_TURN → AVOID_GO
- AVOID_GO → TURN
- DONE → TURN

### 3) Event / Condition (Label pada panah)
Label di panah adalah pemicu perpindahan state:

- **begin**  
  Sistem mulai berjalan dari START ke TURN.

- **ALIGNED**  
  Robot sudah menghadap waypoint (error sudut kecil), sehingga boleh masuk GO.

- **GOAL**  
  Robot sudah cukup dekat dengan waypoint (jarak < toleransi), sehingga masuk DONE.

- **OBS**  
  Ada obstacle di depan (jarak depan < ambang), sehingga masuk AVOID_TURN.

- **TIMEOUT**  
  Timer menghindar habis (durasi tercapai), berpindah ke state berikutnya.

- **next waypoint**  
  Setelah DONE, waypoint diupdate ke target berikutnya, lalu kembali ke TURN.

### 4) Action (Perilaku/Output pada tiap state)
Action utama FSM adalah menerbitkan perintah kecepatan `cmd_vel`:

- **START**: `v=0, w=0` (stop)
- **TURN**: `v=0, w=±ang_speed` (rotate ke arah goal sampai ALIGNED)
- **GO**: `v=lin_speed, w=f(error_heading)` (maju + koreksi arah)
- **AVOID_TURN**: `v=0, w=avoid_dir*ang_speed` selama `avoid_turn_s`
- **AVOID_GO**: `v=0.9*lin_speed, w=0` selama `avoid_fwd_s`
- **DONE**: `v=0, w=0`, lalu `wp_idx++` (next waypoint)

---

## Parameter Penting

Beberapa parameter yang mempengaruhi event/condition:

- `goal_tol` : toleransi jarak untuk kondisi **GOAL**
- `obstacle_front` : ambang jarak obstacle untuk kondisi **OBS**
- `avoid_turn_s` : durasi **AVOID_TURN** (TIMEOUT 1)
- `avoid_fwd_s` : durasi **AVOID_GO** (TIMEOUT 2)
- `lin_speed`, `ang_speed` : kecepatan linear & angular

---

## Catatan Singkat

Robot menghadap waypoint (**TURN**), bergerak menuju waypoint (**GO**), jika ada obstacle maka belok dan maju sebentar untuk menghindar (**AVOID_TURN → AVOID_GO**), lalu kembali mengejar waypoint; saat sampai waypoint robot berhenti (**DONE**), lanjut ke waypoint berikutnya, dan begitu seterusnya.
