# TASKS.md — R2_mcu 実装計画（Codex向け）
対象: STM32F446RE + micro-ROS（低レベルI/O・安全・CAN）  
上位仕様の根拠: R2_ctrlのSPECで、STM32は `/imu` と `/ground_odom` 等を供給する前提になっている :contentReference[oaicite:0]{index=0}  
現状リポジトリ: CubeIDE生成構成（Core/Drivers/FreeRTOS/micro_ros utils） :contentReference[oaicite:1]{index=1}  
現状main.cには BNO055 / Encoder / PS5 / ODrive / RoboMaster などの下地と、micro-ROS publisherの雛形がある :contentReference[oaicite:2]{index=2}

---

## 共通ルール
- 1 task = 1 PRサイズの変更
- 変更ごとに確認:
  - ビルド（CubeIDE）
  - micro-ROS agent接続（UART/USB whichever）
  - CAN送受信（最小で1台）
- “安全停止”を常に最優先（通信断・E-STOP・CAN fault）

---

## Task 0: リポジトリ入口整備（README/STATUS/TASKS）
### 目的
進捗参照と再現可能ビルド手順を固定する（R2_ctrl側は文書が揃っているが、R2_mcuは入口が未整備）。
### 成果物
- `README.md`: ビルド手順、書き込み方法、micro-ROS agent起動、配線(CAN/IMU/Encoder)、トピック一覧
- `STATUS.md`: できてる/進行中/次にやる
- `TASKS.md`: 本ファイル（この計画）
### 受け入れ条件
- 初見メンバーが `README.md` だけでビルド＆書き込み＆agent接続まで到達できる

---

## Task 1: micro-ROS 通信基盤の固定（Node名・QoS・watchdog基盤）
### 目的
「MCUは通信が途切れたら必ず安全停止」までを基盤として確立する。
### 成果物
- micro-ROS初期化コード整理（関数分割: init / spin / pub / sub）
- `/mcu/heartbeat` publish（std_msgs/UInt32 等）: 周期・稼働確認
- “コマンド受信タイムアウト”の共通実装（後述Task 5で駆動へ適用）
### 受け入れ条件
- agent接続中、周期的にheartbeatが見える
- agent停止/ケーブル抜きで「安全停止に入った」ことがログ/LEDでわかる

---

## Task 2: IMU publish を仕様に合わせる（/imu sensor_msgs/Imu）
### 目的
R2_ctrl側仕様どおり `/imu : sensor_msgs/Imu` を出す :contentReference[oaicite:3]{index=3}  
現状はBNO055値を保持しているが、ROSの型・frame_id・timeが未整備 :contentReference[oaicite:4]{index=4}
### 成果物
- `/imu` publish（sensor_msgs/Imu）
  - orientation（quat）、angular_velocity、linear_acceleration を埋める
  - `header.frame_id = "imu_link"`（固定でOK）
  - timestamp を入れる（MCU timeでOK。将来同期する前提で良い）
- `/imu/status` publish（任意）
  - キャリブ状態（sys/gyro/accel/mag）や温度を出す（今の変数がある） :contentReference[oaicite:5]{index=5}
### 受け入れ条件
- Jetson側で `ros2 topic echo /imu` して破綻値が出ない（NaNなし）
- imuのpublish周期が一定（例: 50Hz/100Hz）で維持される

---

## Task 3: 接地エンコーダ（x,y）から /ground_odom を出す
### 目的
R2_ctrl仕様どおり `/ground_odom : nav_msgs/Odometry` を供給 :contentReference[oaicite:6]{index=6}  
（昇降で浮く前提なので “使える時だけ信頼する” 情報も出す）
### 成果物
- `/ground_odom` publish（nav_msgs/Odometry）
  - `child_frame_id="base_link"`、`header.frame_id="odom"`（暫定）
  - twist（vx,vy,wy）だけでもまずOK（poseは積分でも可）
- `/ground_contact` publish（std_msgs/Bool）または `/ground_odom/quality`（std_msgs/Float32）
  - “浮いたらquality=0” を上位が見て切り替えできるようにする
### 受け入れ条件
- 手で動かしたときにvx/vyがそれっぽく変化
- 接地状態を切り替えると quality/contact が切り替わる

---

## Task 4: CAN統合ドライバ（ODrive + RoboMaster）をモジュール化
### 目的
現状main.cに混在している ODrive送信/非常停止/RoboMaster送信を分離し、上位から呼べるAPIにする :contentReference[oaicite:7]{index=7}
### 成果物
- `odrive_can.c/h`:
  - `odrive_set_axis_state(axis, state)`
  - `odrive_set_input_vel(axis, vel_rev_s, torque_ff)`
  - heartbeat受信の監視（既にカウンタ変数があるので整理） :contentReference[oaicite:8]{index=8}
- `robomaster_can.c/h`:
  - `rm_set_current_4ch(can_id, i1..i4)` 等
- CANエラーカウント/バスオフ検知→faultに繋げる
### 受け入れ条件
- 既存のPS5操作（DPAD等）が“同じ挙動”で動く（回帰）

---

## Task 5: /cmd_vel を受けて駆動する（mecanum速度→各輪指令）
### 目的
Jetson側からの標準指令で動かす（R2_ctrlの運動プリミティブが実機フックできる状態へ）。
### 成果物
- Sub: `/cmd_vel`（geometry_msgs/Twist）
- mecanum逆運動学（vx,vy,wz → wheel1..4）
- 出力先は選択式:
  - ODriveなら `set_input_vel`（rev/s）
  - RoboMasterなら RPM→電流PID（現状のPID構造体があるので利用） :contentReference[oaicite:9]{index=9}
- watchdog適用:
  - `/cmd_vel` タイムアウトで 0 指令（安全停止）
### 受け入れ条件
- Jetsonから `ros2 topic pub /cmd_vel ...` で意図どおり平行移動/旋回する
- 指令停止（0）で確実に止まる
- タイムアウトでも確実に止まる

---

## Task 6: モード信号のI/F（FLAT/CLIMB）をMCU側にも入れる
### 目的
段差中は接地オドメトリが無効という設計を、MCUでも“一貫して出力”できるようにする :contentReference[oaicite:10]{index=10}
### 成果物
- Sub: `/loc_mode`（std_msgs/String: "FLAT"|"CLIMB"） ※R2_ctrl仕様 :contentReference[oaicite:11]{index=11}
- CLIMB中の挙動:
  - `/ground_odom` のquality=0（またはpublish停止）
  - 必要なら速度制限（wz制限など）を強制
### 受け入れ条件
- 上位がmodeを切ると、MCUのodom品質が即時に変わる
- CLIMB中に地面オドメトリが上位に混入しない

---

## Task 7: Fault/Diagnostics の体系化（見える化）
### 目的
実機デバッグを可能にする（原因が追えない状態を排除）。
### 成果物
- Pub: `/mcu/diagnostics`（std_msgs/Stringでも最初は可）
  - loop周期、free heap、stack watermark
  - CAN tx/rx count、error count、bus-off回数
  - IMU更新周期、エンコーダ更新周期
- Pub: `/mcu/fault`（std_msgs/UInt32 等、bitmask推奨）
  - e-stop、cmd_vel timeout、can bus-off、odrive heartbeat timeout など
### 受け入れ条件
- 現場で「止まった理由」がtopicで1発判明する

---

## Task 8: E-STOP入力・復帰シーケンスを確定
### 目的
競技運用で必須。現状は `emergency_stop_active` などの下地があるので、GPIO割り込み/復帰条件を固める :contentReference[oaicite:12]{index=12}
### 成果物
- E-STOP入力（GPIO割り込み）
- e-stop中はモータ出力を必ず0（ODrive/RoboMaster両方） :contentReference[oaicite:13]{index=13}
- 復帰は `/estop_reset` などの明示コマンドのみ（勝手に復帰しない）
### 受け入れ条件
- ボタンで即停止、解除しても勝手に動かない、復帰コマンドでのみ再開

---

## Task 9: 上位（R2_ctrl）とのI/F整合テスト（最小E2E）
### 目的
「R2_ctrlが想定するSTM32 I/F」へ合わせ込み、結合で壊れないことを確認する。
### 成果物
- Jetson側の最小テスト手順（READMEに追記でも可）:
  - `/imu` 受信OK
  - `/ground_odom` 受信OK（FLATのみ）
  - `/cmd_vel` で駆動OK
  - `/loc_mode` 切替でodom quality変化OK
### 受け入れ条件
- 上記4点が一通り動く（動画/ログをSTATUSに貼ると強い）

---

## 次の優先順（推奨）
1) Task 0（入口）→ 2) Task 2（/imu）→ 3) Task 5（/cmd_vel駆動）→ 4) Task 3（/ground_odom）→ 5) Task 6（mode）→ 6) Task 7（diagnostics）→ 7) Task 8（E-STOP）→ 8) Task 4（モジュール化）→ 9) Task 9（E2E）
