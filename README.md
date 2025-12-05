
# jetbot 길찾기

- jetbot 기본 컨테이너 샘플에 resnet으로 block unblock 2클래스 분류모델 학습하여 collision avoidance가 구현
- rplidar 추가하여 레이저 데이터를 학습하여 길찾기 구현
- navigation으로 길찾기 구현


- jetbot은 isaacsim asset으로 저장되어 있어 로드하면 액션그래프까지 포함된 상태로 로드 가능
- create > sensor > rotation lidar 추가
- 길을 구성한 벽 생성 및 add physics

![이미지](./image/Pasted%20image%2020251129155219.png)

- 카메라 세팅 후 스타트하면 다음과 같이 확인 가능

![이미지](./image/Pasted%20image%2020251129195744.png)




# 그래프 에디터 script node 추가


![이미지](./image/Pasted%20image%2020251129155455.png)


```python
import os
import csv
import numpy as np
import omni.graph.core as og


def setup(db):
    """
    액션그래프 실행 시 1회 실행되는 초기화 함수.
    CSV 파일 생성 + 헤더 작성.
    """
    state = db.per_instance_state

    # 저장 디렉토리
    save_dir = r"D:/making/jetson_demo/jetbot_data"
    os.makedirs(save_dir, exist_ok=True)

    # CSV 파일 경로
    csv_path = os.path.join(save_dir, "lidar_dataset.csv")

    # 첫 실행 시 헤더 생성
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["frame", "x", "y", "z", "v", "w"])

    # 상태 저장
    state.save_dir = save_dir
    state.csv_path = csv_path
    state.frame_idx = 0

    db.log_warning(f"[LIDAR CSV LOGGER] Saving to: {csv_path}")


def compute(db):
    """
    매 프레임마다 호출됨.
    LiDAR 포인트와 키보드 입력 기반 (v,w)를 읽어서 CSV에 저장.
    """
    state = db.per_instance_state
    frame = state.frame_idx

    # -------- LiDAR 포인트 (Nx3 numpy array) --------
    # Isaac Read Lidar Node의 output:data 는 이미 (N,3) float32 배열임
    points = np.array(db.inputs.points, dtype=np.float32)

    if points.size == 0:
        return True  # LiDAR 준비 전 프레임

    # -------- Keyboard → (v, w) 속도 변환 --------
    v_speed = 0.5       # Jetbot 전진 속도 (m/s)
    w_speed = 1.0       # Jetbot 회전 속도 (rad/s)

    w_key = db.inputs.key_w
    s_key = db.inputs.key_s
    a_key = db.inputs.key_a
    d_key = db.inputs.key_d

    # 기본값
    v = 0.0
    w = 0.0

    if w_key:
        v = v_speed
    elif s_key:
        v = -v_speed

    if a_key:
        w = w_speed
    elif d_key:
        w = -w_speed

    # Differential Controller 로 출력
    db.outputs.lin_vel = v
    db.outputs.ang_vel = w

    # -------- CSV Append --------
    with open(state.csv_path, "a", newline="") as f:
        writer = csv.writer(f)
        for x, y, z in points:
            writer.writerow([frame, float(x), float(y), float(z), v, w])

    state.frame_idx += 1

    if frame % 50 == 0:
        db.log_warning(
            f"[LIDAR CSV LOGGER] frame={frame}, points={points.shape[0]}, v={v:.2f}, w={w:.2f}"
        )

    return True

```



# lidar 로스 패키지 빌드

https://github.com/Slamtec/rplidar_ros/tree/release/v2.0.0  
깃 클론 후 catkin_make

```bash
cd catkin_ws
source devel/setup.bash

roscore

roslaunch rplidar_ros rplidar.launch

# wsl 에서 rviz
```


![이미지](./image/Pasted%20image%2020251129195517.png)



# 네비게이션
