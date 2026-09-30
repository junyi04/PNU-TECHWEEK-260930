<br>

<h1 align="center">
  2026 부산대학교 TECH WEEK:<br>
  Autonomous Mobile Robot의 Search & Rescue Mission
</h1>
<br>

---

<br><br>

![](부산대_TECHWEEK_Physical_AI.png)

## 현재 구현: 4단계 빨간 사과 탐지

Webots R2025a / TurtleBot3 Burger. 출발점을 `(0, 0, 0)`으로 삼아 센서로
위치를 추정하고, LiDAR 지도를 만들며 Frontier 탐색 및 A* 경로 계획을 수행합니다.
기본값은 **시뮬레이션 시간 180초 탐색 후 출발점 복귀**입니다.
탐색 중 YOLO11n으로 사과를 탐지하고 빨간색만 선별하여 지도에 기록합니다.
**사과로 접근해 도착을 판정하는 동작은 다음 단계입니다. 강화학습은 아직 없습니다.**

### 실행

1. 이 저장소 전체를 내려받습니다. 컨트롤러만 옮길 경우
   `controllers/apple_collector` 폴더 전체가 필요합니다 (`navigation.py`, `perception.py`, `models` 포함).
2. **Webots가 사용하는 Python 환경**에서 아래 명령을 실행합니다.

   ```sh
   python -m pip install -r controllers/apple_collector/requirements.txt
   ```

3. Webots에서 `worlds/apartment.wbt`를 열고 실시간 실행 버튼을 누릅니다.
   월드에는 `apple_collector` 연결과 `map`, `detections` Display가 설정되어 있습니다.
4. 코드 변경 후에는 실행 중 월드의 변경 사항을 저장하지 않고 파일을 다시 열어 실행합니다.

현재 검증 환경은 Windows / Webots R2025a / Python 3.13.3 / NumPy 2.2.6입니다.
GPU가 없으면 CPU로 탐지합니다. `controller`는 Webots가 제공하는
모듈이므로 같은 이름의 pip 패키지를 설치하지 마세요.

### 조작과 관측

Webots 3D 화면을 클릭한 후 키를 입력합니다.

| 키 | 동작 |
|---|---|
| N | 현재 위치에서 탐색 시작/재개 (180초 탐색 타이머 재시작) |
| H | 최초 출발점으로 복귀 요청 |
| M | 자동 주행 중단, 수동 대기 |
| W/S/A/D | 누르는 동안 저속 전후진/좌우 회전, 자동 주행 중단 |
| Space / X | 비상 정지 유지 |
| R | 비상 정지 해제 후 수동 대기 |
| T | 현재 위치에서 1단계 전진·회전 점검 시작 |

수동 조작 중에도 지도와 출발점은 유지합니다. 복귀 경로가 없거나 복귀 중 장시간
움직이지 못하면 정지 상태와 원인을 표시합니다. `HOME`은 추정 출발점 16cm 이내에
도달했음을 뜻하며 실제 위치 오차는 별도로 발생할 수 있습니다.

`map` Display: 회색=미탐색, 흰색=빈 공간, 검정=장애물, 보라=이동 흔적,
파랑=계획 경로, 초록=출발점, 빨강=현재 추정 위치.
지도 이미지는 `controllers/apple_collector/logs/map_latest.png`에도 갱신됩니다.
콘솔은 전후좌우 LiDAR 거리, 좌우 엔코더, 모드를 주기적으로 출력합니다.
카메라 해상도는 시작할 때 출력합니다. 자세한 기록은 같은 폴더의
`navigation_latest.jsonl`에 저장되며 재실행할 때 덮어씁니다.

로봇의 `controllerArgs`로 `["--manual"]`, `["--checkout"]`,
`["--explore-seconds=60"]` 등을 지정할 수 있습니다.

### 센서와 제한 사항

- 필수: 360도 LiDAR, 좌우 엔코더. gyro가 있으면 회전 추정에 함께 사용합니다.
- accelerometer는 진단 로그에만 사용합니다. 없어도 실행됩니다.
- camera는 사과 탐지·색상 판별·사과 상대 위치 추정에 사용합니다. 로봇 위치 추정에는 사용하지 않습니다.
- compass, GPS, Supervisor 실제 좌표는 주행에 사용하지 않습니다.
- 엔코더/선택적 gyro와 국소 scan matching 방식입니다. 전역 루프 폐쇄 SLAM이 아니므로
  긴 탐색에서는 오차가 누적될 수 있습니다. 지도는 출발점 중심 약 32m 정사각형입니다.
- 장애물 회피는 실시간 LiDAR 기반의 짧은 궤적 평가와 안전 정지입니다.
  LiDAR 높이 아래의 작은 물체와 단차는 감지하지 못할 수 있습니다.
- 테스트 통과가 모든 배치·보행자 상황에서 무충돌을 보장하지는 않습니다.

### 검증

```sh
python -m unittest discover -s tests -v
python tests/run_navigation_simulation.py --webots "C:\Program Files\Webots\msys64\mingw64\bin\webots.exe" --seconds 25
python tests/run_navigation_simulation.py --webots "C:\Program Files\Webots\msys64\mingw64\bin\webots.exe" --apartment --seconds 35
```

테스트는 `.tmp` 아래 별도 프로젝트를 만들며, 실제 좌표를 읽는 Supervisor는
테스트 관찰기에만 사용합니다. 제출 컨트롤러에는 해당 권한이 없습니다.

2026-09-30 검증 결과: 단위 테스트 17개 통과. 검증 월드는 약 5.21m 이동 후
실제 출발점에서 14cm 거리로 복귀했고, 아파트 월드는 약 5.43m 이동 후 17cm로
복귀했습니다. 두 시험에서 지면 위 장애물 접촉이 기록되지 않았습니다.
아파트 시험은 35초 탐색한 짧은 주행이며 전체 아파트 탐색 완료를 검증한 것은 아닙니다.

### 4단계 설치와 확인

학교 RTX 5070 데스크톱에서는 **그 PC의 프로젝트 폴더**에서 실행합니다.
이미 CUDA용 PyTorch를 설치했다면 첫 번째 pip 명령은 생략합니다.

```powershell
git pull
& C:\Users\user\.venv\Scripts\python.exe -m pip install torch==2.7.1 torchvision==0.22.1 --index-url https://download.pytorch.org/whl/cu128
& C:\Users\user\.venv\Scripts\python.exe -m pip install -r controllers/apple_collector/requirements.txt
```

Webots Python command에는 **현재 실행하는 PC에 존재하는 Python 경로**를 넣습니다.
위 `C:\Users\user\...`는 학교 데스크톱용이며 랩탑에 그대로 넣으면 안 됩니다.
GPU 없는 랩탑은 CPU용 PyTorch를 먼저 설치할 수 있습니다.

```sh
python -m pip install torch==2.7.1 torchvision==0.22.1 --index-url https://download.pytorch.org/whl/cpu
python -m pip install -r controllers/apple_collector/requirements.txt
```

`worlds/apartment.wbt`를 다시 열고 **실시간 모드**로 실행합니다. 가속 실행에서는
영상 처리보다 시뮬레이션이 빨라져 3초 이상 오래된 탐지 결과가 폐기될 수 있습니다.
초기 모델 로딩 동안 `loading`이 나오고, 정상 실행 시 `[vision] running device=cpu`
또는 `device=0`이 출력됩니다. 자동 모드는 CUDA 사용 가능 여부와 초기 추론을 확인하고,
실패하면 CPU로 전환합니다. `--device=cpu`, `--device=0`, `--no-vision`으로 지정할 수도 있습니다.
처음 자동 출발은 모델 로딩이 끝날 때까지 기다립니다. 수동 조작은 계속 사용할 수 있습니다.

View → Overlays에서 로봇의 `detections`와 `map`을 켜면 탐지 영상과 지도를 볼 수 있습니다.
원본 `camera`와 가구의 `display`는 별도 화면이므로 필요하지 않으면 끕니다.
탐지 화면은 마지막 처리 영상이며 실시간 원본 영상보다 늦게 표시됩니다.

- 기본 YOLO11n은 시험 영상의 Webots 사과를 원반·공 등으로 오분류했습니다.
  따라서 apple/orange/sports ball/frisbee 후보에 빨간색·원형 조건을 적용하는
  **시뮬레이터용 보완 판정**을 포함합니다. 일반적인 사과 인식 정확도를 보장하지 않습니다.
- 사과 모델의 지름 약 10cm와 영상 크기로 거리를 추정하고, 촬영 당시 로봇 추정 위치로
  지도 좌표를 계산합니다. 사과 배치 좌표나 Recognition API는 읽지 않습니다.
- 3회 이상, 0.3초 이상 반복 관측된 후보를 기록합니다. 35cm 이내 기록은 동일 대상으로
  연결하므로 가까이 붙은 사과나 큰 위치 오차에는 중복·병합 오류가 생길 수 있습니다.
- `logs/targets_latest.json`에 ID, 추정 좌표, 관측 횟수와 판정 출처를 저장합니다.
  `yolo_apple`은 YOLO 사과 분류, `appearance_candidate`는 보완 판정입니다.
  표시되는 confidence는 원래 YOLO 클래스 점수이며 사과일 확률이 아닙니다.
- 지도 주황색 표시는 반복 확인한 후보 위치입니다. **발견은 도착·수거 완료가 아닙니다.**
  사과를 찾아가는 동작과 두 목표 완료 후 복귀는 다음 단계에서 연결합니다.
- 학습 데이터에 맞춘 YOLO 추가 학습, 아파트 전체 탐색 검증 및 학교 GPU 실행 검증은 남아 있습니다.

4단계 검증 명령:

2026-09-30: 단위 테스트 22개 통과. 별도 Webots 정지 시험에서 CPU 자동 선택,
탐지·지도 Display 연결, 빨간 사과 2개 기록 및 다른 색 3개 제외를 확인했습니다.
두 위치 오차는 각각 약 2.2cm였으며 모두 `appearance_candidate` 판정이었습니다.
이 수치는 시험 배치 한 곳의 결과이며 아파트 전체 주행 정확도를 뜻하지 않습니다.

```sh
python -m unittest discover -s tests -v
python tests/run_perception_simulation.py --webots "C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
```

테스트 전용 `.tmp/perception_sim/worlds/check.wbt`는 자동 종료됩니다. 직접 실행할 때는
항상 저장소의 `worlds/apartment.wbt`를 사용하세요.
