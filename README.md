# PNU Robot Hackathon — Autonomous Search & Return

Webots R2025a / TurtleBot3 Burger. 사전 지도·사과 위치 없이 탐색하고 빨간 사과 두 개를
각각 방문한 뒤 출발점으로 복귀하는 MVP입니다. 물리적인 집게 수거와 강화학습은 포함하지 않습니다.

**검증 범위: 작은 시험 월드에서 두 사과 방문·복귀까지 통과했습니다. 아파트 전체 임무 성공은
아직 검증하지 못했습니다.** 아파트 주행에서 탐지 누락이 의심되는 구간이 있어 추가 확인이 필요합니다.
`worlds/mission_demo.wbt`는 통과한 시험 배치의 데모이며 자동 종료 관찰기 없이 실행됩니다.

## 심사위원 실행 안내

1. 프로젝트 전체를 내려받거나 제출 ZIP을 압축 해제합니다.
2. Python 가상환경을 만들고 **Webots에서 사용할 동일한 Python**에 의존성을 설치합니다.

   ```powershell
   py -m venv .venv
   # GPU 없는 Windows 랩탑:
   & .\.venv\Scripts\python.exe -m pip install torch==2.7.1 torchvision==0.22.1 --index-url https://download.pytorch.org/whl/cpu
   # RTX 5070 데스크톱은 위 명령 대신 CUDA 12.8 빌드:
   # & .\.venv\Scripts\python.exe -m pip install torch==2.7.1 torchvision==0.22.1 --index-url https://download.pytorch.org/whl/cu128
   & .\.venv\Scripts\python.exe -m pip install -r controllers/apple_collector/requirements.txt
   & .\.venv\Scripts\python.exe -c "import sys,torch; print(sys.executable); print(torch.cuda.is_available())"
   ```

   Linux는 `python3 -m venv .venv` 후 `.venv/bin/python`으로 같은 pip 명령을 실행합니다.
   `controller` pip 패키지는 설치하지 않습니다. Webots가 제공하는 모듈입니다.
3. Webots Preferences의 Python command를 출력된 `sys.executable` 경로로 설정합니다.
   **경로는 PC마다 다릅니다.** 학교 데스크톱의 `C:\Users\user\.venv\Scripts\python.exe`를
   랩탑에 복사하면 안 됩니다. 기존 학교 가상환경을 사용해도 됩니다.
4. 확인용 **`worlds/mission_demo.wbt`** 또는 실제 과제 월드 **`worlds/apartment.wbt`**를 열고
   **실시간 실행**합니다. 컨트롤러는 `apple_collector`입니다. 아파트 전체 성공은 아직 검증 전입니다.
   최초 로딩은 외부 Webots PROTO/텍스처를 받느라 오래 걸릴 수 있으므로 인터넷이 필요합니다.
5. 모델 로딩 동안 대기합니다. `[vision] running device=cpu` 또는 `device=0`과 임무 로그를 확인합니다.
6. View → Overlays에서 로봇의 `map`, `detections`를 켭니다. 원본 camera와 가구의 display는
   별도 화면이므로 필요하지 않으면 끕니다.

갱신은 프로젝트 폴더에서 `git pull` 후 같은 Python으로 requirements를 설치합니다.
실행 중 월드 상태는 저장하지 않고 원본 월드를 다시 열어 처음부터 실행합니다.
테스트용 `.tmp/**/check.wbt`는 자동 종료되므로 일반 실행에 사용하지 않습니다.

## 임무와 완료 기준

`SCAN → SEARCH → APPROACH → VERIFY → (다음 사과 탐색) → RETURN → SUCCEEDED`

- 회전 관측, 미탐색 Frontier 탐색, 알려진 영역의 미관측 지점 재방문으로 목표를 찾습니다.
- YOLO11n 후보에 빨간색·형태 조건을 적용하고 반복 관측을 지도에 기록합니다.
- A*로 접근 지점까지 이동하고 약 40~45cm 앞에서 정지·방향 정렬합니다.
- 도착 후 촬영한 새 영상에서 거리 추정 55cm 이내의 동일 목표를 3회 이상, 0.3초 이상
  확인해야 방문 완료로 기록합니다. 이는 **MVP 자체 기준**이며 공식 심사 거리 규정은 아닙니다.
- 두 개 방문 후 출발점으로 복귀합니다. 추정 출발점 16cm 이내 정지까지 해야 `SUCCEEDED`입니다.
- 목표 미발견·접근/확인 불가는 복귀 후 `INCOMPLETE`, 복귀 경로 불가·정체는 `FAILED`입니다.
  집에 돌아왔다는 이유만으로 성공 처리하지 않습니다.
- **기본 180초 제한은 제거했습니다.** 명시적으로 `--explore-seconds=600` 등을 준 경우에만
  임무 시작 후 해당 시간에 미완료 복귀합니다. 대회 시간 제한을 임의로 가정하지 않습니다.

## 조작과 화면

Webots 3D 화면을 클릭한 상태에서 사용합니다.

| 키 | 동작 |
|---|---|
| N | 임무 시작·재개, 기존 방문 기록 유지 |
| H | 임무 중단 후 출발점 복귀 요청 |
| M | 자동 주행 중단, 수동 대기 |
| W / S / A / D | 누르는 동안 전진 / 후진 / 좌회전 / 우회전 |
| Space / X | 비상 정지 유지 |
| R | 비상 정지 해제 후 수동 대기 |
| T | 1단계 센서·전진·회전 점검 |

`map`: 회색 미탐색, 흰색 빈 공간, 검정 장애물, 보라 이동 흔적, 파랑 계획 경로,
초록 출발점, 빨강 로봇, 주황 사과 후보. 상단에 임무 상태와 `visited n/2`를 표시합니다.
`detections`: 마지막 처리 영상과 YOLO 원래 분류·점수·처리 장치·반복 확인한 후보 수입니다.
**후보 수와 방문 완료 수는 다릅니다.** 처리 영상은 원본 카메라보다 늦게 표시됩니다.

Robot의 `controllerArgs` 옵션:

| 옵션 | 용도 |
|---|---|
| `--manual` | 자동 출발 없이 대기 |
| `--device=cpu` / `--device=0` | 추론 장치 강제 지정 |
| `--explore-seconds=600` | 선택적 임무 시간 제한 |
| `--no-vision --explore-seconds=60` | 사과 임무 없는 이동·복귀 진단 |
| `--checkout` | 1단계 센서·모터 점검 |

자동 선택은 CUDA 사용 가능 여부와 초기 추론을 확인하고 실패하면 CPU로 전환합니다.
GPU는 영상 추론에 사용하며 지도·계획·제어는 CPU에서 수행합니다. 추론은 별도 프로세스입니다.
가속 시뮬레이션에서는 영상 지연으로 확인에 실패할 수 있으므로 데모는 실시간 모드를 사용하세요.

## 센서·알고리즘과 한계

- 필수: 360도 2D LiDAR, wheel encoder. camera는 객체 인식과 객체 상대 위치 추정에 사용합니다.
- gyro가 있으면 회전 추정에 사용하며 accelerometer는 진단 로그만 남깁니다.
- compass, GPS/GNSS, Recognition API, Supervisor 실제 좌표, 사전 지도·목표 좌표는 사용하지 않습니다.
- 엔코더/gyro + 국소 scan matching, LiDAR 격자 지도, Frontier, A*, 짧은 궤적 평가·안전 정지를
  조합합니다. 전역 루프 폐쇄 SLAM이 아니므로 긴 주행에서는 위치 오차가 누적될 수 있습니다.
- 알려진 사과 주변은 경로에서 제외하지만 미발견 저상 물체·단차는 LiDAR에 보이지 않을 수 있습니다.
- COCO 사전학습 YOLO11n은 Webots 사과를 공·원반·오렌지로 오분류하기도 합니다.
  이 후보에 색상·형태 보완 판정을 적용하며 아직 별도 학습한 모델은 아닙니다.
- 알려진 사과 지름 약 10cm와 영상 크기로 거리를 추정합니다. 가림·조명·자세 변화에 영향을 받습니다.
- 35cm 내 관측을 동일 대상으로 연결하므로 가까이 붙은 사과의 병합이나 위치 오차에 따른 중복이
  생길 수 있습니다. 규칙 보완은 일반 환경의 빨간 공과 사과를 완벽히 구분하지 못합니다.
- 닫힌 문·통과 불가 통로 뒤까지 탐색을 보장하지 않으며, 탐색 경로 전체의 최단성도 보장하지 않습니다.
  A*는 현재 지도에서 선택한 접근 지점까지의 경로를 계획합니다.

## 파일과 제출

| 파일 | 역할 |
|---|---|
| `controllers/apple_collector/apple_collector.py` | 센서·모터·키보드·안전 정지·실행 연결 |
| `controllers/apple_collector/navigation.py` | 위치 추정, 지도, Frontier, A*, 경로 추종 |
| `controllers/apple_collector/perception.py` | YOLO 프로세스, 색상 판별, 좌표 추정, 추적 |
| `controllers/apple_collector/mission.py` | 탐색·접근·방문 확인·복귀 상태 관리 |
| `controllers/apple_collector/models/yolo11n.pt` | 실행 모델 가중치 |
| `controllers/apple_collector/requirements.txt` | Python 의존성 |
| `worlds/apartment.wbt`, `worlds/mission_demo.wbt`, `protos/` | 과제·데모 월드와 사과 정의 |

**컨트롤러 제출은 `.py` 하나가 아니라 `controllers/apple_collector` 폴더 전체**입니다.
심사위원 재현용으로는 월드·protos·README를 포함한 전체 프로젝트 ZIP을 권장합니다.

```sh
python scripts/build_submission.py
```

`dist/PNU-Robot-MVP.zip`을 생성합니다. 로그·가상환경·테스트 관찰기는 제외하고 SHA256 목록을
포함합니다. 외부 Webots 자산과 Python 패키지는 설치·최초 실행 시 별도로 필요합니다.

`controllers/apple_collector/logs/` 기록은 재실행하면 덮어씁니다:

- `mission_latest.json`: 임무 상태, 방문 기록, 미완료 사유.
- `targets_latest.json`: 후보 ID·좌표·관측 횟수·출처. `yolo_apple` / `appearance_candidate` 구분.
- `navigation_latest.jsonl`: 센서·주행·임무 전환·탐지 상세 기록.
- `map_latest.png`, `stage1_camera.png`: 지도와 초기 카메라 진단 영상.

confidence는 원래 YOLO 클래스 점수이며 사과일 확률이 아닙니다.

## 검증

```sh
python -m unittest discover -s tests -v
python tests/run_mission_simulation.py --webots "C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
python tests/run_mission_simulation.py --webots "C:\Program Files\Webots\msys64\mingw64\bin\webots.exe" --apartment
```

실제 좌표·접촉을 읽는 Supervisor는 별도 테스트 프로젝트 관찰기에만 사용합니다.
제출 컨트롤러에는 해당 권한이 없습니다. 시험 결과는 `.tmp/mission_*/result.json`에 저장됩니다.
기존 4단계 정지 시험에서는 빨간 사과 2개와 다른 색 3개를 구분하고 위치 오차 약 2.2cm를
기록했습니다. 이 수치는 정지 시험 배치 결과이며 아파트 전체 정확도를 뜻하지 않습니다.
학교 RTX 5070 실행은 별도 확인이 필요합니다.

2026-09-30 5·6단계 검증: 단위 테스트 32개 통과. 작은 Webots 시험 월드에서
두 사과 방문 확인·복귀·정지까지 통과했습니다. 임무 시작 후 약 72초(시뮬레이션 시간),
실제 복귀 오차 약 16.2cm, 지면 위 장애물 접촉 0회였습니다. CPU 자동 선택으로 실행했습니다.
원본 시험 결과는 `tests/results/mission_smoke.json`에 있습니다.
