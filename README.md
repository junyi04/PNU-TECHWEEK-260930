<br>

<h1 align="center">
  2026 부산대학교 TECH WEEK:<br>
  Autonomous Mobile Robot의 Search & Rescue Mission
</h1>
<br>

---

<br><br>

![](부산대_TECHWEEK_Physical_AI.png)

## 현재 구현: 2·3단계 탐색·복귀 MVP

Webots R2025a / TurtleBot3 Burger. 출발점을 `(0, 0, 0)`으로 삼아 센서로
위치를 추정하고, LiDAR 지도를 만들며 Frontier 탐색 및 A* 경로 계획을 수행합니다.
기본값은 **시뮬레이션 시간 180초 탐색 후 출발점 복귀**입니다.
빨간 사과 인식, YOLO, 강화학습은 아직 구현하지 않았습니다.

### 실행

1. 이 저장소 전체를 내려받습니다. 컨트롤러만 옮길 경우
   `controllers/apple_collector` 폴더 전체가 필요합니다 (`navigation.py` 포함).
2. **Webots가 사용하는 Python 환경**에서 아래 명령을 실행합니다.

   ```sh
   python -m pip install -r controllers/apple_collector/requirements.txt
   ```

3. Webots에서 `worlds/apartment.wbt`를 열고 실시간 실행 버튼을 누릅니다.
   월드에는 `apple_collector` 연결과 `map` Display가 설정되어 있습니다.
4. 코드 변경 후에는 실행 중 월드의 변경 사항을 저장하지 않고 파일을 다시 열어 실행합니다.

현재 검증 환경은 Windows / Webots R2025a / Python 3.13.3 / NumPy 2.2.6입니다.
이 단계에는 CUDA나 GPU, OpenCV가 필요하지 않습니다. `controller`는 Webots가 제공하는
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
- camera는 연결 확인 및 첫 영상 저장만 합니다. 위치 추정에는 사용하지 않습니다.
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
