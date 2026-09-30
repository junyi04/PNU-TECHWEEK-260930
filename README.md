# FIRST-IN — 소방관보다 먼저 들어가는 수색 로봇

> 사람이 들어가기 어려운 집 안으로 먼저 진입해, 구조 대상을 확인하고 출발점으로 돌아옵니다.

**PNU Robot Hackathon | Webots R2025a · TurtleBot3 Burger · YOLO11n · A***

## 프로젝트 소개

주택 화재가 발생했지만, 내부 상황과 구조 대상자의 위치를 알 수 없습니다. 소방관이 곧바로 진입하기 어려운 상황에서 작은 로봇을 먼저 보내 실내를 살펴볼 수 있다면 어떨까요?

FIRST-IN은 이러한 상황을 배경으로 한 **실내 자율 수색·복귀 프로젝트**입니다. 로봇은 사전 지도 없이 주변을 관측하며 이동하고, 카메라로 수색 대상을 찾아 가까이 접근해 확인합니다. 현장 밖의 운용자는 지도와 탐지 영상을 보며 진행 상황을 확인하고, 필요하면 직접 조종하거나 복귀를 요청할 수 있습니다.

임무의 목표는 대상을 발견하는 데서 끝나지 않습니다. **두 대상을 확인한 로봇이 출발점으로 돌아오는 것**까지를 하나의 작전으로 설계했습니다.

### 과제와 시나리오의 연결

| Webots 과제 요소 | 구조 시나리오에서의 의미 |
|---|---|
| 아파트 월드 | 소방관이 진입하기 어려운 실내 현장 |
| TurtleBot3 Burger | 선행 수색 로봇 |
| 빨간 사과 2개 | 위치를 확인해야 하는 구조 대상 표식 |
| 다른 색 사과 | 수색 대상이 아닌 물체 |
| 시작 위치 | 로봇을 투입하고 회수하는 지점 |
| 지도·탐지 화면 | 현장 밖 운용자가 확인하는 상황 정보 |

실제 구현은 **빨간 사과 표식의 탐지·접근 확인·출발점 복귀**입니다. 사람 인식, 사람이나 사과의 물리적 운반, 화재·연기 감지 및 소화 기능은 포함하지 않습니다. 화재는 프로젝트의 활용 시나리오이며 현재 로봇이 불의 위험도를 판단하는 것은 아닙니다.

## 심사위원 실행 안내

### 1. 실행 환경 준비

- **Webots R2025a**
- **Python 3.13, 64비트**
- 프로젝트 전체 폴더 또는 제출 ZIP
- 패키지 설치와 외부 Webots 자산 로딩을 위한 인터넷 연결

GPU 없이 CPU로 실행할 수 있습니다. NVIDIA GPU를 사용하는 경우 YOLO 영상 추론에 GPU를 사용하며, 지도 작성·경로 계획·모터 제어는 CPU에서 처리합니다.

제출 ZIP을 압축 해제한 뒤, `README.md`와 `controllers`, `worlds`, `protos`가 보이는 **프로젝트 최상위 폴더에서 PowerShell**을 엽니다. GitHub에서 받는 경우에는 다음과 같이 준비합니다.

```powershell
git clone https://github.com/junyi04/PNU-TECHWEEK-260930.git
cd PNU-TECHWEEK-260930
```

### 2. Python 환경 설치

```powershell
py -3.13 -m venv .venv
```

아래 두 가지 중 **실행할 PC에 맞는 한 가지**를 선택합니다.

**CPU 실행:**

```powershell
& .\.venv\Scripts\python.exe -m pip install torch==2.7.1 torchvision==0.22.1 --index-url https://download.pytorch.org/whl/cpu
```

**NVIDIA RTX 5070 실행 — CUDA 12.8 빌드:**

```powershell
& .\.venv\Scripts\python.exe -m pip install torch==2.7.1 torchvision==0.22.1 --index-url https://download.pytorch.org/whl/cu128
```

선택한 설치가 끝나면 공통 패키지를 설치하고 Python 경로를 확인합니다.

```powershell
& .\.venv\Scripts\python.exe -m pip install -r controllers/apple_collector/requirements.txt
& .\.venv\Scripts\python.exe -c "import sys, torch; print('Python:', sys.executable); print('CUDA:', torch.cuda.is_available()); print('Device:', torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'CPU')"
```

GPU 환경에서는 `CUDA: True`와 GPU 이름이 출력되는지 확인합니다. CPU 환경에서는 `CUDA: False`가 정상입니다. YOLO11n 가중치는 프로젝트에 포함되어 있습니다.

**`controller`라는 pip 패키지를 별도로 설치하지 마세요.** 로봇 제어용 `controller` 모듈은 Webots가 제공합니다.

### 3. Webots에 Python 연결

1. Webots에서 **Tools → Preferences**를 엽니다.
2. **Python command** 항목에 앞 단계에서 출력된 Python 실행 파일의 전체 경로를 입력합니다.
3. 설정을 적용합니다.

예를 들어 프로젝트가 `C:\PNU-TECHWEEK-260930`에 있다면 다음 경로입니다.

```text
C:\PNU-TECHWEEK-260930\.venv\Scripts\python.exe
```

반드시 **Webots를 실행하는 PC의 경로**를 사용합니다. 다른 PC의 Python 경로를 복사해서 넣거나, 컨트롤러 파일을 일반 Python 명령으로 직접 실행하지 않습니다.

### 4. 아파트 월드 실행

1. **File → Open World**에서 `worlds/apartment.wbt`를 엽니다.
2. 최초 실행 시 외부 PROTO와 텍스처 로딩이 끝날 때까지 기다립니다.
3. Scene Tree의 `TurtleBot3Burger` 노드에서 `controller`가 **`apple_collector`**인지 확인합니다. 제출 월드에는 이미 연결되어 있습니다.
4. **실시간 실행 모드**로 시뮬레이션을 시작합니다.
5. 모델 로딩이 끝나면 자동으로 수색을 시작합니다. 콘솔의 `[vision] running device=cpu` 또는 `device=0`과 임무 상태를 확인합니다.

시연은 실시간 모드를 사용해 주세요. 고속 실행은 카메라 추론보다 시뮬레이션이 앞서 진행되어 대상 확인이 지연될 수 있습니다.

간단한 배치에서 조작과 화면을 먼저 살펴보려면 `worlds/mission_demo.wbt`를 열 수 있습니다. 실제 과제 월드는 `worlds/apartment.wbt`입니다. `.tmp` 폴더 아래의 월드는 일반 실행용이 아닙니다.

### 5. 지도와 탐지 화면 열기

**View → Overlays**에서 로봇의 `map`과 `detections` 화면을 표시합니다. 가구의 display나 원본 camera 화면은 필요한 경우에만 켭니다.

| 화면 | 확인할 내용 |
|---|---|
| `map` | 탐색한 공간, 장애물, 이동 흔적, 계획 경로, 출발점과 대상 후보 |
| `detections` | 처리된 카메라 영상, 탐지 박스, 모델 분류와 점수, CPU/GPU 사용 정보 |
| 컨트롤러 콘솔 | 현재 모드, 센서 값, 탐지·방문·복귀 진행 상황 |

지도는 회색이 미탐색 공간, 흰색이 빈 공간, 검정이 장애물입니다. 보라색은 이동 흔적, 파란색은 계획 경로, 초록색은 출발점, 빨간색은 로봇, 주황색은 사과 후보를 나타냅니다.

방문 완료한 대상은 초록색 `visited`로 표시합니다. 방문 위치 65cm 이내에서 새 ID로
관측된 대상은 같은 사과의 위치 추정 오차일 수 있어, 별개라는 증거가 없으면 회색
`recheck`로 표시하고 새 방문으로 집계하지 않습니다. 같은 영상에서 분리된 두 대상으로
관측된 경우에는 가까워도 별개로 처리합니다. 이 보수적 판정 때문에 가까운 두 대상이
함께 보이지 않으면 두 번째 대상 확인이 지연될 수 있습니다.

**`visited 2/2`는 두 대상의 방문 확인을 마쳤다는 뜻입니다. 복귀까지 완료된 상태는 `SUCCEEDED`입니다.** 화면에 보이는 탐지 후보 수와 방문 완료 수는 서로 다릅니다.

## 작전 진행 방식

1. **현장 관측** — LiDAR로 주변 구조를 읽고 카메라로 수색 대상을 살펴봅니다.
2. **자율 수색** — 현재 지도에서 도달 가능한 미탐색 영역을 선택하고 A* 경로로 이동합니다.
3. **대상 접근** — 빨간 사과 후보를 발견하면 접근 가능한 위치로 이동합니다.
4. **방문 확인** — 가까운 거리에서 새 카메라 영상으로 반복 확인한 대상을 방문 기록에 추가합니다.
5. **추가 수색** — 한 개만 확인했다면 나머지 대상을 계속 찾습니다. 탐색 후보가 소진되면 알려진 공간을 다시 관측합니다.
6. **복귀** — 두 개를 확인하면 출발점으로 이동합니다. 복귀가 막히면 주변 관측과 경로 계산을 재시도합니다.

주행 경로가 갑자기 막히면 최대 3초간 목적지를 유지하며 통로를 재확인합니다.
빈 공간이 연속 관측되면 장애물 기록을 갱신하고 진행하며, 계속 막혀 있으면 재계획합니다.
이는 일시 장애물 대응이며 사람의 종류나 이동 속도를 분류·예측하는 기능은 아닙니다.
관측되지 않거나 다른 물체에 가려진 장애물은 시간이 지났다는 이유만으로 지우지 않습니다.

같은 영상의 겹치는 탐지 박스와 연속 관측의 좌표 변화를 함께 처리해 중복 대상 등록을 줄이고,
방문 확정 직전에도 기존 방문 기록을 확인합니다. 복귀 시에는 출발점에서 잠시 멀어지는
과거 경유점도 고려하되 현재 지도에서 안전한 경로로 연결되는 경우에만 사용합니다.

방문 확인은 추정 거리 55cm 이내에서 새로운 영상으로 3회 이상, 0.3초 이상 확인하는 자체 기준을 사용합니다. 복귀는 **추정 출발점 반경 16cm 이내**에서 정지하는 기준입니다. 시뮬레이터의 실제 좌표로 순간이동하거나 위치를 맞추는 방식은 아닙니다.

기본 임무 시간 제한은 없습니다. 안전한 이동 경로가 없으면 일시 정지하면서 다시 계획할 수 있습니다. 위치 추정 오차, 가려진 대상, 낮은 장애물 등에 따라 수색과 복귀가 지연되거나 완료되지 않을 수 있으며, 모든 아파트 실행의 성공을 보장하지는 않습니다.

## 운용자 조작

**Webots 3D 화면을 클릭한 상태에서** 키를 누릅니다.

| 키 | 기능 |
|---|---|
| **N** | 자동 임무 시작·재개, 기존 방문 기록 유지 |
| **H** | 수색 중단 후 출발점으로 복귀 요청 |
| **M** | 자동 주행 중단, 수동 대기 |
| **W / S** | 누르는 동안 전진 / 후진 |
| **A / D** | 누르는 동안 좌회전 / 우회전 |
| **Space / X** | 비상 정지 |
| **R** | 비상 정지 해제 후 수동 대기 |

비상 정지 후 자동 임무를 다시 진행하려면 **R → N** 순서로 누릅니다. `H`로 조기 복귀하거나 탐지 장애로 복귀한 경우, 두 대상을 모두 확인하지 않았다면 결과는 `INCOMPLETE`로 표시됩니다.

처음부터 다시 실행하려면 시뮬레이션을 멈추고 **실행 중인 월드 상태를 저장하지 않은 채 원본 `worlds/apartment.wbt`를 다시 엽니다.** 컨트롤러만 재시작하면 현재 로봇 위치가 새 출발점이 될 수 있으므로, 처음 배치로 돌아갈 때는 월드 전체를 다시 여세요.

## 구현 구성

| 구성 | 역할 |
|---|---|
| 360도 2D LiDAR | 주변 거리 측정, 장애물 지도 작성, 주행 여유 공간 확인 |
| Wheel encoder + gyro | 이동량과 회전량을 이용한 상대 위치 추정 |
| Camera + YOLO11n | 객체 후보 탐지, 빨간색·형태 보완 판별, 대상 상대 위치 추정 |
| Frontier 탐색 | 미탐색 공간으로 이어지는 관측 지점 선택 |
| A* 경로 계획 | 현재 안전 지도에서 목적지까지 이동 경로 계산 |
| 임무 상태 관리 | 관측·수색·접근·확인·복귀의 전환 및 재시도 |

GPS/GNSS, compass, 사전 지도, 미리 입력한 사과 위치, Recognition API, Supervisor 실제 좌표를 제출 컨트롤러에서 사용하지 않습니다. Accelerometer는 진단 기록에 사용합니다.

현재 제어는 **학습된 YOLO 탐지 모델과 규칙 기반 탐색·주행의 조합**이며 강화학습은 사용하지 않습니다. A*는 현재 지도에서의 이동 경로를 계산하고, 아직 모르는 공간까지 포함한 전체 수색 순서의 최단성을 보장하지는 않습니다.

## 실행 중 확인 사항

| 상황 | 확인 방법 |
|---|---|
| `Python was not found` 또는 모듈 오류 | Webots의 Python command가 패키지를 설치한 `.venv`의 실행 파일인지 확인 |
| GPU가 사용되지 않음 | 동일한 `.venv`로 CUDA 확인 명령 실행. GPU가 없어도 CPU로 실행 가능 |
| 월드가 늦게 열림 | 최초 외부 자산 로딩과 인터넷 연결 확인 |
| 지도·영상이 보이지 않음 | 모델 로딩 완료 후 View → Overlays에서 `map`, `detections` 활성화 |
| 로봇이 움직이지 않음 | 시뮬레이션 재생 상태와 콘솔 확인. 비상 정지 상태라면 R → N |
| 사과가 보이지만 방문 수가 늘지 않음 | 단순 탐지는 방문 완료가 아님. 접근 후 새 영상에서 근접 확인 필요 |
| 복귀가 오래 걸림 | `RETURN` 또는 `RECOVER_RETURN` 상태와 지도 경로 확인. 막힌 경로는 재계산 |

최근 임무 상태는 다음 파일로 확인할 수 있습니다. 로그는 다음 실행 시 덮어써집니다.

```powershell
Get-Content .\controllers\apple_collector\logs\mission_latest.json
Get-Content .\controllers\apple_collector\logs\navigation_latest.jsonl -Tail 1
```

## 제출 파일 안내

| 경로 | 내용 |
|---|---|
| `controllers/apple_collector/apple_collector.py` | Webots 센서·모터·조작·실행 연결 |
| `controllers/apple_collector/navigation.py` | 위치 추정, 지도 작성, 탐색, 경로 계획·추종 |
| `controllers/apple_collector/perception.py` | 영상 추론, 색상·형태 판별, 대상 추적 |
| `controllers/apple_collector/mission.py` | 수색·대상 확인·복귀 임무 관리 |
| `controllers/apple_collector/models/yolo11n.pt` | 객체 탐지 모델 가중치 |
| `controllers/apple_collector/requirements.txt` | Python 의존성 |
| `worlds/apartment.wbt` | 과제 실행 월드 |
| `worlds/mission_demo.wbt` | 간단한 배치의 시연 월드 |
| `protos/` | 월드에서 사용하는 객체 정의 |

**컨트롤러는 `controllers/apple_collector` 폴더 전체를 제출합니다.** 심사위원이 동일한 월드를 열 수 있도록 `worlds`, `protos`, 이 README를 함께 전달하고 폴더 구조를 유지합니다.

제출용 ZIP은 프로젝트 최상위 폴더에서 다음 명령으로 생성합니다.

```powershell
& .\.venv\Scripts\python.exe scripts/build_submission.py
```

생성 파일은 **`dist/PNU-Robot-MVP.zip`**입니다. Python 가상환경과 실행 로그는 포함되지 않으므로, 다른 PC에서는 위 실행 안내에 따라 환경을 준비합니다.
