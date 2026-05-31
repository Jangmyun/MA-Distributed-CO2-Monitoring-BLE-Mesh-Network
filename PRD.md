# PRD: BLE Mesh 분산 진동 모니터링 + 펌웨어 FFT 이상 감지 + Three.js 피드백 제어 시뮬레이션

> **과목:** 마이크로프로세서 응용 (기말 프로젝트)
> **버전:** v0.3
> **작성일:** 2026-05-27

---

## 1. 프로젝트 개요

### 1.1 한 줄 요약

세 대의 nRF52840 DK가 BLE Mesh로 MPU-6050 진동 데이터를 분산 수집하고, 각 노드에서 FFT를 수행하여 이상 징후를 감지한다. 감지 결과는 WebSocket을 통해 브라우저로 전달되어 Three.js 3D 모터 시뮬레이터의 회전 속도를 실시간 피드백 제어하며, 동시에 주파수 스펙트럼·토폴로지·제어 응답 타임라인을 단일 웹 대시보드에서 시각화한다.

### 1.2 문제 정의

- 단일 지점 진동 측정은 복수 설비의 공간적 이상 분포를 반영하지 못함
- 기존 시스템은 이상 감지 후 알람에 그치며, 제어 피드백 루프로 이어지지 않음
- 임베디드 FFT + 브라우저 3D 제어 시뮬레이션을 통합한 실증 솔루션이 부족함
- 시간 도메인 임계값 방식은 노이즈에 취약하고 초기 이상 징후를 놓치기 쉬움

### 1.3 목표

- nRF52840 펌웨어에서 256-point FFT 수행 → 주파수 도메인 이상 감지 **[수업 외 기술 ①②③]**
- 이상 감지 결과를 Three.js 3D 시뮬레이터에 전달 → 모터 RPM 실시간 피드백 제어 **[수업 외 기술 ④]**
- 인프라 없이 동작하는 BLE Mesh 기반 자율 센서 네트워크
- 스펙트럼·토폴로지·3D 모터·제어 타임라인을 단일 웹 대시보드에 통합

### 1.4 수업 외 기술 카드

| # | 기술 | 적용 위치 | 근거 |
|---|------|-----------|------|
| ① | CMSIS-DSP `arm_rfft_fast_f32` | nRF52840 펌웨어 | Cortex-M4F HW FPU 활용 256-point RFFT |
| ② | Hanning Window | nRF52840 펌웨어 | 스펙트럼 누설 억제 윈도우 함수 |
| ③ | 주파수 도메인 이상 감지 알고리즘 | nRF52840 펌웨어 | SFM + DFS + RSD 3중 지표 |
| ④ | Three.js 3D 물리 시뮬레이션 | 웹 대시보드 | 브라우저 기반 3D 모터 제어 시뮬레이터 |

### 1.5 평가 기준 대응

| 평가 항목 | 대응 전략 |
|-----------|-----------|
| Novelty | BLE Mesh + 펌웨어 FFT + Three.js 3D 피드백 제어 + D3.js 스펙트럼 — 4개 기술 스택 통합 |
| Completeness | 3노드 실물 배치 + Three.js 이상 시나리오 3종 + 감속/정지/복구 제어 루프 실시간 시연 |
| 수업 외 기술 | CMSIS-DSP ① / Hanning Window ② / 주파수 이상 감지 ③ / Three.js 3D 시뮬레이션 ④ |

---

## 2. 시스템 아키텍처

### 2.1 전체 구성도

```
┌─────────────────────────────────────────────────────────────┐
│                    [임베디드 레이어]                          │
│                                                             │
│  [Node A] ──BLE Mesh── [Node B: Relay] ──BLE Mesh── [Node C: GW] │
│  MPU-6050              MPU-6050                  MPU-6050   │
│  + FFT                 + FFT                     + FFT + 집계│
│  Rich Shield           Rich Shield               Rich Shield │
└──────────────────────────────┬──────────────────────────────┘
                               │ BLE GATT (Custom Service)
                               ▼
┌─────────────────────────────────────────────────────────────┐
│                    [PC 브릿지 레이어]                         │
│                                                             │
│   gateway_bridge.py                                         │
│   ┌─────────────────┐                                       │
│   │  BLE GATT       │  →  WebSocket Server (:8765)          │
│   │  Receiver       │                                       │
│   │  (bleak)        │                                       │
│   └─────────────────┘                                       │
└──────────────────────────────┬──────────────────────────────┘
                               │ WebSocket
                               ▼
┌─────────────────────────────────────────────────────────────┐
│                    [웹 대시보드 — 단일 페이지]                 │
│                                                             │
│  ┌──────────────┐  ┌──────────────┐  ┌────────────────────┐│
│  │ D3.js        │  │ D3.js FFT    │  │ Three.js           ││
│  │ Force Graph  │  │ Spectrum     │  │ 3D Motor Sim       ││
│  │ (토폴로지)    │  │ (스펙트럼)    │  │ (피드백 제어)       ││
│  └──────────────┘  └──────────────┘  └────────────────────┘│
│  ┌──────────────────────────────────────────────────────┐   │
│  │ Control Timeline (RPM 변화 + 이상 이벤트 타임라인)      │   │
│  └──────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

### 2.2 하드웨어 구성

| 노드 | 역할 | 구성 |
|------|------|------|
| Node A | Sensor Node | nRF52840 DK + MPU-6050 (I2C) + Rich Shield |
| Node B | Sensor Node + Relay | nRF52840 DK + MPU-6050 (I2C) + Rich Shield |
| Node C | Gateway (Mesh ↔ GATT) | nRF52840 DK + MPU-6050 (I2C) + Rich Shield |
| PC | 브릿지 + 웹 서버 | Python 3.10 + bleak 0.21 + 정적 HTML 서빙 |

### 2.3 소프트웨어 스택

| 레이어 | 구성 요소 | 버전 / 비고 |
|--------|-----------|-------------|
| 펌웨어 | Zephyr RTOS + nRF Connect SDK | 3.3 / 2.4 |
| 펌웨어 DSP | CMSIS-DSP `arm_rfft_fast_f32` | `CONFIG_CMSIS_DSP=y` — 수업 외 ① |
| BLE | Bluetooth Mesh 1.0.1 + Custom GATT | Vendor Model |
| PC 브릿지 | Python + bleak + asyncio + websockets | BLE → WebSocket 중계 |
| 3D 시뮬레이터 | Three.js r165 | 브라우저 3D 모터 렌더링 + 제어 — 수업 외 ④ |
| 웹 시각화 | D3.js v7 + HTML/CSS/JS | 스펙트럼·토폴로지·타임라인 |

---

## 3. 핵심 기술 ①②③ — 펌웨어 FFT 및 이상 감지

### 3.1 MPU-6050 데이터 수집

- **인터페이스:** I2C 400 kHz Fast Mode (Zephyr sensor API)
- **샘플링 레이트:** 1 kHz (DLPF 설정, 대역폭 260 Hz)
- **측정 축:** Z축 가속도 (회전 설비 수직 진동 성분)
- **버퍼:** 256 샘플 DMA 더블 버퍼링 → 수집과 FFT 병렬 처리

### 3.2 FFT 파이프라인

- **윈도우 함수:** Hanning Window — 스펙트럼 누설 억제 **[수업 외 기술 ②]**
- **FFT:** 256-point RFFT → 128 주파수 빈, 분해능 3.9 Hz, Fs = 1 kHz
- **라이브러리:** `arm_rfft_fast_f32` (CMSIS-DSP, Cortex-M4F HW FPU) **[수업 외 기술 ①]**
- **처리 주기:** ~0.256초 (256 샘플 수집 완료 시마다)

```c
// Zephyr 펌웨어 FFT 파이프라인 (의사 코드)
arm_rfft_fast_instance_f32 fft_inst;
arm_rfft_fast_init_f32(&fft_inst, 256);

apply_hanning_window(buf, 256);              // 수업 외 ②: 누설 억제
arm_rfft_fast_f32(&fft_inst, buf, out, 0);  // 수업 외 ①: CMSIS-DSP RFFT
arm_cmplx_mag_f32(out, mag, 128);            // 복소수 → 진폭
detect_anomaly(mag, baseline, &result);      // 수업 외 ③: 3중 지표 이상 감지
```

### 3.3 이상 감지 알고리즘 [수업 외 기술 ③]

#### 3.3.1 기준선(Baseline) 학습
- 부팅 후 30초간 정상 스펙트럼 수집 → 빈별 평균/표준편차 계산
- Flash에 저장, 버튼 장기 누름으로 재학습 가능

#### 3.3.2 3중 이상 지표

| 지표 | 의미 | 이상 징후 |
|------|------|-----------|
| **SFM** (Spectral Flatness) | 기하평균 / 산술평균 | 상승 → 에너지 분산 → 불균형·마모 |
| **DFS** (Dominant Freq Shift) | 기준선 대비 최대 진폭 주파수 이동량 | > ±15 Hz → 베어링 마모·축 정렬 불량 |
| **RSD** (RMS Spectral Deviation) | 기준선 대비 스펙트럼 L2 거리 | 급증 → 전반적 진동 패턴 급변 |

#### 3.3.3 심각도 판정

| 심각도 | 조건 | Three.js 제어 명령 |
|--------|------|--------------------|
| 🟢 Normal | 3가지 지표 모두 정상 | `setRPM(1200)` — 정상 유지 |
| 🟡 Warning | 1가지 지표 초과 | `setRPM(600)` — 50% 감속 |
| 🔴 Critical | 2가지 이상 초과 | `setRPM(0)` — 즉시 정지 |

---

## 4. 핵심 기술 ④ — Three.js 3D 모터 시뮬레이터

### 4.1 씬 구성

Three.js 씬은 단순화된 **모터-축-디스크 어셈블리**를 3D로 렌더링한다.

| 오브젝트 | Three.js 구현 | 역할 |
|----------|---------------|------|
| 모터 하우징 | `CylinderGeometry` | 고정 바디 |
| 회전축 | `CylinderGeometry` (thin) | 토크 전달 시각화 |
| 디스크 (플라이휠) | `CylinderGeometry` (flat) | 회전 속도 시각화 |
| 불균형 추 | `SphereGeometry` (편심 위치) | 이상 시나리오 시각화 |
| 진동 이펙트 | 카메라 shake + 메시 position jitter | 이상 심각도 표현 |

### 4.2 제어 루프

```javascript
// Three.js 제어 루프 핵심 구조
const ws = new WebSocket('ws://localhost:8765');

ws.onmessage = (event) => {
  const state = JSON.parse(event.data);
  const severity = getWorstSeverity(state.nodes);  // 전 노드 중 최악

  // 제어 정책 적용
  if      (severity === 'critical') motorSim.setRPM(0);
  else if (severity === 'warning')  motorSim.setRPM(600);
  else                              motorSim.setRPM(1200);

  // 시각화 갱신
  updateTopologyGraph(state.nodes, state.links);   // D3.js
  updateSpectrumCharts(state.nodes);               // D3.js
  updateTimeline(severity, motorSim.currentRPM);   // D3.js
};

// Three.js 애니메이션 루프
function animate() {
  requestAnimationFrame(animate);
  motorSim.tick();          // 목표 RPM으로 부드럽게 수렴 (lerp)
  applyVibrationEffect();   // 심각도에 비례한 카메라 흔들림
  renderer.render(scene, camera);
}
```

### 4.3 이상 시나리오 시각화

이상 심각도에 따라 Three.js 씬이 다르게 반응한다.

| 심각도 | 디스크 색상 | 회전 속도 | 진동 이펙트 | 불균형 추 |
|--------|------------|-----------|-------------|-----------|
| 🟢 Normal | 초록 | 1200 RPM | 없음 | 숨김 |
| 🟡 Warning | 노랑 | 600 RPM | 미세 카메라 흔들림 | 표시 (반투명) |
| 🔴 Critical | 빨강 | 0 (정지) | 강한 메시 jitter + 카메라 shake | 표시 (빨강) |

### 4.4 수동 시나리오 주입 (발표용)

실물 진동원이 없어도 브라우저 UI 버튼으로 시나리오를 WebSocket을 통해 브릿지에 주입할 수 있다.

```
[정상 운전] → [불균형 부하 주입] → [베어링 마모 악화] → [복구]
```

브릿지는 수신한 시나리오를 GATT Write로 Gateway에 전달 → 펌웨어 이상 감지 알고리즘이 해당 패턴의 진동 데이터를 생성하여 실제 FFT 경로를 그대로 탄다.

---

## 5. 기능 명세

### 5.1 센서 노드 (Node A, B)

**진동 수집 및 FFT**
- MPU-6050 I2C → 256 샘플 DMA 수집
- Hanning Window + `arm_rfft_fast_f32` + 3중 이상 지표 계산
- 심각도 판정 → Mesh publish

**BLE Mesh 송신 페이로드**
```json
{
  "node_id": "A",
  "severity": "warning",
  "dom_freq": 62.5,
  "sfm": 0.31,
  "rms_dev": 0.18,
  "top8_bins": [0, 0, 0, 980, 0, 520, 0, 0],
  "ts": 1735200000
}
```

**Rich Shield LCD**
- 행 0~1: RMS 진폭 + 심각도 (Normal / WARN / CRIT)
- 행 2~5: 8빈 주파수 스펙트럼 막대 그래프
- 행 6: Mesh 상태 + 마지막 TX 경과
- 행 7: `[1] 모드전환  [2] 기준선재학습`

### 5.2 Gateway 노드 (Node C)

- 전 노드 최신 FFT 결과 + 심각도 집계
- 15초 수신 없으면 offline 처리
- RSSI 기반 링크 품질 추정

**GATT Server Characteristics**

| Characteristic | 권한 | 용도 |
|---------------|------|------|
| Spectrum Aggregation | Notify | 전 노드 집계 JSON 브로드캐스트 |
| Baseline Reset | Write | 특정 노드 기준선 재학습 트리거 |
| Threshold Control | Write | 이상 감지 임계값 원격 조정 |
| Scenario Inject | Write | 발표용 시나리오 수동 주입 |

**Gateway LCD**
```
열: 0123456789012345
행0: A:OK  B:WRN C:OK
행1: ----+-----+-----
행2:  ## | ##  |  ##
행3:  ## |#### |  ##
행4:  ## |#### |  ##
행5: ----+-----+-----
행6: SIM:1200->600RPM
행7: B! DF:62Hz SFM:.31
```

### 5.3 PC 브릿지 (gateway_bridge.py)

BLE GATT 수신, WebSocket 브로드캐스트, 시나리오 주입을 단일 asyncio 루프에서 처리한다.

```python
async def main():
    ble   = await BleakClient(GATEWAY_ADDR).connect()
    ws_sv = await websockets.serve(ws_handler, '0.0.0.0', 8765)

    async def on_notify(_, data):
        state = parse_json(data)
        await ws_broadcast(state)          # 웹 대시보드로 전달

    async def ws_handler(ws):             # 브라우저 → 시나리오 주입
        async for msg in ws:
            cmd = json.loads(msg)
            if cmd['type'] == 'inject_scenario':
                await gatt_write(SCENARIO_CHAR, cmd['scenario'])

    ble.start_notify(SPECTRUM_CHAR_UUID, on_notify)
    await asyncio.gather(ws_sv.wait_closed())
```

### 5.4 웹 대시보드 — 4패널 단일 페이지

**전체 GATT 집계 JSON (WebSocket 페이로드)**
```json
{
  "nodes": [
    {"id":"A","severity":"normal","dom_freq":48.8,"sfm":0.12,
     "rms_dev":0.03,"top8_bins":[0,0,0,1450,0,0,0,0],"rssi":-45,"online":true},
    {"id":"B","severity":"warning","dom_freq":62.5,"sfm":0.31,
     "rms_dev":0.18,"top8_bins":[0,0,0,980,0,520,0,0],"rssi":-67,"online":true},
    {"id":"C","severity":"normal","dom_freq":48.8,"sfm":0.13,
     "rms_dev":0.04,"top8_bins":[0,0,0,1420,0,0,0,0],"rssi":0,"online":true}
  ],
  "links": [
    {"src":"A","dst":"B","rssi":-52},
    {"src":"B","dst":"C","rssi":-48}
  ],
  "ts": 1735200000
}
```

**패널 구성**

| 패널 | 기술 | 내용 |
|------|------|------|
| 좌상 | D3.js | Force-directed 토폴로지 그래프. 노드 색 = 심각도, 엣지 굵기 = RSSI |
| 우상 | Three.js | 3D 모터-축-디스크 어셈블리. RPM 실시간 제어 + 진동 이펙트 |
| 좌하 | D3.js | 3노드 FFT 스펙트럼 바 차트 (기준선 오버레이 포함) |
| 우하 | D3.js | 제어 응답 타임라인 (RPM 꺾은선 + 이상 이벤트 마커) |

**제어 패널 (상단 바)**
- 시나리오 버튼: `[정상 운전]` `[불균형 부하]` `[베어링 마모]` → WebSocket으로 브릿지에 전송
- 임계값 슬라이더 → Threshold Control Characteristic GATT Write

---

## 6. 데이터 흐름

### 6.1 정상 시나리오

```
1. MPU-6050 → 1 kHz 진동 수집
2. 256 샘플 완료 → Hanning Window + RFFT
3. 3중 지표 계산 → Normal 판정
4. BLE Mesh publish (~0.256초 주기)
5. Gateway 집계 → GATT Notify
6. gateway_bridge.py 수신 → WebSocket broadcast
7. 브라우저: Three.js setRPM(1200) + D3.js 스펙트럼·그래프 갱신
```

### 6.2 이상 감지 → Three.js 피드백 제어

```
1. Node B 불균형 시나리오 주입 (버튼 or 실물 진동원)
2. Node B SFM 상승 + DFS 이탈 → Warning 판정
3. GATT Notify → 브릿지 → WebSocket broadcast
4. 브라우저: getWorstSeverity() → "warning"
5. Three.js: setRPM(600) → 디스크 감속 애니메이션
6. 진동 이펙트: 미세 카메라 흔들림 시작
7. 타임라인: RPM 1200→600 하강 기록
8. 상황 악화 → Critical → setRPM(0) → 즉시 정지 + 강한 shake
9. 심각도 복귀 → setRPM(1200) 자동 재가동
```

### 6.3 노드 장애 시나리오

```
1. Node B 전원 차단 → 15초 timeout → offline 처리
2. Force Graph: Node B 회색 + 점선
3. Node A → Node C 직접 연결 시도 → 토폴로지 동적 변경
```

---

## 7. 개발 일정 (4주)

| 주차 | 작업 |
|------|------|
| **W1** | MPU-6050 I2C 드라이버 + 1 kHz 수집 + LCD 표시. CMSIS-DSP 빌드 환경 구성 (`CONFIG_CMSIS_DSP=y`). Three.js 씬 기본 구조 (모터-축-디스크 렌더링 + 회전 애니메이션) |
| **W2** | FFT 파이프라인 + 이상 감지 알고리즘 구현 및 단일 노드 검증. BLE Mesh Provisioning + Vendor Model 3노드 송수신. WebSocket 브릿지 기본 연결 |
| **W3** | gateway_bridge.py 완성 (BLE GATT → WebSocket). Three.js 제어 루프 + 심각도별 시각 반응. D3.js 스펙트럼 바 차트 + Force Graph |
| **W4** | 제어 응답 타임라인 + 시나리오 주입 버튼 통합. LCD UI 다듬기. 데모 리허설 (이상 주입 → 감속/정지 → 복구 전체 루프) |

---

## 8. 데모 시나리오 (발표 5분)

1. **인트로 (30초):** 3노드 배치 설명. 브라우저에서 대시보드 4패널 (토폴로지·Three.js 모터·스펙트럼·타임라인) 동시 표시
2. **정상 운전 (1분):** 모터 1200 RPM. 3노드 모두 녹색. 단일 주파수 피크 스펙트럼 확인
3. **불균형 이상 유도 (1분):** `[불균형 부하]` 버튼 클릭 → Node B Warning → Three.js 모터 600 RPM 감속 + 노랑·카메라 흔들림
4. **베어링 마모 악화 (45초):** `[베어링 마모]` 버튼 → Critical → Three.js 즉시 정지 + 빨강 + 강한 shake. 버저 알람
5. **자동 복구 (30초):** `[정상 운전]` 복귀 → setRPM(1200) 재가동. 타임라인 RPM 상승 확인
6. **노드 장애 (30초):** Node B 전원 차단 → 토폴로지 변경 시각화
7. **마무리 (15초):** FFT 이상 감지 → 3D 피드백 제어 루프 의의, 한계, 확장성

---

## 9. 위험 요소 및 대응

| 위험 | 대응책 |
|------|--------|
| CMSIS-DSP 빌드 통합 실패 | W1에서 단독 빌드 검증 확보. 실패 시 float 배열 DFT 직접 구현으로 fallback |
| BLE GATT → WebSocket 지연 >200ms | 제어 응답 요구사항 완화 (데모용 허용). 실시간성 한계는 마무리에서 명시 |
| 진동원 부재 (모터 미보유) | 대시보드 시나리오 버튼으로 완전 대체 가능 (브릿지 → GATT Write → 펌웨어 패턴 생성) |
| Three.js 3D 렌더링 성능 | 폴리곤 수 최소화 (단순 원통 형상). 저사양 맥북에서도 60fps 유지 가능 |
| Mesh PDU 크기 제한 (≤384 bytes) | 전체 128빈 대신 이상 지표 수치 + top 8 빈만 전송 (<40 bytes) |
| 기준선 학습 중 노이즈 포함 | 30초 수집 후 중앙값 필터링 → 이상치 제거 후 기준선 저장 |

---

## 10. 확장 가능성 (Future Work)

- **TinyML 연동:** Edge Impulse로 이상 유형 자동 분류 (불균형 / 베어링마모 / 축 정렬 불량)
- **Three.js 고도화:** 복수 모터 어셈블리 + 설비 배치도 시뮬레이션
- **클라우드 연동:** MQTT 브릿지 → InfluxDB + Grafana 대시보드
- **GPU 환경:** Isaac Sim으로 전환 시 고충실도 물리 시뮬레이션 가능
- **추가 센서 융합:** 온도(NTC) + 전류(ACS712)로 복합 이상 진단

---

## 11. 참고 자료

- Bluetooth Mesh Profile Specification 1.0.1
- Nordic nRF Connect SDK – Bluetooth Mesh samples
- ARM CMSIS-DSP Software Library – `arm_rfft_fast_f32` API Reference
- InvenSense MPU-6050 Product Specification Rev 3.4
- Three.js Documentation r165 – `CylinderGeometry`, `AnimationMixer`, `Clock`
- D3.js Force Simulation API Documentation
- Python bleak Library Documentation (v0.21)
- Randall J. Allemang, Donald J. Ewins – *Structural Dynamics Fundamentals*
