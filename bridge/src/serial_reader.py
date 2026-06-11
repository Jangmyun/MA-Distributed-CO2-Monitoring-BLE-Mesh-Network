"""
serial_reader.py — UART 직렬 포트 읽기 모듈

Node C 펌웨어 UART 출력 형식 (둘 중 하나):

  형식 1 — JSON 한 줄:
    {"nodes":[{"id":"A","x":123,"y":-234,"z":981,"rssi":-45},...],
     "links":[{"src":"A","dst":"B","rssi":-52},...]}

  형식 2 — 단순 ACCEL 라인 (펌웨어 printk 출력):
    ACCEL:C:123,-234,981
    ACCEL:A:50,100,960

파싱 결과:
  형식 1 → {"type": "json",  "data": <원본 dict>}
  형식 2 → {"type": "accel", "data": {"id": "C", "x": 123, "y": -234, "z": 981}}
"""

import asyncio
import json
import logging
import re
import time
from typing import Awaitable, Callable, Optional

# Zephyr UART 셸이 삽입하는 ANSI 이스케이프 시퀀스 제거용
_ANSI_RE = re.compile(r'\x1b\[[0-9;]*[A-Za-z]')

log = logging.getLogger("bridge.serial")

SERIAL_BAUD_DEFAULT = 115200
RECONNECT_DELAY_S   = 3.0

ParsedMessage = dict  # {"type": "json"|"accel", "data": ...}
MessageCallback = Callable[[ParsedMessage], Awaitable[None]]


# ── 파싱 함수 ─────────────────────────────────────────────────────────────────

def parse_accel_line(line: str) -> Optional[ParsedMessage]:
    """'ACCEL:<id>:<x>,<y>,<z>' 형식을 파싱한다.

    예) "ACCEL:C:123,-234,981"
        → {"type": "accel", "data": {"id": "C", "x": 123, "y": -234, "z": 981}}
    """
    try:
        prefix, nid, vals = line.strip().split(":", 2)
        if prefix.upper() != "ACCEL":
            return None
        x, y, z = (int(v) for v in vals.split(","))
        return {"type": "accel", "data": {"id": nid.strip().upper(), "x": x, "y": y, "z": z}}
    except (ValueError, AttributeError):
        return None


def parse_json_line(line: str) -> Optional[ParsedMessage]:
    """JSON 단일 라인을 파싱한다.

    예) '{"nodes":[...],"links":[...]}' → {"type": "json", "data": <dict>}
    """
    try:
        return {"type": "json", "data": json.loads(line)}
    except json.JSONDecodeError:
        log.debug("JSON 파싱 실패: %s", line[:60])
        return None


def strip_ansi(s: str) -> str:
    """ANSI 이스케이프 시퀀스와 Zephyr UART 셸 프롬프트를 제거한다.

    Zephyr UART 셸은 각 출력 라인에 'uart:~$ ' 프롬프트와
    커서 이동 시퀀스(\x1b[8D, \x1b[J 등)를 삽입한다.
    예) '\x1b[1;32muart:~$ \x1b[m\x1b[8D\x1b[JACCEL:C:123,-234,981'
    """
    return _ANSI_RE.sub('', s)


def extract_payload(line: str) -> str:
    """ANSI 제거 후 실제 데이터 부분만 추출한다.

    Zephyr 셸 프롬프트('uart:~$ ') 뒤에 실제 데이터가 위치하므로
    마지막 프롬프트 이후 문자열을 반환한다. 프롬프트가 없으면 전체 반환.
    """
    cleaned = strip_ansi(line).strip()
    # 'uart:~$ ' 프롬프트가 여러 번 나올 수 있으므로 마지막 위치 기준으로 분리
    prompt = "uart:~$ "
    idx = cleaned.rfind(prompt)
    if idx != -1:
        cleaned = cleaned[idx + len(prompt):].strip()
    return cleaned


def parse_line(line: str) -> Optional[ParsedMessage]:
    """UART 한 줄을 파싱해 메시지 dict를 반환한다. 인식 불가 시 None."""
    payload = extract_payload(line)
    if not payload:
        return None
    if payload.startswith("{"):
        return parse_json_line(payload)
    if payload.upper().startswith("ACCEL:"):
        return parse_accel_line(payload)
    return None


# ── SerialReader ──────────────────────────────────────────────────────────────

class SerialReader:
    """비동기 UART 라인 리더.

    포트가 끊어지면 RECONNECT_DELAY_S 초 후 자동 재접속한다.
    각 파싱된 메시지는 on_message 콜백(코루틴)으로 전달된다.
    """

    def __init__(
        self,
        port: str,
        baud: int = SERIAL_BAUD_DEFAULT,
        on_message: Optional[MessageCallback] = None,
    ) -> None:
        self.port       = port
        self.baud       = baud
        self._callback  = on_message

    def set_callback(self, cb: MessageCallback) -> None:
        self._callback = cb

    async def run(self) -> None:
        """이벤트 루프를 블록하지 않고 UART를 읽는다 (스레드 풀 실행)."""
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, self._blocking_loop, loop)

    def _blocking_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        import serial  # type: ignore  (pyserial)

        log.info("시리얼 포트 열기: %s @ %d baud", self.port, self.baud)
        while True:
            try:
                with serial.Serial(self.port, self.baud, timeout=1.0) as ser:
                    log.info("시리얼 연결 완료: %s", self.port)
                    while True:
                        raw = ser.readline()
                        if not raw:
                            continue
                        try:
                            line = raw.decode("utf-8", errors="replace").strip()
                        except Exception:
                            continue

                        log.info("UART ← %r", line)
                        msg = parse_line(line)
                        if msg and self._callback:
                            asyncio.run_coroutine_threadsafe(
                                self._callback(msg), loop
                            )
                        elif line:
                            log.info("UART (미인식) ← %r", line)

            except serial.SerialException as e:
                log.error("시리얼 오류: %s — %.0f초 후 재시도", e, RECONNECT_DELAY_S)
                time.sleep(RECONNECT_DELAY_S)
            except Exception as e:
                log.error("예기치 않은 오류: %s", e)
                time.sleep(RECONNECT_DELAY_S)
