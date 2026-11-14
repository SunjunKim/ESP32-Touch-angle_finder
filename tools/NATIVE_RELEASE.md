# Axicube 0.0.2 — Python 설치 없는 업데이트

이 ZIP은 사전 컴파일 펌웨어와 Espressif 공식 esptool 5.1.0 실행 파일을 포함한다.
**Python·Arduino IDE·라이브러리 설치, 인터넷 연결, 소스 빌드가 필요 없다.**
esptool 자체에는 Python 런타임이 포함되어 있지만 별도 설치하지 않는다.

1. 자기 OS/CPU용 ZIP을 쓰기 가능한 폴더에 압축 해제한다.
2. 데이터 USB 케이블로 Waveshare ESP32-S3-Touch-LCD-1.69를 연결하고 시리얼 모니터를 닫는다.
3. 압축 해제한 폴더의 터미널에서 실행한다.

   macOS: `bash update.command` (Apple Silicon은 arm64, Intel은 amd64 패키지)

   Windows x64: `.\update.cmd`

4. 후보 기기가 맞는지 확인하고 `UPDATE`를 입력한다. 리셋 → 쓰기/검증 → 재부팅을 요청한다.
5. 화면의 버전, 터치, 기존 CAL과 영점을 확인한다. 앱 부팅 성공은 자동 검증하지 않는다.

탐색만: macOS `bash update.command --list`, Windows `.\update.cmd -List`.
검사만: macOS `--dry-run`, Windows `-DryRun` (리셋/쓰기 없음).
후보 선택: macOS `--port /dev/cu.usbmodem…`, Windows `-Port COM8`.
확인 생략: macOS `--yes`, Windows `-Yes`.

macOS는 USB modem 포트를, Windows는 Espressif VID/PID의 COM 포트를 후보로 찾는다.
이는 Waveshare 모델의 증명이 아니다. 다른 보드가 섞여 있으면 반드시 기기를 직접 확인한다.
여러 후보에서 임의 선택하지 않는다. macOS는 실행 직전 포트 노드 변경을 검사하지만
USB 일련번호 기반 추적은 아니므로 확인 중 케이블/기기를 바꾸지 않는다.

체크섬은 손상 검사용이지 서명이 아니다. 신뢰하는 배포자에게 받은 패키지만 실행한다.
NVS `0x9000–0xDFFF`를 제외한 네 구간만 쓰며 전체 삭제를 하지 않는다.
기존 프로젝트와 같은 파티션 배치에서 CAL을 유지한다. 다른 배치의 데이터 이행은 보장하지 않는다.

자동 진입 실패 시 BOOT를 누른 채 RESET을 눌렀다 떼고 BOOT를 놓은 후 재탐색한다.
수동 다운로드 모드에서는 업로드 후 RESET을 직접 눌러야 할 수 있다.
macOS Gatekeeper/Windows SmartScreen 또는 PowerShell 실행 정책이 차단할 수 있다.
Windows `update.cmd`는 실행한 PowerShell 프로세스에만 `ExecutionPolicy Bypass`를 적용하며,
사용자·시스템의 실행 정책 설정을 변경하지 않는다. 조직의 관리 정책은 여전히 적용될 수 있다.
보안을 전역 해제하지 말고 출처를 확인한 뒤 OS의 개별 실행 허용 절차나 관리자 지침을 따른다.
Windows ARM은 지원 대상에 포함하지 않는다. OS별 실기 업로드 검증 범위는 릴리스 노트를 참고한다.

`UPSTREAM.json`에 공식 다운로드 URL과 원본 체크섬을 기록한다.
`BUILD_INFO.json`에는 프로젝트 소스 커밋, 보드 설정과 펌웨어 체크섬을 기록한다.
`source/axicube-버전-source.zip`은 해당 커밋의 프로젝트 소스이며,
빌드 의존성과 설치 절차는 그 안의 `docs/BUILD.md`에 있다.
`source/esptool-5.1.0.tar.gz`는 esptool의 공식 소스 배포본(GPLv2+)이다.
프로젝트 GPL-3.0-only 전문은 `LICENSE`, 외부 구성요소 고지는 `THIRD_PARTY_NOTICES.md`,
폰트 라이선스는 `licenses/`에 포함되어 있다. 외부 자료에는 각 원저작자의 조건이 적용된다.
