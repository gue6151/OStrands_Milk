OStrands Milk 1.9.15 - UBE / RaceMenu 모프 추적 수정 (추가 보완)

수정 파일:
- src/Body.cpp

수정 내용:
1) 모프 변화 감지 후에만 젖꼭지 끝 정점 재측정
   기존 2초 주기는 전체 가슴 메시 후보를 다시 훑고 젖꼭지 끝 영역을 재측정했습니다.
   이번 수정은 이미 찾은 젖꼭지 ring 정점들의 현재 위치만 가볍게 비교합니다.
   위치 변화가 감지되면 즉시 후보 메시를 다시 확인하고 젖꼭지 끝 영역을 재측정합니다.
   모프 변화가 없으면 전체 재측정을 생략합니다. 메시 교체/정점 수 변경을 놓치지 않도록
   기존 2초 간격의 구조 검사도 안전망으로 유지합니다.

2) RaceMenu가 갱신하는 렌더 정점 버퍼 우선 읽기
   이전 수정본은 DynamicTriShape의 dynamicData 또는 skin partition의 rawVertexData를 읽었습니다.
   RaceMenu/SKEE BodyMorph가 geometry의 rendererData->rawVertexData를 갱신하는 경로에서는
   skin partition 쪽 버퍼가 이전 기본 모양을 유지할 가능성이 있습니다. Skyrim 렌더 메시 구조와
   모프 버퍼를 다룬 별도 SKSE 프로젝트의 구현 자료를 참고해 이 경로를 추가했습니다.
   다만 UBE + Skyrim AE 1.6.1170 조합에서의 런타임 동작은 아직 직접 검증하지 못했습니다.
   그래서 현재 정점 좌표를 읽을 때 다음 순서를 사용하도록 보완했습니다.
     - DynamicTriShape dynamicData
     - BSGeometry rendererData->rawVertexData
     - skin partition buffData->rawVertexData (fallback)
   이 읽기 경로를 젖꼭지 위치 계산과 ring 재측정 양쪽에 적용했습니다.

주의 / 검증 상태:
- 이 ZIP은 수정된 소스 코드이며, 컴파일된 DLL은 포함하지 않습니다.
- 원본 ZIP에는 extern/CommonLibSSE-NG가 포함되어 있지 않습니다.
- SKSE Menu Framework 헤더와 SimpleIni도 빌드 시 필요합니다.
- 현재 실행 환경에는 GitHub 직접 다운로드 네트워크가 차단되어 있어 의존성을 가져올 수 없습니다.
- 현재 실행 환경에는 Windows SDK / MSVC 라이브러리도 없어 Windows용 DLL 링크를 완료하지 못했습니다.
- 소스 수정은 했지만 컴파일 및 게임 내 UBE/RaceMenu 테스트는 아직 수행하지 못했습니다.
- 모프 감지는 저장된 젖꼭지 ring 정점의 skin-space 좌표를 비교하는 휴리스틱입니다.
  실제 UBE/RaceMenu에서 좌표 버퍼가 어떻게 갱신되는지에 따라 감지 임계값 조정이 필요할 수 있습니다.
  RaceMenu의 공식 모프 변경 콜백을 연결한 방식은 아니며, 정점 위치 변화를 감지하는 방식입니다.
- 따라서 실제 동작이 확인된 완성 패치라고 보장하지 않습니다.

대상 환경:
- Skyrim SE/AE 1.6.1170.0
- UBE body / RaceMenu BodyMorph
