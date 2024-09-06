$(function () {
  // show/hide할 기준 요소를 선택
  const $protocolSelect = $("#protocol");
  const $espnowSelect = $("#espnow");

  const $groups = {
    protocol: $("#group-protocol"), // 0
    espnow: $("#group-espnow"), // 1
    address: $("#group-address"), // 2
    sensor: $("#group-sensor"), // 3
    period: $("#group-period"), // 4
    mac: $("#group-mac"), // 5
    addition: $("#group-addition"), // 6
  };

  // required 속성 부여할 대상 input 태그를 선택 (ID 또는 클래스 등으로 선택)
  var $requiredAddressInputField = $("#address"); // address
  var $requiredPeriodInputField = $("#period"); // period
  var $requiredMacInputField = $("#mac"); // mac address

  isToggled = false;

  // 토글 스위치 상태 변경 함수
  function toggleSwitch(toggleParameter) {
    if (toggleParameter === true) {
      $("#addition").prop("checked", true); // checked 상태로 변경
      console.log("toggleSwitch called: Checked"); // Debug
    } else {
      $("#addition").prop("checked", false); // unchecked 상태로 변경
      console.log("toggleSwitch called: UnChecked"); // Debug
    }
  }

  function updateVisibility() {
    const protocol = $protocolSelect.val();
    const espnow = $espnowSelect.val();

    // 초기화할 필드들을 빈 배열로 초기화
    let fieldsToClear = [];

    // 모든 그룹을 숨기기 전에 초기화할 필드를 결정
    if (protocol === "1") {
      // Wired 선택 시
      fieldsToClear = [$groups.period, $groups.mac];
    } else if (protocol === "2") {
      // Wireless 선택 시
      if (espnow === "1") {
        // RS485(Standardization) 선택 시
        fieldsToClear = [$groups.period];
      } else if (espnow === "2") {
        // Sensing(Measurement) 선택 시
        fieldsToClear = [$groups.address];
      }
    }

    // 선택된 필드들만 초기화
    fieldsToClear.forEach(($group) => {
      $group.find("input, select").val(""); // 해당 그룹의 input과 select를 초기화
    });

    // 토글 스위치 비활성화로 초기화
    isToggled = false;
    toggleSwitch(isToggled);

    // 모든 그룹을 숨김
    $.each($groups, function (key, $group) {
      $group.hide();
    });

    // 항상 프로토콜 그룹은 보임
    $groups.protocol.show(); // 0

    if (protocol === "1") {
      // Wired가 선택된 경우
      $requiredAddressInputField.attr("required", "required"); // Modbus Address input required 속성 부여

      $requiredPeriodInputField.removeAttr("required"); // 센싱 주기 input required 속성 제거
      $requiredMacInputField.removeAttr("required"); // MAC input required 속성 제거

      $groups.address.show(); // 1 표시
      $groups.sensor.show(); // 2 표시
    } else if (protocol === "2") {
      // Wireless가 선택된 경우
      $groups.espnow.show(); // 4 표시

      if (espnow === "1") {
        // RS485(Standardization)가 선택된 경우
        $requiredAddressInputField.attr("required", "required"); // Modbus Address input required 속성 부여
        $requiredMacInputField.attr("required", "required"); // MAC input required 속성 부여

        $requiredPeriodInputField.removeAttr("required"); // 센싱 주기 input required 속성 제거

        $groups.address.show(); // 1
        $groups.sensor.show(); // 2
        // $groups.espnow.show(); // 4 이미 표시
        $groups.mac.show(); // 5
      } else if (espnow === "2") {
        // Sensing(Measurement)가 선택된 경우
        $requiredPeriodInputField.attr("required", "required"); // 센싱 주기 input required 속성 부여
        $requiredMacInputField.attr("required", "required"); // MAC input required 속성 부여

        $requiredAddressInputField.removeAttr("required"); // Modbus Address input required 속성 제거

        $groups.sensor.show(); // 2
        $groups.period.show(); // 3
        // $groups.espnow.show(); // 4 이미 표시
        $groups.mac.show(); // 5
        $groups.addition.show(); // 6

        // 무선 센서보드 활성화 시 period input 태그 required 속성 부여하기
      }
    }

    // 항상 제출 버튼은 보임
    $(".form-actions").show(); // 7

    console.log("updateVisibility called"); // Debug
  }

  // 이벤트 리스너 등록
  $protocolSelect.on("change", updateVisibility);
  $espnowSelect.on("change", updateVisibility);

  // 페이지 로드 시 초기 상태 설정
  updateVisibility();
});
