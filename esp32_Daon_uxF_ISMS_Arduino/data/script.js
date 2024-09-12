$(function () {
  // show/hide할 기준 요소를 선택
  const $protocolSelect = $("#protocol");
  const $espnowSelect = $("#espnow");

  const $groups = {
    protocol: $("#group-protocol"), // 0
    espnow: $("#group-espnow"), // 1
    address: $("#group-address"), // 2
    sensor1: $("#sensor1-group"), // 3
    sensor2: $("#sensor2-group"), // 4
    period: $("#group-period"), // 5
    mac: $("#group-mac"), // 6
    addition: $("#group-addition"), // 7
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

  // 옵션 별 표시내용 결정하는 함수
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

    // $("#sensor1").val(""); // sensor1의 값을 초기화
    $("#sensor2").val(""); // sensor2의 값을 초기화

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

      $groups.address.show(); // 2 표시

      // 센서1 만 표시되도록 크기조절
      $(".sensor1-group").css("width", "100%");
      $(".sensor2-group").css("display", "none"); // sensor2-group을 완전히 숨김
      $groups.sensor1.show(); // 3 표시
    } else if (protocol === "2") {
      // Wireless가 선택된 경우
      $groups.espnow.show(); // 1 표시

      if (espnow === "1") {
        // RS485(Standardization)가 선택된 경우
        $requiredAddressInputField.attr("required", "required"); // Modbus Address input required 속성 부여
        $requiredMacInputField.attr("required", "required"); // MAC input required 속성 부여

        $requiredPeriodInputField.removeAttr("required"); // 센싱 주기 input required 속성 제거

        $groups.address.show(); // 2

        //센서 그룹 크기 복구
        $(".sensor1-group").css("width", "50%");
        $(".sensor2-group").css("display", "block"); // sensor2-group을 다시 표시
        $(".sensor2-group").css("width", "50%");
        //센서 최대 2개 표시
        $groups.sensor1.show(); // 3
        $groups.sensor2.show(); // 4
        // $groups.espnow.show(); // 1 이미 표시
        $groups.mac.show(); // 6
      } else if (espnow === "2") {
        // Sensing(Measurement)가 선택된 경우
        $requiredPeriodInputField.attr("required", "required"); // 센싱 주기 input required 속성 부여
        $requiredMacInputField.attr("required", "required"); // MAC input required 속성 부여

        $requiredAddressInputField.removeAttr("required"); // Modbus Address input required 속성 제거

        // $groups.espnow.show(); // 1 이미 표시
        // 센서1 만 표시되도록 크기조절
        $(".sensor1-group").css("width", "100%");
        $(".sensor2-group").css("display", "none"); // sensor2-group을 완전히 숨김
        $groups.sensor1.show(); // 3
        $groups.sensor2.hide(); // 4 숨김

        $groups.period.show(); // 5
        $groups.mac.show(); // 6
        $groups.addition.show(); // 7

        // 무선 센서보드 활성화 시 period input 태그 required 속성 부여하기
      }
    }

    // 항상 제출 버튼은 보임
    $(".form-actions").show(); // 7

    console.log("updateVisibility called"); // Debug
  }

  // 센서 선택 콤보박스의 중복 선택을 막는 함수
  function updateSensorOptions() {
    const sensor1Val = $("#sensor1").val();
    const sensor2Val = $("#sensor2").val();

    // value 1~4에 대해 모두 적용하는 경우 사용
    // // sensor2 옵션을 먼저 초기화 (모두 활성화)
    // $("#sensor2 option").each(function () {
    //   $(this).prop("disabled", false);
    // });

    // // sensor1의 값이 비어 있지 않으면, 해당 값을 sensor2에서 disable
    // if (sensor1Val) {
    //   $("#sensor2 option[value='" + sensor1Val + "']").prop("disabled", true);
    // }

    // // sensor1 옵션을 먼저 초기화 (모두 활성화)
    // $("#sensor1 option").each(function () {
    //   $(this).prop("disabled", false);
    // });

    // // sensor2의 값이 비어 있지 않으면, 해당 값을 sensor1에서 disable
    // if (sensor2Val) {
    //   $("#sensor1 option[value='" + sensor2Val + "']").prop("disabled", true);
    // }

    // sensor2 옵션 초기화 (모두 활성화, 하지만 value=2,3은 계속 비활성화 유지)
    $("#sensor2 option").each(function () {
      const value = $(this).val();
      if (value !== "2" && value !== "3") {
        $(this).prop("disabled", false);
      }
    });

    // sensor1의 값이 1 또는 4인 경우, 해당 값을 sensor2에서 disable
    if (sensor1Val === "1" || sensor1Val === "4") {
      $("#sensor2 option[value='" + sensor1Val + "']").prop("disabled", true);
    }

    // sensor1 옵션 초기화 (모두 활성화, 하지만 value=2,3은 계속 비활성화 유지)
    $("#sensor1 option").each(function () {
      const value = $(this).val();
      if (value !== "2" && value !== "3") {
        $(this).prop("disabled", false);
      }
    });

    // sensor2의 값이 1 또는 4인 경우, 해당 값을 sensor1에서 disable
    if (sensor2Val === "1" || sensor2Val === "4") {
      $("#sensor1 option[value='" + sensor2Val + "']").prop("disabled", true);
    }
  } // updateSensorOptions()

  // sensor1과 sensor2의 값이 변경될 때마다 옵션 업데이트
  $("#sensor1, #sensor2").on("change", updateSensorOptions);

  // 이벤트 리스너 등록
  $protocolSelect.on("change", updateVisibility);
  $espnowSelect.on("change", updateVisibility);

  updateVisibility(); // 페이지 로드 시 초기 상태 설정
  updateSensorOptions(); // 페이지 로드 시 초기 옵션 상태 설정
});
