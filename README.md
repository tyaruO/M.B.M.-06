# M.B.M.-06
robot

from enum import Enum
from dataclasses import dataclass
from typing import Optional


class RobotMode(Enum):
    SPHERE = "sphere"
    TRANSFORMING = "transforming"
    DEPLOYED = "deployed"
    ANCHORED = "anchored"
    WORKING = "working"
    EMERGENCY_STOP = "emergency_stop"


class LimbRole(Enum):
    STOWED = "stowed"
    SUPPORT = "support"
    MANIPULATOR = "manipulator"


@dataclass
class LimbState:
    limb_id: int
    role: LimbRole = LimbRole.STOWED
    joint_position: float = 0.0
    load_n: float = 0.0
    locked: bool = False
    contact_detected: bool = False


@dataclass
class RobotState:
    mode: RobotMode = RobotMode.SPHERE
    limbs: list = None
    emergency_stop: bool = False
    power_available: bool = True
    sensor_data_valid: bool = False

    def __post_init__(self):
        if self.limbs is None:
            self.limbs = [
                LimbState(i) for i in range(6)
            ]


class MBM06Controller:
    """
    M.B.M.-06 制御ソフトウェアの基本構造。

    注意:
    このクラスは実機ドライバーではない。
    モーター出力や油圧弁を直接操作しない。
    """

    def __init__(self):
        self.state = RobotState()
        self.events = []

    def log(self, message: str):
        self.events.append(message)
        print("[M.B.M.-06]", message)

    def safety_check(self) -> bool:
        if self.state.emergency_stop:
            self.log("緊急停止中")
            return False

        if not self.state.power_available:
            self.log("電源異常")
            return False

        if not self.state.sensor_data_valid:
            self.log("センサーデータ未確認")
            return False

        return True

    def emergency_stop(self):
        self.state.emergency_stop = True
        self.state.mode = RobotMode.EMERGENCY_STOP

        for limb in self.state.limbs:
            limb.locked = True

        self.log("緊急停止要求を発行")

    def reset_emergency(self):
        """
        実機では原因除去と安全確認後にのみ
        リセットを許可する必要がある。
        """
        if not self.state.power_available:
            raise RuntimeError("電源異常を解消してください")

        self.state.emergency_stop = False
        self.state.mode = RobotMode.SPHERE
        self.log("緊急停止状態を解除")

    def deploy(self) -> bool:
        if not self.safety_check():
            return False

        if self.state.mode != RobotMode.SPHERE:
            self.log("展開不可：球体モードではありません")
            return False

        self.state.mode = RobotMode.TRANSFORMING

        # 実機では各機構の位置・ロック解除を確認する
        for limb in self.state.limbs:
            limb.role = LimbRole.SUPPORT
            limb.locked = False

        self.state.mode = RobotMode.DEPLOYED
        self.log("6本の肢の展開シーケンスを要求")
        return True

    def anchor(self, support_ids: list[int]) -> bool:
        if not self.safety_check():
            return False

        if self.state.mode != RobotMode.DEPLOYED:
            self.log("固定不可：展開状態ではありません")
            return False

        if len(support_ids) != 4:
            self.log("固定不可：支持脚は4本指定してください")
            return False

        if len(set(support_ids)) != 4:
            self.log("固定不可：脚IDが重複しています")
            return False

        if any(i < 0 or i >= 6 for i in support_ids):
            self.log("固定不可：脚IDが範囲外です")
            return False

        for limb in self.state.limbs:
            limb.role = LimbRole.MANIPULATOR

        for limb_id in support_ids:
            limb = self.state.limbs[limb_id]
            limb.role = LimbRole.SUPPORT

        # 実機では接地・荷重・ロックの各センサーを
        # 確認してから固定完了と判定する必要がある。
        self.log(
            f"4本の支持脚を指定：{support_ids}"
        )

        if not all(
            self.state.limbs[i].contact_detected
            and self.state.limbs[i].locked
            for i in support_ids
        ):
            self.log("固定未確認：作業アームを有効化しない")
            return False

        self.state.mode = RobotMode.ANCHORED
        self.log("支持状態の確認完了")
        return True

    def begin_lifting(self, arm_ids: list[int],
                      target_load_n: float) -> bool:
        if not self.safety_check():
            return False

        if self.state.mode != RobotMode.ANCHORED:
            self.log("持ち上げ不可：支持状態が未確認")
            return False

        if len(arm_ids) != 2 or len(set(arm_ids)) != 2:
            self.log("持ち上げ不可：作業アームは2本必要")
            return False

        if any(i < 0 or i >= 6 for i in arm_ids):
            self.log("持ち上げ不可：アームIDが範囲外")
            return False

        if target_load_n <= 0:
            self.log("持ち上げ不可：荷重指定が不正")
            return False

        for arm_id in arm_ids:
            if self.state.limbs[arm_id].role != LimbRole.MANIPULATOR:
                self.log("持ち上げ不可：アームの役割が不正")
                return False

        # ここでは実際の駆動指令を送信しない。
        # 実機ではトルク限界、転倒モーメント、
        # 荷重制限、工具の把持状態を検証する必要がある。
        self.state.mode = RobotMode.WORKING

        self.log(
            f"持ち上げ要求：アーム={arm_ids}, "
            f"目標荷重={target_load_n:.1f} N"
        )
        return True


if __name__ == "__main__":
    robot = MBM06Controller()

    # 実機接続前のソフトウェア構造確認用。
    # センサー値を模擬して安全確認を通過させる例。
    robot.state.sensor_data_valid = True

    print("M.B.M.-06 control core initialized")
    print("Mode:", robot.state.mode.value)

    # 実機の駆動指令は実装していない。


from enum import Enum
from dataclasses import dataclass, field


class Mode(Enum):
    SPHERE = "sphere"
    TRANSFORMING = "transforming"
    DEPLOYED = "deployed"
    ANCHORED = "anchored"
    WORKING = "working"
    FAULT = "fault"
    E_STOP = "emergency_stop"


class LimbRole(Enum):
    STOWED = "stowed"
    SUPPORT = "support"
    MANIPULATOR = "manipulator"


@dataclass
class Limb:
    joint_positions: list[float] = field(
        default_factory=lambda: [0.0, 0.0, 0.0]
    )
    role: LimbRole = LimbRole.STOWED
    contact: bool = False
    locked: bool = False
    load_n: float = 0.0


@dataclass
class Sensors:
    valid: bool = False
    tilt_deg: float = 0.0
    battery_percent: float = 100.0
    communication_ok: bool = False


class MBM06Controller:
    LIMB_COUNT = 6

    def __init__(self):
        self.mode = Mode.SPHERE
        self.limbs = [Limb() for _ in range(self.LIMB_COUNT)]
        self.sensors = Sensors()
        self.fault_reason = None

    def enter_fault(self, reason: str):
        self.mode = Mode.FAULT
        self.fault_reason = reason
        print(f"[FAULT] {reason}")

    def emergency_stop(self, reason: str):
        # 実機では独立した安全回路にも停止要求を送る。
        self.mode = Mode.E_STOP
        self.fault_reason = reason
        print(f"[E-STOP] {reason}")

    def safety_check(self) -> bool:
        if self.mode == Mode.E_STOP:
            return False

        if not self.sensors.valid:
            self.enter_fault("Sensor data invalid")
            return False

        if not self.sensors.communication_ok:
            self.enter_fault("Communication lost")
            return False

        if self.sensors.battery_percent <= 5.0:
            self.enter_fault("Battery critically low")
            return False

        if abs(self.sensors.tilt_deg) > 15.0:
            self.enter_fault("Excessive body tilt")
            return False

        return self.mode not in (Mode.FAULT, Mode.E_STOP)

    def begin_deployment(self) -> bool:
        if self.mode != Mode.SPHERE:
            return False

        if not self.safety_check():
            return False

        self.mode = Mode.TRANSFORMING
        print("[CMD] Deployment requested")
        return True

    def confirm_deployed(self, all_limbs_verified: bool):
        if self.mode != Mode.TRANSFORMING:
            return False

        if not all_limbs_verified:
            self.enter_fault("Deployment not verified")
            return False

        self.mode = Mode.DEPLOYED
        print("[STATE] Deployed")
        return True

    def configure_roles(
        self,
        support_ids: list[int],
        manipulator_ids: list[int],
    ) -> bool:
        if self.mode != Mode.DEPLOYED:
            return False

        if len(support_ids) != 4 or len(manipulator_ids) != 2:
            return False

        if len(set(support_ids + manipulator_ids)) != 6:
            return False

        if set(support_ids + manipulator_ids) != set(range(6)):
            return False

        for i in support_ids:
            self.limbs[i].role = LimbRole.SUPPORT

        for i in manipulator_ids:
            self.limbs[i].role = LimbRole.MANIPULATOR

        return True

    def confirm_anchored(self) -> bool:
        if self.mode != Mode.DEPLOYED:
            return False

        supports = [
            limb for limb in self.limbs
            if limb.role == LimbRole.SUPPORT
        ]

        if len(supports) != 4:
            return False

        if not all(limb.contact and limb.locked for limb in supports):
            return False

        self.mode = Mode.ANCHORED
        print("[STATE] Support verified")
        return True

    def request_lifting(self, target_load_n: float) -> bool:
        if self.mode != Mode.ANCHORED:
            return False

        if not self.safety_check():
            return False

        if target_load_n <= 0:
            return False

        manipulators = [
            limb for limb in self.limbs
            if limb.role == LimbRole.MANIPULATOR
        ]

        if len(manipulators) != 2:
            return False

        # 荷重上限・重心・関節トルクの検証は
        # 実機の仕様に基づいて別途実装する必要がある。
        print(f"[CMD] Lift request: {target_load_n:.1f} N")
        self.mode = Mode.WORKING
        return True


if __name__ == "__main__":
    robot = MBM06Controller()
    print("M.B.M.-06 control core initialized")
    print("Mode:", robot.mode.value)
