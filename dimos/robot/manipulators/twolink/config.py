# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Two-link planar arm hardware configuration helpers."""

from __future__ import annotations

import math
from pathlib import Path

from dimos.control.components import HardwareComponent, HardwareType
from dimos.core.global_config import global_config
from dimos.hardware.spec import JointLimits
from dimos.robot.manipulators._modeling import joint_names

TWOLINK_DOF = 2
# Matches the joint ranges in assets/twolink.xml.
TWOLINK_JOINT_LIMIT = 2.6  # rad

# TODO: move the scene to LFS and load it with LfsPath, like PIPER_SIM_PATH.
TWOLINK_SIM_PATH = Path(__file__).parent / "assets" / "twolink.xml"


def make_twolink_hardware(
    hw_id: str = "arm",
    *,
    adapter_type: str = "mock",
    auto_enable: bool = True,
) -> HardwareComponent:
    mock_limits = JointLimits(
        position_lower=[-TWOLINK_JOINT_LIMIT] * TWOLINK_DOF,
        position_upper=[TWOLINK_JOINT_LIMIT] * TWOLINK_DOF,
        velocity_max=[math.pi] * TWOLINK_DOF,
    )
    return HardwareComponent(
        hardware_id=hw_id,
        hardware_type=HardwareType.MANIPULATOR,
        joints=joint_names(TWOLINK_DOF),
        adapter_type=adapter_type,
        auto_enable=auto_enable,
        limits=mock_limits if adapter_type == "mock" else None,
    )


def twolink_hardware(hw_id: str = "arm") -> HardwareComponent:
    """The two-link arm; in simulation it attaches to the blueprint's MujocoSimModule.

    There is no real two-link arm yet, so outside simulation it runs on the mock adapter.
    """
    if global_config.simulation:
        return make_twolink_hardware(hw_id, adapter_type="sim_mujoco")
    return make_twolink_hardware(hw_id, adapter_type="mock")
