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

"""coordinator_twolink outside simulation: built in real workers on the mock adapter."""

from __future__ import annotations

from collections.abc import Iterator

import pytest

from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.trajectory_task.trajectory_task import TrajectoryExecutionStatus
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.robot.manipulators.twolink.blueprints.basic import coordinator_twolink
from dimos.utils.testing.waiting import wait_until

JOINTS = ["joint1", "joint2"]
TARGET = [1.0, -0.5]


@pytest.fixture
def system() -> Iterator[ModuleCoordinator]:
    parsed = BlueprintConfigParser(coordinator_twolink).parse(
        environ={}, overrides={"g": {"viewer": "none"}}
    )
    built = ModuleCoordinator.build(coordinator_twolink, parsed)
    yield built
    built.stop()


def positions(system: ModuleCoordinator) -> list[float]:
    seen = system.get_instance(ControlCoordinator).get_joint_positions()
    return [seen[joint] for joint in JOINTS]


def test_blueprint_uses_the_mock_adapter_outside_simulation() -> None:
    atom = next(
        atom
        for atom in coordinator_twolink.active_blueprints
        if issubclass(atom.module, ControlCoordinator)
    )
    [hardware] = atom.kwargs["hardware"]
    assert hardware.adapter_type == "mock"
    assert hardware.joints == JOINTS


def test_a_trajectory_moves_the_mock_arm_to_the_target(system: ModuleCoordinator) -> None:
    coordinator = system.get_instance(ControlCoordinator)
    wait_until(lambda: set(JOINTS) <= set(coordinator.get_joint_positions()), timeout=10.0)
    trajectory = JointTrajectory(
        joint_names=JOINTS,
        points=[
            TrajectoryPoint(positions=positions(system)),
            TrajectoryPoint(positions=TARGET, time_from_start=0.3),
        ],
    )

    result = coordinator.execute_trajectory(trajectory)

    assert result.status is TrajectoryExecutionStatus.ACCEPTED, result
    wait_until(lambda: positions(system) == pytest.approx(TARGET, abs=1e-6), timeout=10.0)
