"""Tests for Gazebo entity lookup and detachable-joint request generation."""

from zhishu_robot_admittance.grasp_coordinator import (
    build_attachment_request,
    find_model_entity_id,
)


def test_find_model_entity_id_uses_named_model():
    output = """
pose {
  name: "ground_plane"
  id: 3
}
pose {
  name: "zhishu_robot"
  id: 26
  position { x: 0.1 }
}
"""
    assert find_model_entity_id(output, "zhishu_robot") == 26
    assert find_model_entity_id(output, "missing") is None


def test_attachment_request_targets_robot_and_crate():
    request = build_attachment_request(26)
    assert "id: 26, type: MODEL" in request
    assert "<parent_link>L_hand_link</parent_link>" in request
    assert "<child_model>carry_box</child_model>" in request
    assert "<child_link>box_link</child_link>" in request
    assert "/admittance_box/detach" in request
