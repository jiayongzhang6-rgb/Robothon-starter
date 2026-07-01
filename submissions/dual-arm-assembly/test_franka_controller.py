"""Unit tests for the dual-arm space module assembly controller."""
import os
import sys
import numpy as np

def test_import():
    """Test that the controller module can be imported."""
    sys.path.insert(0, os.path.dirname(__file__))
    import franka_controller
    assert hasattr(franka_controller, 'model')
    assert hasattr(franka_controller, 'data')
    print("✓ Module import test passed")

def test_model_loading():
    """Test that MuJoCo model loads correctly."""
    import franka_controller
    assert franka_controller.model is not None
    assert franka_controller.data is not None
    print("✓ Model loading test passed")

def test_joint_indices():
    """Test that joint indices are correctly defined."""
    import franka_controller
    assert len(franka_controller.L_Q) == 7
    assert len(franka_controller.R_Q) == 7
    assert len(franka_controller.L_D) == 7
    assert len(franka_controller.R_D) == 7
    print("✓ Joint indices test passed")

def test_gripper_control():
    """Test gripper control values exist."""
    import franka_controller
    assert hasattr(franka_controller, 'fL')
    assert hasattr(franka_controller, 'fR')
    print("✓ Gripper control test passed")

def test_ik_solver():
    """Test IK solver function exists."""
    import franka_controller
    assert hasattr(franka_controller, 'solve_ik') or hasattr(franka_controller, 'get_ik')
    print("✓ IK solver test passed")

def test_body_ids():
    """Test that body IDs are correctly defined."""
    import franka_controller
    assert franka_controller.hL >= 0
    assert franka_controller.hR >= 0
    assert franka_controller.mA >= 0
    assert franka_controller.mB >= 0
    assert franka_controller.mC >= 0
    print("✓ Body IDs test passed")

def test_scene_xml():
    """Test that scene XML file exists."""
    import franka_controller
    assert os.path.exists(franka_controller.XML)
    print("✓ Scene XML test passed")

def test_task_steps():
    """Test that 8 task steps are defined."""
    import franka_controller
    assert hasattr(franka_controller, 'STEPS')
    assert len(franka_controller.STEPS) == 8
    print("✓ Task steps test passed")

if __name__ == "__main__":
    test_import()
    test_model_loading()
    test_joint_indices()
    test_gripper_control()
    test_ik_solver()
    test_body_ids()
    test_scene_xml()
    test_task_steps()
    print("\n✅ All 8 tests passed!")
