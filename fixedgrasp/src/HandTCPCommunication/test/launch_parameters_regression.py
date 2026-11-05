#!/usr/bin/env python3
"""Inspect launch parameters without starting hardware nodes."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

import rospkg
import yaml


class LaunchParametersTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.package = Path(rospkg.RosPack().get_path("HandTCPCommunication"))
        cls.launch = cls.package / "launch" / "HandCtrl_node.launch"

    def params(self, *arguments, namespace="/"):
        env = dict(os.environ, ROS_NAMESPACE=namespace)
        output = subprocess.check_output(
            ["roslaunch", "--dump-params", str(self.launch), *arguments],
            env=env, text=True, timeout=15,
        )
        return yaml.safe_load(output)

    def test_defaults_resolve_package_files(self):
        params = self.params()
        self.assertEqual("192.168.1.18", params["/arm/arm_ip"])
        self.assertEqual(8080, params["/arm/arm_port"])
        self.assertEqual(5000, params["/arm/arm_timeout_ms"])
        self.assertEqual(20, params["/arm/arm_speed_percent"])
        self.assertEqual(str(self.package / "target_pose.json"), params["/arm/target_pose_file"])
        self.assertEqual(str(self.package / "tools_Tcp.json"), params["/arm/tools_tcp_file"])
        self.assertEqual(str(self.package / "CalibrateMatrix_EyeInHand.txt"), params["/arm/hand_eye_calibration_file"])

    def test_arguments_override_defaults(self):
        params = self.params("arm_ip:=127.0.0.2", "arm_port:=18080", "arm_timeout_ms:=1234", "arm_speed_percent:=35")
        self.assertEqual("127.0.0.2", params["/arm/arm_ip"])
        self.assertEqual(18080, params["/arm/arm_port"])
        self.assertEqual(1234, params["/arm/arm_timeout_ms"])
        self.assertEqual(35, params["/arm/arm_speed_percent"])

    def test_calibration_path_argument_and_yaml_override(self):
        params = self.params("hand_eye_calibration_file:=/tmp/custom-calibration.txt")
        self.assertEqual("/tmp/custom-calibration.txt", params["/arm/hand_eye_calibration_file"])
        with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml") as config:
            yaml.safe_dump({"hand_eye_calibration_file": "/tmp/deployment-calibration.txt"}, config)
            config.flush()
            params = self.params("motion_config_file:=" + config.name)
        self.assertEqual("/tmp/deployment-calibration.txt", params["/arm/hand_eye_calibration_file"])

    def test_deployment_yaml_takes_precedence(self):
        with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml") as config:
            yaml.safe_dump({"arm_ip": "127.0.0.3", "arm_speed_percent": 17,
                            "workspace_bounds_m": [[-1, 1]] * 3}, config)
            config.flush()
            params = self.params("arm_speed_percent:=35", "motion_config_file:=" + config.name)
        self.assertEqual("127.0.0.3", params["/arm/arm_ip"])
        self.assertEqual(17, params["/arm/arm_speed_percent"])
        self.assertEqual([[-1, 1]] * 3, params["/arm/workspace_bounds_m"])

    def test_deployment_yaml_stays_in_node_namespace(self):
        with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml") as config:
            yaml.safe_dump({"arm_port": 18081}, config)
            config.flush()
            params = self.params("motion_config_file:=" + config.name, namespace="/test_robot")
        self.assertEqual(18081, params["/test_robot/arm/arm_port"])
        self.assertNotIn("/arm/arm_port", params)


if __name__ == "__main__":
    unittest.main()
