""" Launch file for testing the composable nodes publisher and subscriber. """

import os
import unittest
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_testing.actions import ReadyToTest
import launch_testing
import launch_testing.asserts
import pytest


def generate_launch_description():
    """
    Generate the launch description for the ROS2 launch testing framework.

    This function sets up the environment to execute a GTest binary as part of the launch test.
    It constructs a LaunchDescription object that includes the test process to be executed.

    Returns:
        A tuple containing the LaunchDescription and a context dictionary with references to
        the processes involved in the test.
    """
    gtest_executable_path = os.path.join(
        os.getenv("TEST_EXECUTABLES_DIR", ""), "test_publisher_subscriber"
    )

    test_process = ExecuteProcess(
        cmd=[gtest_executable_path], name="test_publisher_subscriber", output="screen"
    )

    return LaunchDescription([test_process, ReadyToTest()]), {
        "test_process": test_process
    }


@pytest.mark.launch_test
def generate_test_description():
    """
    Wrapper function to generate test description for pytest.

    This function is marked with pytest's launch_test decorator, indicating it generates
    a launch description for a ROS2 launch test.

    Returns:
        The result of calling generate_launch_description(), which is a launch description
        and context dictionary for the test.
    """
    return generate_launch_description()


class TestGtestOutput(unittest.TestCase):
    """
    Test class for validating the output of the GTest binary execution.
    """

    def test_gtest_run_complete(self, proc_output, test_process):
        """
        Test case to verify that the GTest binary has completed execution.

        Args:
            proc_output: The process output capture helper provided by launch_testing.
            test_process: The test process being monitored.

        This test waits for a specific string to appear in the stdout of the test process,
        indicating that all tests have passed.
        """
        proc_output.assertWaitFor("All tests passed", timeout=10, stream="stdout")
