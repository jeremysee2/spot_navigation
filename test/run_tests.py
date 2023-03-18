#!/usr/bin/env python3
import typing
import rosunit

test_cases: typing.List[typing.Tuple[str, str, str]] = [
    ("spot_nav", "spot_nav_test", "spot_nav_test.TestSuiteSpotNav"),
    ("nav_ros", "nav_ros_test", "nav_ros_test.TestSuiteNavROS"),
]

for test_case in test_cases:
    rosunit.unitrun(test_case[0], test_case[1], test_case[2])
