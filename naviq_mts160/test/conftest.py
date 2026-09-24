"""Test-session isolation: unless the caller chose a ROS domain, use one of our own so a driver running
on the host (publishing on /mts160/... and /diagnostics) cannot feed the tests."""
import os

os.environ.setdefault("ROS_DOMAIN_ID", str(1 + os.getpid() % 100))
