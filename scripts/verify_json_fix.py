import sys
import unittest
import json
import math

# Mocking modules to allow importing the script without ROS installed
try:
    from unittest.mock import MagicMock
except ImportError:
    from mock import MagicMock # For Python 2.7

sys.modules['rospy'] = MagicMock()
sys.modules['sensor_msgs'] = MagicMock()
sys.modules['sensor_msgs.msg'] = MagicMock()
sys.modules['requests'] = MagicMock()

# Now we can import the target files
# Adjust path if necessary or run from the directory
sys.path.append('.')
from lidar_api_publisher import LidarApiPublisher
from lidar_api_publisher_lite import LidarApiPublisherLite

class TestLidarSanitization(unittest.TestCase):
    def test_sanitize_float_normal(self):
        pub = LidarApiPublisher()
        self.assertEqual(pub._sanitize_float(1.23, 10.0), 1.23)
        self.assertEqual(pub._sanitize_float(0.0, 10.0), 0.0)

    def test_sanitize_float_inf(self):
        pub = LidarApiPublisher()
        # inf should be replaced by default_value (max_range)
        self.assertEqual(pub._sanitize_float(float('inf'), 999.0), 999.0)
        
    def test_sanitize_float_neg_inf(self):
        pub = LidarApiPublisher()
        # -inf should be replaced by 0
        self.assertEqual(pub._sanitize_float(float('-inf'), 999.0), 0)

    def test_sanitize_float_nan(self):
        pub = LidarApiPublisher()
        # nan should be replaced by 0 or handled safe
        val = pub._sanitize_float(float('nan'), 999.0)
        self.assertEqual(val, 0)

    def test_lite_sanitization(self):
        pub = LidarApiPublisherLite()
        self.assertEqual(pub._sanitize_float(float('inf'), 50.0), 50.0)
        self.assertEqual(pub._sanitize_float(float('nan'), 50.0), 0)

    def test_json_serializability(self):
        """Ensure the output of create_payload is JSON serializable even with bad inputs."""
        pub = LidarApiPublisher()
        pub.robot_id = "test_bot"  # Fix: Ensure robot_id is a string, not a MagicMock
        
        # Mock scan data with inf/nan
        mock_scan = MagicMock()
        mock_scan.ranges = [1.0, float('inf'), float('nan'), 2.0]
        mock_scan.intensities = [100, float('inf'), 0]
        mock_scan.range_max = 10.0
        mock_scan.header.stamp.to_sec.return_value = 1234567890.0
        mock_scan.header.frame_id = "laser"
        mock_scan.angle_min = -1.57
        mock_scan.angle_max = 1.57
        mock_scan.angle_increment = 0.01
        mock_scan.time_increment = 0.001
        mock_scan.scan_time = 0.1
        mock_scan.range_min = 0.1
        
        payload = pub.create_payload(mock_scan)
        
        # This acts as the ultimate test: will it crash?
        try:
            json_str = json.dumps(payload)
            # print("JSON Output:", json_str)
        except (ValueError, TypeError) as e:
            # Debug: print types in payload
            print("FAILED. Payload keys and types:")
            for k, v in payload.items():
                print("{}: {}".format(k, type(v)))
                if isinstance(v, dict):
                    for sub_k, sub_v in v.items():
                        print("  {}: {}".format(sub_k, type(sub_v)))
            self.fail("JSON serialization failed: {}".format(e))
            
        # Verify values in payload
        self.assertEqual(payload['scan_data']['ranges'][1], 10.0) # inf -> max
        self.assertEqual(payload['scan_data']['ranges'][2], 0)    # nan -> 0

if __name__ == '__main__':
    unittest.main()
