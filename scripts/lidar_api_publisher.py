#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

"""
このプログラムは、LiDARのスキャンデータをAPIにPOSTするためのノードです。
"""

import rospy
from sensor_msgs.msg import LaserScan
import requests
import json
import time

class LidarApiPublisher:
    def __init__(self):
        rospy.init_node('lidar_api_publisher', anonymous=True)
        
        # パラメータの取得
        self.api_endpoint = rospy.get_param('~api_endpoint', 'https://your-api-endpoint.com/lidar')
        self.publish_rate = rospy.get_param('~publish_rate', 1.0)  # Hz
        self.api_timeout = rospy.get_param('~api_timeout', 5.0)  # seconds
        self.robot_id = rospy.get_param('~robot_id', 'lightrover_01')
        
        # 最新のLiDARデータを保持
        self.latest_scan = None
        self.last_publish_time = 0
        
        # LaserScanのサブスクライバー
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        rospy.loginfo('LiDAR API Publisher initialized')
        rospy.loginfo('API Endpoint: %s', self.api_endpoint)
        rospy.loginfo('Publish Rate: %.2f Hz', self.publish_rate)
        rospy.loginfo('Robot ID: %s', self.robot_id)
    
    def scan_callback(self, data):
        """LiDARスキャンデータを受信したときのコールバック"""
        self.latest_scan = data
        
        # レート制限のチェック
        current_time = time.time()
        if current_time - self.last_publish_time < (1.0 / self.publish_rate):
            return
        
        self.last_publish_time = current_time
        
        # APIにデータを送信
        self.publish_to_api(data)
    
    def publish_to_api(self, scan_data):
        """LiDARデータをAPIにPOST"""
        try:
            # LiDARデータをJSON形式に変換
            payload = self.create_payload(scan_data)
            
            # APIにPOSTリクエスト
            headers = {'Content-Type': 'application/json'}
            response = requests.post(
                self.api_endpoint,
                data=json.dumps(payload),
                headers=headers,
                timeout=self.api_timeout
            )
            
            if response.status_code == 200:
                rospy.logdebug('Successfully posted LiDAR data to API')
            else:
                rospy.logwarn('API returned status code: %d', response.status_code)
                rospy.logwarn('Response: %s', response.text)
                
        except requests.exceptions.Timeout:
            rospy.logerr('API request timed out')
        except requests.exceptions.RequestException as e:
            rospy.logerr('Failed to post data to API: %s', str(e))
        except Exception as e:
            rospy.logerr('Unexpected error: %s', str(e))
    
    def create_payload(self, scan_data):
        """LiDARデータからAPIペイロードを作成"""
        payload = {
            'robot_id': self.robot_id,
            'timestamp': scan_data.header.stamp.to_sec(),
            'frame_id': scan_data.header.frame_id,
            'scan_data': {
                'angle_min': scan_data.angle_min,
                'angle_max': scan_data.angle_max,
                'angle_increment': scan_data.angle_increment,
                'time_increment': scan_data.time_increment,
                'scan_time': scan_data.scan_time,
                'range_min': scan_data.range_min,
                'range_max': scan_data.range_max,
                'ranges': list(scan_data.ranges),
                'intensities': list(scan_data.intensities) if scan_data.intensities else []
            }
        }
        return payload
    
    def run(self):
        """ノードの実行"""
        rospy.spin()

if __name__ == '__main__':
    try:
        publisher = LidarApiPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass
