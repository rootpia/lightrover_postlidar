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
import threading

class LidarApiPublisher:
    def __init__(self):
        rospy.init_node('lidar_api_publisher', anonymous=True)
        
        # パラメータの取得
        self.api_endpoint = rospy.get_param('~api_endpoint', 'https://your-api-endpoint.com/lidar')
        self.api_key = rospy.get_param('~api_key', '')
        self.publish_rate = rospy.get_param('~publish_rate', 1.0)  # Hz
        self.api_timeout = rospy.get_param('~api_timeout', 5.0)  # seconds
        self.robot_id = rospy.get_param('~robot_id', 'lightrover_01')
        
        # 最新のLiDARデータを保持
        self.latest_scan = None
        self.last_publish_time = 0
        
        # スレッド管理
        self.upload_thread = None
        
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
        
        # 前回の送信がまだ終わっていなければスキップ
        if self.upload_thread and self.upload_thread.is_alive():
            rospy.logdebug("Previous upload still in progress, skipping")
            return

        self.last_publish_time = current_time
        
        # Threadで送信処理を実行
        self.upload_thread = threading.Thread(target=self.publish_to_api, args=(data,))
        self.upload_thread.daemon = True
        self.upload_thread.start()
    
    def publish_to_api(self, scan_data):
        """LiDARデータをAPIにPOST (別スレッドで実行)"""
        try:
            # LiDARデータをJSON形式に変換
            payload = self.create_payload(scan_data)
            
            # APIにPOSTリクエスト
            headers = {
                'Content-Type': 'application/json'
            }
            if self.api_key:
                headers['x-api-key'] = self.api_key

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
        # inf/nan対策: リスト内の数値をサニタイズ
        ranges = [self._sanitize_float(r, scan_data.range_max) for r in scan_data.ranges]
        intensities = [self._sanitize_float(i, 0) for i in (scan_data.intensities if scan_data.intensities else [])]

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
                'ranges': ranges,
                'intensities': intensities
            }
        }
        return payload
    
    def _sanitize_float(self, value, default_value):
        """inf/nan を安全な値またはNoneに変換"""
        if float(value) == float('inf'):
            return default_value # または None
        if float(value) == float('-inf'):
            return 0 # 距離なので0
        if value != value: # NaN check
            return 0 # または None
        return value
    
    def run(self):
        """ノードの実行"""
        rospy.spin()

if __name__ == '__main__':
    try:
        publisher = LidarApiPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass
