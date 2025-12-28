#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

"""
このプログラムは、LiDARのスキャンデータを間引いてAPIにPOSTするノードです。
データサイズを削減し、通信コストを抑えます。
"""

import rospy
from sensor_msgs.msg import LaserScan
import requests
import json
import time
import math
import threading

class LidarApiPublisherLite:
    def __init__(self):
        rospy.init_node('lidar_api_publisher_lite', anonymous=True)
        
        # パラメータの取得
        self.api_endpoint = rospy.get_param('~api_endpoint', 'https://your-api-endpoint.com/lidar')
        self.api_key = rospy.get_param('~api_key', '')
        self.publish_rate = rospy.get_param('~publish_rate', 1.0)  # Hz
        self.api_timeout = rospy.get_param('~api_timeout', 5.0)  # seconds
        self.robot_id = rospy.get_param('~robot_id', 'lightrover_01')
        
        # データ削減パラメータ
        self.angle_resolution = rospy.get_param('~angle_resolution', 10)  # 度単位
        self.send_intensities = rospy.get_param('~send_intensities', False)
        self.max_range_only = rospy.get_param('~max_range_only', False)  # 最小/最大距離のみ送信
        
        # 最新のLiDARデータを保持
        self.latest_scan = None
        self.last_publish_time = 0
        
        # スレッド管理
        self.upload_thread = None
        
        # LaserScanのサブスクライバー
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        rospy.loginfo('LiDAR API Publisher Lite initialized')
        rospy.loginfo('API Endpoint: %s', self.api_endpoint)
        rospy.loginfo('Publish Rate: %.2f Hz', self.publish_rate)
        rospy.loginfo('Angle Resolution: %d degrees', self.angle_resolution)
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
            # LiDARデータをJSON形式に変換（軽量化版）
            payload = self.create_lite_payload(scan_data)
            
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
                
        except requests.exceptions.Timeout:
            rospy.logerr('API request timed out')
        except requests.exceptions.RequestException as e:
            rospy.logerr('Failed to post data to API: %s', str(e))
        except Exception as e:
            rospy.logerr('Unexpected error: %s', str(e))
    
    def create_lite_payload(self, scan_data):
        """軽量化されたAPIペイロードを作成"""
        
        if self.max_range_only:
            # 最小/最大距離のみを送信
            valid_ranges = [r for r in scan_data.ranges if scan_data.range_min <= r <= scan_data.range_max]
            
            payload = {
                'robot_id': self.robot_id,
                'timestamp': scan_data.header.stamp.to_sec(),
                'frame_id': scan_data.header.frame_id,
                'scan_summary': {
                    'min_distance': min(valid_ranges) if valid_ranges else 0,
                    'max_distance': max(valid_ranges) if valid_ranges else 0,
                    'avg_distance': sum(valid_ranges) / len(valid_ranges) if valid_ranges else 0,
                    'num_points': len(valid_ranges)
                }
            }
        else:
            # 角度分解能に基づいてデータを間引く
            thinned_ranges, thinned_angles = self.thin_out_data(scan_data)
            
            payload = {
                'robot_id': self.robot_id,
                'timestamp': scan_data.header.stamp.to_sec(),
                'frame_id': scan_data.header.frame_id,
                'scan_data': {
                    'angle_min': scan_data.angle_min,
                    'angle_max': scan_data.angle_max,
                    'range_min': scan_data.range_min,
                    'range_max': scan_data.range_max,
                    'angles': thinned_angles,  # 間引いた角度リスト
                    'ranges': thinned_ranges,   # 間引いた距離リスト
                    'resolution': self.angle_resolution
                }
            }
            
            # Intensityを含める場合
            if self.send_intensities and scan_data.intensities:
                thinned_intensities = self.thin_out_intensities(scan_data)
                payload['scan_data']['intensities'] = thinned_intensities
        
        return payload
    
    def thin_out_data(self, scan_data):
        """角度分解能に基づいてデータを間引く"""
        angle_increment_deg = math.degrees(scan_data.angle_increment)
        step = max(1, int(self.angle_resolution / angle_increment_deg))
        
        thinned_ranges = []
        thinned_angles = []
        
        for i in range(0, len(scan_data.ranges), step):
            angle = scan_data.angle_min + i * scan_data.angle_increment
            r = scan_data.ranges[i]
            
            # サニタイズ
            r = self._sanitize_float(r, scan_data.range_max)
            
            thinned_angles.append(round(math.degrees(angle), 2))
            thinned_ranges.append(r)
        
        return thinned_ranges, thinned_angles
    
    def thin_out_intensities(self, scan_data):
        """Intensityデータを間引く"""
        angle_increment_deg = math.degrees(scan_data.angle_increment)
        step = max(1, int(self.angle_resolution / angle_increment_deg))
        
        thinned_intensities = []
        for i in range(0, len(scan_data.intensities), step):
            val = scan_data.intensities[i]
            # サニタイズ (optional for int but good for float intensities)
            val = self._sanitize_float(val, 0)
            thinned_intensities.append(val)
        
        return thinned_intensities

    def _sanitize_float(self, value, default_value):
        """inf/nan を安全な値またはNoneに変換"""
        if isinstance(value, float):
            if float(value) == float('inf'):
                return default_value
            if float(value) == float('-inf'):
                return 0
            if value != value: # NaN
                return 0
        return value
    
    def run(self):
        """ノードの実行"""
        rospy.spin()

if __name__ == '__main__':
    try:
        publisher = LidarApiPublisherLite()
        publisher.run()
    except rospy.ROSInterruptException:
        pass
