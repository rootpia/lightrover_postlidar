"""
AWS Lambda関数のサンプル
API Gateway + Lambda + DynamoDBでLiDARデータを受信・保存
"""

import json
import boto3
from datetime import datetime
from decimal import Decimal

# DynamoDB接続
dynamodb = boto3.resource('dynamodb')
table = dynamodb.Table('LidarData')  # テーブル名は環境に応じて変更

def lambda_handler(event, context):
    """
    LiDARデータを受信してDynamoDBに保存するLambda関数
    """
    try:
        # リクエストボディの取得
        if 'body' in event:
            body = json.loads(event['body'])
        else:
            body = event
        
        # 必須フィールドの確認
        required_fields = ['robot_id', 'timestamp', 'scan_data']
        for field in required_fields:
            if field not in body:
                return {
                    'statusCode': 400,
                    'body': json.dumps({
                        'error': f'Missing required field: {field}'
                    })
                }
        
        # データの整形（DynamoDB用にfloatをDecimalに変換）
        item = convert_floats_to_decimal(body)
        
        # プライマリキーの設定
        item['id'] = f"{body['robot_id']}_{body['timestamp']}"
        item['created_at'] = datetime.utcnow().isoformat()
        
        # DynamoDBに保存
        table.put_item(Item=item)
        
        return {
            'statusCode': 200,
            'headers': {
                'Content-Type': 'application/json',
                'Access-Control-Allow-Origin': '*'
            },
            'body': json.dumps({
                'message': 'LiDAR data saved successfully',
                'id': item['id']
            })
        }
        
    except Exception as e:
        print(f"Error: {str(e)}")
        return {
            'statusCode': 500,
            'body': json.dumps({
                'error': 'Internal server error',
                'message': str(e)
            })
        }

def convert_floats_to_decimal(obj):
    """
    floatをDecimalに変換（DynamoDB用）
    """
    if isinstance(obj, list):
        return [convert_floats_to_decimal(item) for item in obj]
    elif isinstance(obj, dict):
        return {key: convert_floats_to_decimal(value) for key, value in obj.items()}
    elif isinstance(obj, float):
        return Decimal(str(obj))
    else:
        return obj
