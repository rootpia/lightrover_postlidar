# LiDAR API Publisher

ライトローバーのLiDARデータをAPIにPOSTするROSパッケージです。

## 概要

このパッケージは、YDLiDARから取得したスキャンデータをリアルタイムでAWS API（またはその他のREST API）に送信します。

## ファイル構成

```
lightrover_ros/
├── scripts/
│   └── lidar_api_publisher.py    # LiDARデータをAPIにPOSTするノード
├── launch/
│   ├── lidar_api_publisher.launch    # API送信ノードのみを起動
│   └── lidar_with_api.launch         # LiDAR + API送信を同時起動
└── aws/
    ├── lambda_handler.py              # AWS Lambda関数サンプル
    └── template.yaml                  # AWS SAMテンプレート
```

## 必要な依存関係

### ROS側（Raspberry Pi）

```bash
# Python 2.7用のrequestsライブラリをインストール
sudo apt-get update
sudo apt-get install python-pip
pip install requests
```

### AWS側

- AWS CLI
- AWS SAM CLI（デプロイ用）
- boto3（Lambdaで自動的に利用可能）

## セットアップ

### 1. ROSパッケージへの追加

```bash
# scriptsディレクトリにファイルを配置
cd ~/catkin_ws/src/lightrover_ros/scripts/
# lidar_api_publisher.pyをコピー

# 実行権限を付与
chmod +x lidar_api_publisher.py

# launchディレクトリにファイルを配置
cd ~/catkin_ws/src/lightrover_ros/launch/
# lidar_api_publisher.launch と lidar_with_api.launch をコピー

# ビルド
cd ~/catkin_ws
catkin_make
```

### 2. AWS APIのデプロイ

#### Option A: AWS SAMを使用する場合

```bash
# SAM CLIのインストール（初回のみ）
pip install aws-sam-cli

# デプロイ
cd aws/
sam build
sam deploy --guided

# 出力されたAPIエンドポイントURLをメモ
# 例: https://xxxxxxxxxx.execute-api.ap-northeast-1.amazonaws.com/prod/api/lidar
```

#### Option B: 手動でセットアップする場合

1. **DynamoDBテーブルの作成**
   - テーブル名: `LidarData`
   - パーティションキー: `id` (String)
   - GSI: `robot_id` (Hash), `timestamp` (Range)

2. **Lambda関数の作成**
   - ランタイム: Python 3.9
   - `lambda_handler.py` のコードをコピー
   - DynamoDBへのアクセス権限を付与

3. **API Gatewayの設定**
   - REST APIを作成
   - POST /api/lidar エンドポイントを作成
   - Lambdaと統合

### 3. launchファイルの設定

`lidar_with_api.launch` を編集し、APIエンドポイントを設定：

```xml
<param name="api_endpoint" type="string" value="https://your-actual-endpoint.com/api/lidar" />
```

その他のパラメータ：
- `publish_rate`: データ送信頻度（Hz）デフォルト: 1.0
- `api_timeout`: APIタイムアウト（秒）デフォルト: 5.0
- `robot_id`: ロボット識別ID デフォルト: "lightrover_01"

## 使用方法

### パターン1: LiDARとAPI送信を同時起動

```bash
roslaunch lightrover_ros lidar_with_api.launch
```

### パターン2: API送信ノードのみを起動（LiDARは別途起動済み）

```bash
# 別ターミナルでLiDARを起動
roslaunch lightrover_ros gmapping.launch

# API送信ノードを起動
roslaunch lightrover_ros lidar_api_publisher.launch
```

### パターン3: パラメータを指定して起動

```bash
roslaunch lightrover_ros lidar_api_publisher.launch \
  api_endpoint:=https://your-api.com/lidar \
  publish_rate:=2.0 \
  robot_id:=robot_abc123
```

## データフォーマット

### 送信されるJSON形式

```json
{
  "robot_id": "lightrover_01",
  "timestamp": 1234567890.123,
  "frame_id": "lidar_link",
  "scan_data": {
    "angle_min": -3.14159,
    "angle_max": 3.14159,
    "angle_increment": 0.0174533,
    "time_increment": 0.0001,
    "scan_time": 0.1,
    "range_min": 0.1,
    "range_max": 12.0,
    "ranges": [1.5, 1.6, 1.7, ...],
    "intensities": [100, 105, 110, ...]
  }
}
```

## トラブルシューティング

### LiDARデータが送信されない

```bash
# LiDARトピックが配信されているか確認
rostopic list | grep scan
rostopic echo /scan -n 1

# ノードが起動しているか確認
rosnode list | grep lidar_api_publisher

# ログを確認
rosnode info /lidar_api_publisher
```

### API接続エラー

```bash
# ログを確認
roslaunch lightrover_ros lidar_api_publisher.launch --screen

# 手動でAPIテスト
curl -X POST https://your-api-endpoint.com/api/lidar \
  -H "Content-Type: application/json" \
  -d '{"robot_id":"test","timestamp":123,"scan_data":{}}'
```

### DynamoDBにデータが保存されない

```bash
# AWS CLIでテーブルを確認
aws dynamodb describe-table --table-name LidarData

# Lambda関数のログを確認
aws logs tail /aws/lambda/LidarDataFunction --follow
```

## パフォーマンスチューニング

### データ送信頻度の調整

LiDARは通常10Hzでスキャンしますが、すべてのデータを送信すると帯域幅を圧迫する可能性があります。

```xml
<!-- 1秒に1回送信（推奨） -->
<param name="publish_rate" type="double" value="1.0" />

<!-- 5秒に1回送信（低頻度） -->
<param name="publish_rate" type="double" value="0.2" />

<!-- 2秒に1回送信 -->
<param name="publish_rate" type="double" value="0.5" />
```

### データサイズの削減

大量のrangesデータを送信する場合、以下の最適化を検討してください：

1. **間引き**: 角度分解能を下げる
2. **圧縮**: データを圧縮してから送信
3. **差分送信**: 変化があった部分のみ送信

## AWS コスト見積もり

1秒に1回のペースでLiDARデータを送信する場合：

- **API Gateway**: 約260万リクエスト/月 → 約$9/月
- **Lambda**: 実行回数と実行時間による → 約$1-3/月
- **DynamoDB**: ストレージと読み書き容量による → 約$5-20/月（データ保持期間に依存）

**TTL設定推奨**: 古いデータを自動削除してコストを削減

## セキュリティ

### 本番環境での推奨事項

1. **API認証の追加**
   - API KeyまたはIAM認証を使用
   - `lidar_api_publisher.py`にヘッダー追加

2. **データ暗号化**
   - HTTPSを使用（デフォルト）
   - DynamoDBの暗号化を有効化

3. **ネットワークセキュリティ**
   - VPC内にLambdaを配置
   - セキュリティグループで送信元を制限

## ライセンス

MIT License

## 参考リンク

- [ライトローバー製品ページ](https://www.vstone.co.jp/products/lightrover/index.html)
- [ROS sensor_msgs/LaserScan](http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html)
- [AWS SAM Documentation](https://docs.aws.amazon.com/serverless-application-model/)
