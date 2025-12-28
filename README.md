# LiDAR API Publisher

ライトローバーのLiDARデータをAPIにPOSTするROSパッケージです。

## 概要

このパッケージは、YDLiDARから取得したスキャンデータをリアルタイムでAWS API（またはその他のREST API）に送信します。

## ファイル構成

```
lightrover_ros/
├── scripts/
│   ├── lidar_api_publisher.py       # LiDARデータをAPIにPOSTするノード
│   ├── lidar_api_publisher_lite.py  # データ削減（軽量）版ノード
│   └── verify_json_fix.py           # 検証用スクリプト
├── launch/
│   ├── lidar_api_publisher.launch   # API送信ノードのみを起動
│   └── lidar_with_api.launch        # LiDAR + API送信を同時起動
├── configuration_files/
│   ├── config.yaml                  # 設定ファイル（.gitignore対象、要作成）
│   └── config.sample.yaml           # 設定ファイルサンプル（雛形）
├── aws/
│   ├── aws_lambda_handler.py        # AWS Lambda関数サンプル
│   └── template.yaml                # AWS SAMテンプレート
├── Dockerfile                       # Docker実行環境定義
└── docker-compose.yml               # Docker実行構成
```

## セットアップ

### オプション1: Dockerを使用する場合（推奨）

ROS環境がセットアップされていない場合でも、Dockerを使用して簡単に実行できます。

1. **設定ファイルの準備**
   ```bash
   cp configuration_files/config.sample.yaml configuration_files/config.yaml
   ```
   `configuration_files/config.yaml` を編集し、`api_endpoint` や `api_key` を設定してください。

2. **ビルドと起動**
   ```bash
   docker-compose build
   docker-compose up
   ```

### オプション2: 既存のROS環境で使用する場合

1. **依存関係のインストール**
   ```bash
   # Python 2.7用のrequestsライブラリをインストール
   sudo apt-get update
   sudo apt-get install python-pip
   pip install requests
   ```

2. ** ROSパッケージへの追加**
   ```bash
   # scriptsディレクトリにファイルを配置
   cd ~/catkin_ws/src/lightrover_ros/scripts/
   # Pythonファイルのコピーと実行権限付与
   chmod +x lidar_api_publisher.py lidar_api_publisher_lite.py

   # launchディレクトリにファイルを配置
   cd ~/catkin_ws/src/lightrover_ros/launch/
   # launchファイルをコピー
   
   # configuration_filesの配置
   # lightrover_ros/configuration_files/config.yaml を作成
   ```

3. **ビルド**
   ```bash
   cd ~/catkin_ws
   catkin_make
   ```

## AWS APIのデプロイ

AWS SAMを使用してAPI Gateway + Lambda + DynamoDBのバックエンドを構築します。

```bash
cd aws/
sam build
sam deploy --guided
```

デプロイが完了すると、API Keyが生成されますが、セキュリティの観点から画面には表示されない場合があります。
AWSコンソールの「API Gateway」→「APIキー」、または以下のコマンドで確認し、`configuration_files/config.yaml` に設定してください。

```bash
# 生成されたAPI Keyの値を確認
aws apigateway get-api-keys --include-values
```


## 使用方法（ROS環境）

### LiDARとAPI送信を同時起動

```bash
roslaunch lightrover_ros lidar_with_api.launch
```

※ 設定は `configuration_files/config.yaml` から自動的に読み込まれます。

### API送信ノードのみを起動

```bash
roslaunch lightrover_ros lidar_api_publisher.launch
```

## 設定ファイル (config.yaml)

```yaml
# API設定
api_endpoint: "https://your-api.com/lidar"
api_key: "your-api-key"

# 送信設定
publish_rate: 1.0  # 1秒に1回
api_timeout: 5.0

# ロボット設定
robot_id: "lightrover_01"
```

## トラブルシューティング

### データが送信されない
- `rostopic list` で `/scan` が出ているか確認
- Dockerの場合、ホスト側で `roscore` が動いているか、ネットワーク設定を確認（`network_mode: host` 推奨）
- `configuration_files/config.yaml` のエンドポイント設定を確認

### APIエラー (400 Bad Request / 500 Internal Server Error)
- `verify_json_fix.py` を実行してJSONシリアライズに問題がないか確認
- AWS CloudWatch Logs でLambdaのエラーログを確認

### "inf" や "nan" の扱い
- 本パッケージでは、JSON規格違反を防ぐため、`inf` は最大距離に、`nan` は 0 に自動的に置換してから送信します。

## ライセンス

MIT License
