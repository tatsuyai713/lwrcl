# LWRCL for Fast DDS

このディレクトリには Fast DDS をバックエンドとした LWRCL のコアライブラリとビルドスクリプトが含まれています。

詳細なドキュメント（API リファレンス、インストール方法、サンプルなど）は、プロジェクトルートの [README.md](../README.md) をご参照ください。

## ディレクトリ構成

```
lwrcl/
├── lwrcl/                # LWRCL コアライブラリ
│   ├── lwrcl/           # 基本ライブラリ
│   ├── tf2/             # tf2 ライブラリ
│   └── tf2_ros/         # tf2_ros ライブラリ
├── apps/                 # サンプルアプリケーション
├── data_types/           # ROS 2 互換データ型
└── libraries/            # サポートライブラリ
```

## ビルドスクリプト

| スクリプト | 説明 |
|-----------|------|
| `build_libraries.sh` | サポートライブラリのビルド |
| `build_data_types.sh` | ROS データ型のビルド |
| `build_lwrcl.sh` | LWRCL コアライブラリのビルド |
| `build_apps.sh` | サンプルアプリケーションのビルド |
| `build_*_qnx.sh` | QNX 8.0 用ビルドスクリプト |
