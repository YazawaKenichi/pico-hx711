# pico-hx711

Raspberry Pi Pico で HX711 の値を読み、micro-ROS を用いて ROS 2 トピック通信を行う

# 環境構築
## micro_ros_raspberrypi_pico_sdk の用意
- 公式 GitHub に任せる [micro-ROS/micro_ros_raspberrypi_pico_sdk](https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk)

```
/path/to/project_root
    |- pico-sdk
    |   |- pico_sdk_init.cmake
    |- micro_ros_raspberrypi_pico_sdk
```

ファイル構造がこんな感じになってればおけ

## セットアップ
このリポジトリをクローン

# ライセンス
`include/libmicroros` は [micro-ROS/micro_ros_raspberrypi_pico_sdk](https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk) からコピーしてパクって来たもの

もとのリポジトリに Apache-2.0 ライセンスが適用されているので本リポジトリも Apache-2.0 になるはず

私としては Apache-2.0 あんま好きじゃないので `libmicroros` 以外は MIT にしたい

