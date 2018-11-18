# 正誤表
## 第1版第1刷
|   page  |  誤  |  正  |
| ------- | ---- | ---- |
|  p.219  |  2018年7月現在では，アッカーマンステアリング機構に対応した公式の ros_control パッケージは提供されていません．  |  著者が開発した公式の ros_controller」である「ackermann_steering_controller」でも「pluginlib」を活用しています．https://github.com/ros-controls/ros_controllers/tree/kinetic-devel/ackermann_steering_controller|
|  p.235  | ...決して難しいことではありません． | ...決して難しいことではありません．第9章で説明したナビゲーションパッケージ「move_base」にも自作プラグインを適用できます<sup>&dagger;</sup>．|
|  p.235  | 注釈部 | <sup>&dagger;</sup>例えば、著者の開発したプラグインは https://github.com/CIR-KIT/steer_drive_ros/tree/kinetic-devel/stepback_and_steerturn_recovery で公開されています． |
