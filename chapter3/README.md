### packageを作る

```shell
$ cd <your_catkin_ws>/src
$ catkin_create_pkg chapter3 roscpp rospy
$ cd chapter3
$ mkdir scripts
```

### Pythonプログラムを追加する

#### talker.py

上記で作成した`scripts`ディレクトリに`talker.py`という名前で以下のプログラムを保存する．

./chapter3/scripts/talker.py

#### listener.py

`talker.py`と同様に`scripts`ディレクトリに`listener.pyt`という名前で以下のプログラムを保存する．

./chapter3/scripts/listener.py

### pythonプログラムを実行可能にする

```shell
roscd chapter3/scripts
chmod +x *.py
```

### pythonプログラムを実行する

```shell
$ roscore
```
別ウィンドウで
```shell
$ rosrun chapter3 takler.py
```
別ウィンドウで
```shell
$ rosrun chapter3 listener.py
```
