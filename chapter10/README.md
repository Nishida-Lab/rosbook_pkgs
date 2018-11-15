# 正誤表
|   page  |  誤  |  正  |
| ------- | ---- | ---- |
|  p.216  | ソースコード 10-6 `spawn_object.launch` | 後述の通り置き換え． |
|  p.216  | ... テーブルにビールとコップが置かれている状況になります． | ... テーブルにビールとコップが置かれている状況になります．このサンプルでは「timed_roslaunch」を使用してビールとコップの設置時間を遅らせることで、安定した環境構築ができる工夫がされています． |

### spawn_object.launch
```xml
<launch>
  <param name="table_sdf" textfile="$(find smach_sample)/sdf/table/model-1_4.sdf" />
  <node name="$(anon spawn1)" pkg="gazebo_ros" type="spawn_model" output="screen"
      args="-sdf -param table_sdf -model table_model -x 1.6 -z -0.4 -Y 1.570" />
  <include file="$(find timed_roslaunch)/launch/timed_roslaunch.launch">
    <arg name="time" value="1" />
    <arg name="pkg" value="smach_sample" />
    <arg name="file" value="spawn_beer_cup.launch" />
  </include>
</launch>
```

# トラブルシューティング
## spawn_object.launch の使用について

### 現象
下記コマンドでオブジェクトをspawnする際にテーブル上に配置されるはずのビール缶やカップがテーブルの下に配置されてしまう場合があります。

```bash
roslaunch smach_sample spawn_object.launch
```

### 対処法
#### spawnのタイミングを考慮した launch ファイルの利用
下記 launch コマンドにより、テーブルをspawnしてから1秒後にビール缶とカップがspawnされます。


```bash
roslaunch smach_sample spawn_object_with_delay.launch
```

内部的には、[timed_roslaunch](http://wiki.ros.org/timed_roslaunch)というパッケージを利用することでビール缶とカップのspawnタイミングを遅延させています。

#### 順序起動
この場合、下記のコマンドを順番に起動することにより、テーブル上に確実にビール缶とカップがテーブル上に配置されます。
- テーブルのspawn
  - 最初にテーブルのみをspawnします。

```bash
roslaunch smach_sample spawn_table.launch
```

- ビール缶とカップのspawn
  - 上記コマンドでテーブルがspawnされたことを確認した後に、下記コマンドを実行します。

```bash
roslaunch smach_sample spawn_beer_cup.launch
```
