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
