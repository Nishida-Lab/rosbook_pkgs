## spawn_object.launch の使用について

### 現象
下記コマンドでオブジェクトをspawnする際にテーブル上に配置されるはずのビール缶やカップがテーブルの下に配置されてしまう場合があります。

```bash
roslaunch smach_sample spawn_object.launch
```

### 対処法
この場合、下記のコマンドを順番に起動することにより、テーブル上に確実にビール缶とカップがテーブル上に配置されます。

```bash
roslaunch smach_sample spawn_table.launch
```

上記コマンドでテーブルがspawnされたことを確認した後に、下記コマンドを実行する。

```bash
roslaunch smach_sample spawn_beer_cup.launch
```
