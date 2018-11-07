# rosbook packages [![Build Status](https://travis-ci.org/Nishida-Lab/rosbook_pkgs.svg?branch)](https://travis-ci.org/Nishida-Lab/rosbook_pkgs) 

<a href="//af.moshimo.com/af/c/click?a_id=721029&p_id=170&pc_id=185&pl_id=4062&s_v=b5Rz2P0601xu&url=http%3A%2F%2Fwww.amazon.co.jp%2Fexec%2Fobidos%2FASIN%2F462767581X" target="_blank" ><img src="https://images-na.ssl-images-amazon.com/images/I/5138TndGCwL.jpg" border="0" ></a><img src="//i.moshimo.com/af/i/impression?a_id=721029&p_id=170&pc_id=185&pl_id=4062" width="1" height="1" style="border:none;">

## インストール方法
### サンプルプログラムの取得
```bash
cd ~/<catkin_ws>/src
git clone https://github.com/Nishida-Lab/rosbook_pkgs.git
```

### 依存パッケージの解決
```bash
cd ~/<catkin_ws>
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### ビルド
```bash
cd ~/<catkin_ws>
catkin_make
source devel/setup.bash
```

## 本の概要

- タイトル：実用ロボット開発のためのROSプログラミング
- 著者：[九工大 西田 健](http://lab.cntl.kyutech.ac.jp/~nishida/)，[安川電機(+九工大) 森田賢](http://lab.cntl.kyutech.ac.jp/~nishida/member-ja.html)，[玉川大 岡田浩 之](http://www.tamagawa.jp/graduate/brain/staff/labs/okada.html)，[千葉工大 原祥 尭](https://www.slideshare.net/hara-y?utm_campaign=profiletracking&utm_medium=sssite&utm_source=ssslideview)，[信州大 山崎 公俊](http://www.ais.shinshu-u.ac.jp/ja/)，[九工大 田向 権](http://www.brain.kyutech.ac.jp/~tamukoh/)，[東大 垣内 洋平](http://www.jsk.t.u-tokyo.ac.jp/~youhei/)，[千葉大 大川 一也](http://www.em.eng.chiba-u.jp/~okawa/index.php?%E5%A4%A7%E5%B7%9D%20%E4%B8%80%E4%B9%9F)，[TORK 齋藤功](https://opensource-robotics.tokyo.jp/)，[田中 良道](https://qiita.com/RyodoTanaka)，[有田 裕太](https://github.com/AriYu)，[石田 裕太郎](http://www.brain.kyutech.ac.jp/~tamukoh/member/isida/)
- 出版日：2018/10/13
- 出版社：森北出版
- ISBN：978-4-627-67581-0
- ページ数：304ページ
- サイズ：B5変形

## 目次
- 第1章 ROSとは何か?
- 第2章 ROSの準備
- 第3章 ROSの仕組み
- 第4章 可視化とデバッグ
- 第5章 センサとアクチュエータ
- 第6章 3Dモデリングと制御シミュレーション
- 第7章 コンピュータビジョン
- 第8章 ポイントクラウド
- 第9章 ナビゲーション
- 第10章 ロボットの行動監視と制御
- 第11章 プラグインの開発
- 第12章 テストコードの作成
- 第13章 Travis CIとの連携
- 第14章 MATLABとの統合
- 第15章 システム統合(ロボットを使ってみよう)

## BibTex
### 日本語
```
@book{practical-ros-programming_jp,
  title     = {実用ロボット開発のためのROSプログラミング},
  author    = {西田健，森田賢，岡田浩之，原祥尭，山崎公俊，田向権，垣内洋平，大川一也，齋藤功，田中良道，有田裕太，石田裕太郎},
  month     = {10},
  year      = {2018},
  publisher = {森北出版株式会社},
  totalpages= {304},
  isbn      = {9784627675810},
  url       = {\url{https://github.com/Nishida-Lab/rosbook_pkgs}}
}
```

### 英語
```
@book{practical-ros-programming_en,
  title     = {ROS Robot Programming for practical robotics development},
  author    = {Takeshi Nishida，Masaru Morita，Hiroyuki Okada，Yoshitaka Hara，Kimitoshi Yamazaki，Hakaru Tamukoh，Takashi Kakiuchi，Kazuya Ohkawa，Isao Saito，Ryodo Tanaka, Yuta Arita，Yutaro Ishida},
  month     = {10},
  year      = {2018},
  publisher = {Morikita Publishing Co., Ltd.},
  totalpages= {304},
  isbn      = {9784627675810},
  url       = {\url{https://github.com/Nishida-Lab/rosbook_pkgs}}
}
```

## 書籍サイト
- [森北出版](http://www.morikita.co.jp/books/book/3240)
- [Amazon](https://www.amazon.co.jp/exec/obidos/ASIN/462767581X?tag=maftracking129219-22&linkCode=ure&creative=6339)
- [楽天ブックス](https://books.rakuten.co.jp/rb/15628639/?scid=af_pc_etc&sc2id=af_103_1_10000645)
- [honto](https://honto.jp/netstore/search.html?gnrcd=1&k=462767581X&srchf=1&srchGnrNm=1)
- [図書館](https://calil.jp/book/462767581X)
