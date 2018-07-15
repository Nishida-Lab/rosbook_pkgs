# rosbook packages [![Build Status](https://travis-ci.org/Nishida-Lab/rosbook_pkgs.svg?branch)](https://travis-ci.org/Nishida-Lab/rosbook_pkgs) 

このリポジトリは西田健先生が出版するROS本のサンプルコードをまとめるものです。
このリポジトリをワークスペースにクローンすれば動くようにすべきだと思います。

フォルダ構成ですが、`catkin_create_pkg`でpkgを作成し、C++コードは`src`と`include`ディレクトリに、Pythonコードは`script`ディレクトリに格納する感じで行きたいと思います。

コーディング規約は以下に従うようにしましょう。
- C++
  * http://wiki.ros.org/CppStyleGuide
- Python
  * http://wiki.ros.org/PyStyleGuide
  
パッケージを追加する単位でブランチを切る感じでお願いします。
