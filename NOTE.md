# パッケージの作成
基本形
```
ros2 pkg create PKG_NAME
```
依存関係を指定する
```
ros2 pkg create PKG_NAME --dependencies DEPEND_PKG1 DEPEND_PKG2
```
サービスなどを作りたいときは以下の依存関係を入れる。

- ```rosidl_default_generator```<br>
    package.xml にその他依存関係などを追記する必要がある。

# ament_python
## setup.py

# その他 Tips
　ビルドタイプが ament_python の環境下でサービスやメッセージを作成する文献が見当たらない。なので新たにパッケージを作ったほうが無難。
