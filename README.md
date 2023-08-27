# voicevox_ros2
## インストール
　このパッケージを最初にビルドすると、
VOICEVOX
の依存関係や辞書データを自動でインストール、ダウンロードします。
```
/path/to/voicevox_core/setup.sh
```
　インストールが完了したらビルドを実行してください。
このパッケージのみをビルドする場合
```
cd /path/to/your_workspace
colcon build --pacakges-select voicevox_ros2
```
ワークスペースすべてのパッケージをビルドする場合
```
cd /path/to/your_workspace
colcon build --symlink-install
```
または
```
cd /path/to/your_workspace
colcon build
```

# 使用方法
## 起動と確認
　VoiceVox_Ros2_Core （メインノード）を起動するコマンド
```
ros2 run voicevox_ros2 voicevox_ros2_core
```
　正常に VoiceVox_Ros2_Core が起動すると、以下のようなログが表示されます。
```
[INFO] [xxxx.xxxx] [voicevox_ros2_core]: start voicevox_ros2 
```
　正常に VoiceVox_Ros2_Core が動作しているか確認したい場合は、以下のコマンドで確認できます。

- Publisher によるテスト
    ```
    ros2 run voicevox_ros2 pub
    ```
- Service によるテスト
    ```
    ros2 run voicevox_ros2 srv
    ```

　正常に VoiceVox_Ros2_Core が動作すると、以下のようなログが表示され、音声が再生されます。
```
[INFO] [xxxx.xxxx] [voicevox_ros2_core]: GENERATE voice
```

## Publisher
　Pub&Sub 通信で VoiceVox_Ros2 を使用する場合

- **トピック**<br>
    ***/voicevox_ros2/speaker***
- **メッセージタイプ**<br>
    ***voicevox_interface/msg/Speaker***
    ```
    Speaker.text # 発音させたい文字列
    Speaker.id # 発音するキャラクターIDを指定
    ```
    キャラクターIDについては、
    **[キャラクターID一覧](#id)**
    を参照してください。
    
## Service
　サービスから VoiceVox_Ros2 を使用する場合

- **クライアント**<br>
    ***/voicevox_ros2/speaker_srv***
- **メッセージタイプ**<br>
    ***voicevox_interface/srv/Speaker***
    ```
    Speaker.text # 発音させたい文字列
    Speaker.id # 発音するキャラクターIDを指定
    ---
    return bool
    ```
    キャラクターIDについては、
    **[キャラクターID一覧](#id)**
    を参照してください。

## 関数 tts_speaker を使用する
　voicevox_ros2 には簡易的に voicevox_ros2 を使用するための関数
```voicevox_ros2_tts.tts_speaker```
が用意されており、
```setup.sh```
を実行していれば、すぐにこの関数を使用することができます。この関数を使用するには、以下のサンプルコードを参考にしてください。
```python
#!/usr/bin/emv python3

## ROS2 ノード関連
from rclpy.node import Node
import rclpy
## tts_speaker 関数をインポート
from voicevox_ros2_tts import tts_speaker

# debug
if __name__ == '__main__':
    ## 関数を使用する前にノードを立ててください。
    rclpy.init()
    node = rclpy.create_node('vvr2_common_test')
    ## 関数を使用する
    tts_speaker(node, text='こんにちは！', id=26)
    ## 終了
    print('done')
    rclpy.shutdown()
    node.destroy_node()
```

　```tts_speaker```
は、以下の引数を求めます。

- **node（必須）**<br>
    ```rclpy.node.Node```
    または
    ```rclpy.create_node```
    により定義された
    **Node オブジェクト**
    を代入してください。

- **text**<br>
    デフォルト値：
    ```"テキスト引数が指定されていません。"```
    (str)<br>
    VoiceVox に発音させたいテキストを入力してください。

- **id**<br>
    デフォルト値：
    ```3```
    (int)<br>
    VoiceVox から発音するキャラクターIDを入力してください。デフォルトキャラクターは
    **ずんだもん（ノーマル）**
    です。キャラクターIDについては、
    **[キャラクターID一覧](#id)**
    を参照してください。

<a id="id"></a>
# キャラクター ID 一覧
|キャラクター名|スタイル|ID|
|:----|:----|:----|
|四国めたん|ノーマル|2|
||あまあま|0|
||ツンツン|6|
||セクシー|4|
||ささやき|36|
||ヒソヒソ|37|
|ずんだもん|ノーマル|3|
||あまあま|1|
||ツンツン|7|
||セクシー|5|
||ささやき|22|
||ヒソヒソ|38|
|春日部つむぎ|ノーマル|8|
|雨晴はう|ノーマル|10|
|波音リツ|ノーマル|9|
|玄野武宏|ノーマル|11|
||喜び|39|
||ツンギレ|40|
||悲しみ|41|
|白上虎太郎|ふつう|12|
||わーい|32|
||びくびく|33|
||おこ|34|
||びえーん|35|
|青山龍星|ノーマル|13|
|冥鳴ひまり|ノーマル|14|
|九州そら|ノーマル|16|
||あまあま|15|
||ツンツン|18|
||セクシー|17|
||ささやき|19|
|もち子さん|ノーマル|20|
|剣崎雌雄|ノーマル|21|
|WhiteCUL|ノーマル|23|
||たのしい|24|
||かなしい|25|
||びえーん|26|
|後鬼|人間ver.|27|
||ぬいぐるみver.|28|
|No.7|ノーマル|29|
||アナウンス|30|
||読み聞かせ|31|
|ちび式じい|ノーマル|42|
|櫻歌ミコ|ノーマル|43|
||第二形態|44|
||ロリ|45|
|小夜/SAYO|ノーマル|46|
|ナースロボ＿タイプＴ|ノーマル|47|
||楽々|48|
||恐怖|49|
||内緒話|50|
