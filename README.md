# voicevox_ros2
## ROS1 版 voicevox_ros
　ROS1 Noetic に対応した voicevox tts パッケージ、
***voicevox_ros***
は、以下のリポジトリを参照してください。

- https://github.com/GAI-313/voicevox_ros/tree/master

## アップデート
　英単語をカタカナ変換して発音させることでカタコトながら英単語の発話に対応させました。
　
## インストール
　ワークスペースディレクトリの ```/src``` にこのパッケージをクローンしてください。
```
cd path/to/your_ws/src
git clone https://github.com/GAI-313/voicevox_ros2.git
```
　次に、```colcon build``` でパッケージをビルドしてください。
```
cd /path/to/your_ws
colcon build --packages-up-to voicevox_ros2
```
```--packages-up-to``` オプションを使用することで特定のパッケージとそれに依存するパッケージをビルドします。<br>
　依存パッケージ ```voicevox_ros2_interface``` がビルドされると自動的に **VOICEVOX_core** がインストールされます。

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

## メソッド tts_speaker を使用する
　voicevox_ros2 には簡易的に voicevox_ros2 を使用するためのメソッド
```voicevox_ros2_tts.tts_speaker```
が用意されており、
```setup.sh```
を実行していれば、すぐにこのメソッドを使用することができます。このメソッドを使用するには、以下のサンプルコードを参考にしてください。
```python
#!/usr/bin/emv python3

## ROS2 ノード関連
from rclpy.node import Node
import rclpy
## tts_speaker メソッドをインポート
from voicevox_ros2_tts import tts_speaker

# debug
if __name__ == '__main__':
    ## メソッドを使用する前にノードを立ててください。
    rclpy.init()
    node = rclpy.create_node('vvr2_common_test')
    ## メソッドを使用する
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

- **timeout**<br>
    デフォルト値：
    ```5```
    (int)<br>
    *voicevox_ros2_core*
    ノードが起動するまでの待機時間を設定します。単位は
    **秒(sec)**
    です。指定した時間を過ぎたらメソッドは正常に終了します。

- **wait**<br>
    デフォルト値：
    ```True```
    (bool)<br>
    発音が完了するまで待機するかどうかを設定します。
    - ***True***<br>
        発音が完了するまで待機します。
    - ***False***<br>
        発音が完了しなくても後続のプロセスを実行します。

## StateMachine（Smach、状態遷移）に組み込む
　状態遷移プロセスに VoiceVox_Ros2 を組み込む場合、以下のモジュールとメソッドをインポートしてください。
```python
from voicevox_ros2_tts import tts_SpeakerState
```
　メソッド
*tts_SpeakerState*
には以下の引数を要求します。

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

- **timeout**<br>
    デフォルト値：
    ```5```
    (int)<br>
    *voicevox_ros2_core*
    ノードが起動するまでの待機時間を設定します。単位は
    **秒(sec)**
    です。指定した時間を過ぎたら state は
    ```failure```
    を返します。

- **wait**<br>
    デフォルト値：
    ```True```
    (bool)<br>
    　発音が完了するまで待機するかどうかを設定します。
    - ***True***<br>
        発音が完了するまで待機します。
    - ***False***<br>
        発音が完了しなくても後続のプロセスを実行します。

　このステートは以下の結果を返します。

- ***success***<br>
    　正常に
    *voicevox_ros2_node*
    がプロセスを遂行した。
    
- ***timeout***<br>
    　プロセスが引数
    ```timeout```
    まで正常に動作しなかった。
    
- ***failure***<br>
    　プロセスが重篤なエラーにより停止した

## voice_saver
　```voice_saver``` を使用すると、指定した場所に任意の音声ファイルを保存します。このノードの使用方法は以下のとおりです。ヘルプを表示するにはオプション ```-h``` をつけます。
```
ros2 run voicevox_ros2 voice_saver -h
```
　カレントディレクトリに ```こんにちは``` と発音する音声ファイルを生成するには、オプション ```-t``` に発話したいテキストを入力します。
 ```
ros2 run voicevox_ros2 voice_saver -t こんにちは
```
　オプションは以下のとおりです。
|オプション|デフォルト値|概要|
|:---:|:---:|:---|
|-t , --text|なし|発話したいテキストを入力してください。必須引数です。|
|-i , --id|3|発話させるキャラクターIDを設定します。<br>キャラクターIDは **[キャラクターID一覧](#id)** を参照してください。|
|-p , --path|カレントディレクトリ|ファイルの保存先を絶対パスで選択します。|
|-f , --file_name|output|ファイル名をしてします。拡張子を入力する必要はありません。|

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

# リマインド
- **カタコト英語発音に対応（対応済み）**<br>
　VoiceVoxは仕様上
**英語をネイティブに発音できません**
。しかし、今後カタコトではあるものの英単語を発音できるように対応させる予定です。

|発音させたい英単語例|現在の発音スペル|今後修正予定のスペル|
|:----|:----|:----|
|ROS Humble|アールオーエス　エイチユーエムビーエルイー|ロス　ハンブル|
