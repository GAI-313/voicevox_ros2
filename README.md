# VoiceVox ROS2

## Install

1. **voicevox_core をインストールする**<br>
    任意の場所で以下のコマンドを実行し VoiceVox_Core をダウンロードします．`VERSION` は任意のバージョンを [こちら](https://github.com/VOICEVOX/voicevox_core/releases) から選択してください．

    - X86
        ```bash
        export VERSION=0.16.1
        wget https://github.com/VOICEVOX/voicevox_core/releases/download/${VERSION}/download-linux-x64
        wget https://github.com/VOICEVOX/voicevox_core/releases/download/${VERSION}/voicevox_core-${VERSION}-cp310-abi3-manylinux_2_34_x86_64.whl
        ```
    - ARM64
        ```bash
        export VERSION=0.16.1
        wget https://github.com/VOICEVOX/voicevox_core/releases/download/${VERSION}/download-linux-x64
        wget https://github.com/VOICEVOX/voicevox_core/releases/download/${VERSION}/voicevox_core-${VERSION}-cp310-abi3-manylinux_2_34_aarch64.whl
        ```

    ダウンロードしたファイルをインストールします．
    ```bash
    # x86
    chmod +x download-linux-x64
    # ARM64
    chmod +x download-linux-arm64
    ```
    ```bash
    # x86 and ARM64
    yes | ./download-linux-arm64
    # use CUDA
    yes | ./download-linux-arm64 --devices cuda
    ```
    次に Python API もインストールします．
    ```bash
    # x86
    pip install voicevox_core-${VERSION}-cp310-abi3-manylinux_2_34_x86_64.whl
    # ARM64
    pip install voicevox_core-${VERSION}-cp310-abi3-manylinux_2_34_aarch64.whl
    ```

1. **依存関係のインストール**<br>
    このパッケージでは音声再生に `ALSA` を使います．以下のコマンドを実行して必要パッケージをインストールします．
    ```bash
    sudo apt install -y alsa-base
    ```

1. **パッケージのビルド**<br>
    このパッケージをワークスペースにクローンしたら以下のコマンドをワークスペース直下で実行してください．
    ```bash
    colcon build --symlink-install --packages-up-to voicevox_ros2
    . install/setup.bash
    ```

## Usage
　インストール後，以下のコマンドを実行して voicevox_ros2 を起動します．`</path/to/voicevox_core>/` を voicevox_core をダウンロード，インストールしたパスに書き換えてください．
```bash
ros2 run voicevox_ros2 voicevox_ros2 --ros-args \
    -p voicevox_onnxruntime_path:=</path/to/voicevox_core>/onnxruntime/lib/ \
    -p voicevox_model_path:=</path/to/voicevox_core>/models/vvms/0.vvm \
    -p open_jtalk_dict_dir:=</path/to/voicevox_core>/dict/open_jtalk_dic_utf_8-1.11
```
起動後以下のように **`VoiceVox ROS2 Start!`** と表示されたら成功です．
```bash
$ ros2 run voicevox_ros2 voicevox_ros2 --ros-args \
    -p voicevox_onnxruntime_path:=/ws/onnxruntime/lib/ \
    -p voicevox_model_path:=/ws/models/vvms/0.vvm \
    -p open_jtalk_dict_dir:=/ws/dict/open_jtalk_dic_utf_8-1.11

[INFO] [1757237824.950648245] [voicevox_ros2]: voicevox_core parameters: 
    /ws/voicevox_core/onnxruntime/lib/libvoicevox_onnxruntime.so.1.17.3
    /ws/voicevox_core/models/vvms/0.vvm
    /ws/voicevox_core/dict/open_jtalk_dic_utf_8-1.11
[INFO] [1757237824.965820257] [voicevox_ros2]: load model: /ws/voicevox_core/models/vvms/0.vvm
[INFO] [1757237825.899022126] [voicevox_ros2]: VoiceVox ROS2 Start!
```

> [!CAUTION]
    以下のようなエラーが発生した場合，各パラメータで指定された voicevox_core のパスが間違えている可能性があります．
    ```
    Exception: /ws/onnxruntime/lib/libvoicevox_onnxruntime.so.1.17.3: cannot open shared object file: No such file or directory
    ...
    voicevox_core.InitInferenceRuntimeError: ONNX Runtimeのロードまたは初期化ができませんでし
    ```

---

　voicevox_ros2 ノード起動後，別ターミナルで以下のコマンドを実行し，「test」と発話すれば成功です．
```bash
ros2 service call /speak voicevox_ros2_msgs/srv/Speaking 'text: test'
```
　`Speaking` サービスのフィールド構成は以下のコマンドで確認してください．
```bash
ros2 interface show voicevox_ros2_msgs/srv/Speaking 
```
　発話に失敗したとき，サービスクライアントに `False` が返されます（以下ブロック例）．このときパラメータ `voicevox_model_path` に指定された speakser_id が存在しないことが原因と考えられます．詳細は実行中の voicevox_ros2 に出力されるエラーメッセージを参照してください．vvm ファイルと speakser_id の対応は **Info** セクションを参照してください．
```
$ ros2 service call /speak voicevox_ros2_msgs/srv/Speaking '{text: test, speaker_id: 21}'
requester: making request: voicevox_ros2_msgs.srv.Speaking_Request(speaker_id=21, text='test', enable_interrogative_upspeak=True, pitch_scale=0.0, intonation_scale=0.0, speed_scale=0.0, volume_scale=0.0)

response:
voicevox_ros2_msgs.srv.Speaking_Response(success=False)
```

## Info
　パラメータで読み込む vvm ファイルと発話キャラクターは以下のように対応しています．

| VVMファイル名 | 話者名 | スタイル名 | speaker_id |
|---|---|---|---|
| 0.vvm | 四国めたん | ノーマル | 2 |
| 0.vvm | 四国めたん | あまあま | 0 |
| 0.vvm | 四国めたん | ツンツン | 6 |
| 0.vvm | 四国めたん | セクシー | 4 |
| 0.vvm | ずんだもん | ノーマル | 3 |
| 0.vvm | ずんだもん | あまあま | 1 |
| 0.vvm | ずんだもん | ツンツン | 7 |
| 0.vvm | ずんだもん | セクシー | 5 |
| 0.vvm | 春日部つむぎ | ノーマル | 8 |
| 0.vvm | 雨晴はう | ノーマル | 10 |
| 1.vvm | 冥鳴ひまり | ノーマル | 14 |
| 2.vvm | 九州そら | ノーマル | 16 |
| 2.vvm | 九州そら | あまあま | 15 |
| 2.vvm | 九州そら | ツンツン | 18 |
| 2.vvm | 九州そら | セクシー | 17 |
| 3.vvm | 波音リツ | ノーマル | 9 |
| 3.vvm | 波音リツ | クイーン | 65 |
| 3.vvm | 中国うさぎ | ノーマル | 61 |
| 3.vvm | 中国うさぎ | おどろき | 62 |
| 3.vvm | 中国うさぎ | こわがり | 63 |
| 3.vvm | 中国うさぎ | へろへろ | 64 |
| 4.vvm | 玄野武宏 | ノーマル | 11 |
| 4.vvm | 剣崎雌雄 | ノーマル | 21 |
| 5.vvm | 四国めたん | ささやき | 36 |
| 5.vvm | 四国めたん | ヒソヒソ | 37 |
| 5.vvm | ずんだもん | ささやき | 22 |
| 5.vvm | ずんだもん | ヒソヒソ | 38 |
| 5.vvm | 九州そら | ささやき | 19 |
| 6.vvm | No.7 | ノーマル | 29 |
| 6.vvm | No.7 | アナウンス | 30 |
| 6.vvm | No.7 | 読み聞かせ | 31 |
| 7.vvm | 後鬼 | 人間ver. | 27 |
| 7.vvm | 後鬼 | ぬいぐるみver. | 28 |
| 8.vvm | WhiteCUL | ノーマル | 23 |
| 8.vvm | WhiteCUL | たのしい | 24 |
| 8.vvm | WhiteCUL | かなしい | 25 |
| 8.vvm | WhiteCUL | びえーん | 26 |
| 9.vvm | 白上虎太郎 | ふつう | 12 |
| 9.vvm | 白上虎太郎 | わーい | 32 |
| 9.vvm | 白上虎太郎 | びくびく | 33 |
| 9.vvm | 白上虎太郎 | おこ | 34 |
| 9.vvm | 白上虎太郎 | びえーん | 35 |
| 10.vvm | 玄野武宏 | 喜び | 39 |
| 10.vvm | 玄野武宏 | ツンギレ | 40 |
| 10.vvm | 玄野武宏 | 悲しみ | 41 |
| 10.vvm | ちび式じい | ノーマル | 42 |
| 11.vvm | 櫻歌ミコ | ノーマル | 43 |
| 11.vvm | 櫻歌ミコ | 第二形態 | 44 |
| 11.vvm | 櫻歌ミコ | ロリ | 45 |
| 11.vvm | ナースロボ＿タイプＴ | ノーマル | 47 |
| 11.vvm | ナースロボ＿タイプＴ | 楽々 | 48 |
| 11.vvm | ナースロボ＿タイプＴ | 恐怖 | 49 |
| 11.vvm | ナースロボ＿タイプＴ | 内緒話 | 50 |
| 12.vvm | †聖騎士 紅桜† | ノーマル | 51 |
| 12.vvm | 雀松朱司 | ノーマル | 52 |
| 12.vvm | 麒ヶ島宗麟 | ノーマル | 53 |
| 13.vvm | 春歌ナナ | ノーマル | 54 |
| 13.vvm | 猫使アル | ノーマル | 55 |
| 13.vvm | 猫使アル | おちつき | 56 |
| 13.vvm | 猫使アル | うきうき | 57 |
| 13.vvm | 猫使ビィ | ノーマル | 58 |
| 13.vvm | 猫使ビィ | おちつき | 59 |
| 13.vvm | 猫使ビィ | 人見知り | 60 |
| 14.vvm | 栗田まろん | ノーマル | 67 |
| 14.vvm | あいえるたん | ノーマル | 68 |
| 14.vvm | 満別花丸 | ノーマル | 69 |
| 14.vvm | 満別花丸 | 元気 | 70 |
| 14.vvm | 満別花丸 | ささやき | 71 |
| 14.vvm | 満別花丸 | ぶりっ子 | 72 |
| 14.vvm | 満別花丸 | ボーイ | 73 |
| 14.vvm | 琴詠ニア | ノーマル | 74 |
| 15.vvm | ずんだもん | ヘロヘロ | 75 |
| 15.vvm | ずんだもん | なみだめ | 76 |
| 15.vvm | 青山龍星 | ノーマル | 13 |
| 15.vvm | 青山龍星 | 熱血 | 81 |
| 15.vvm | 青山龍星 | 不機嫌 | 82 |
| 15.vvm | 青山龍星 | 喜び | 83 |
| 15.vvm | 青山龍星 | しっとり | 84 |
| 15.vvm | 青山龍星 | かなしみ | 85 |
| 15.vvm | 青山龍星 | 囁き | 86 |
| 15.vvm | もち子さん | ノーマル | 20 |
| 15.vvm | もち子さん | セクシー／あん子 | 66 |
| 15.vvm | もち子さん | 泣き | 77 |
| 15.vvm | もち子さん | 怒り | 78 |
| 15.vvm | もち子さん | 喜び | 79 |
| 15.vvm | もち子さん | のんびり | 80 |
| 15.vvm | 小夜/SAYO | ノーマル | 46 |
| 16.vvm | 後鬼 | 人間（怒り）ver. | 87 |
| 16.vvm | 後鬼 | 鬼ver. | 88 |
| 17.vvm | Voidoll | ノーマル | 89 |
| 18.vvm | ぞん子 | ノーマル | 90 |
| 18.vvm | ぞん子 | 低血圧 | 91 |
| 18.vvm | ぞん子 | 覚醒 | 92 |
| 18.vvm | ぞん子 | 実況風 | 93 |
| 18.vvm | 中部つるぎ | ノーマル | 94 |
| 18.vvm | 中部つるぎ | 怒り | 95 |
| 18.vvm | 中部つるぎ | ヒソヒソ | 96 |
| 18.vvm | 中部つるぎ | おどおど | 97 |
| 18.vvm | 中部つるぎ | 絶望と敗北 | 98 |
| 19.vvm | 離途 | ノーマル | 99 |
| 19.vvm | 離途 | シリアス | 101 |
| 19.vvm | 黒沢冴白 | ノーマル | 100 |
| 20.vvm | ユーレイちゃん | ノーマル | 102 |
| 20.vvm | ユーレイちゃん | 甘々 | 103 |
| 20.vvm | ユーレイちゃん | 哀しみ | 104 |
| 20.vvm | ユーレイちゃん | ささやき | 105 |
| 20.vvm | ユーレイちゃん | ツクモちゃん | 106 |
| 21.vvm | 猫使アル | つよつよ | 110 |
| 21.vvm | 猫使アル | へろへろ | 111 |
| 21.vvm | 猫使ビィ | つよつよ | 112 |
| 21.vvm | 東北ずん子 | ノーマル | 107 |
| 21.vvm | 東北きりたん | ノーマル | 108 |
| 21.vvm | 東北イタコ | ノーマル | 109 |
