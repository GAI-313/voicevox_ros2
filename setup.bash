current_dir=$(dirname "$(readlink -f "$0")")

cd $current_dir

pypath=$current_dir/voicevox_ros2/voicevox_ros2/voicevox_ros2/voicevox_ros2_common

# bashrc
# チェックする文字列
search_string="#voicevox_ros2 setting
export LD_LIBRARY_PATH=\\\"\$LD_LIBRARY_PATH:\$current_dir/voicevox_core
export JTALK_PATH=\"\$current_dir/voicevox_core/open_jtalk_dic_utf_8-1.11"

# チェックするファイルパス
bashrc_path=~/.bashrc

if grep -qF "$search_string" "$bashrc_path"; then
    :
else
<<<<<<< HEAD
    pip install simpleaudio
=======
    pip install playsound
>>>>>>> 328f79a784210f9502acf5667c9fb91d438c21ef
    echo "LISTEN! : THIS PACKAGE NEED SOMEN PACKAGES. PLEASE ENTER THE PASSWORD"
    sudo apt update; sudo apt install -y python3-gst-1.0
    cd $current_dir/voicevox_core
    pip install voicevox_core-*.whl
    echo "#voicevox_ros2 setting" >> ~/.bashrc
    echo "export LD_LIBRARY_PATH=\"\$LD_LIBRARY_PATH:$current_dir/voicevox_core\"" >> ~/.bashrc
    echo "export JTALK_PATH=\"$current_dir/voicevox_core/open_jtalk_dic_utf_8-1.11\"" >> ~/.bashrc
    echo "export PYTHONPATH=\"\$PYTHONPATH:$pypath\"" >> ~/.bashrc
fi

# voicevox pkg install
if [ -d "$current_dir/voicevox_core" ]; then
    echo "Voicevox_core is installed"    
else
    curl -sSfL https://raw.githubusercontent.com/VOICEVOX/voicevox_core/8cf307df4412dc0db0b03c6957b83b032770c31a/scripts/downloads/download.sh | bash -s
    cd $current_sir/voicevox_core
    wget https://github.com/VOICEVOX/voicevox_core/releases/download/0.14.1/voicevox_core-0.14.1+cpu-cp38-abi3-linux_x86_64.whl
    pip install voicevox_core*.whl
    wget https://raw.githubusercontent.com/VOICEVOX/voicevox_core/406f6c41408836840b9a38489d0f670fb960f412/example/python/run.py
fi


