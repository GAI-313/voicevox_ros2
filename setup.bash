curl -sSfL https://raw.githubusercontent.com/VOICEVOX/voicevox_core/8cf307df4412dc0db0b03c6957b83b032770c31a/scripts/downloads/download.sh | bash -s
cd voicevox_core
wget https://github.com/VOICEVOX/voicevox_core/releases/download/0.14.1/voicevox_core-0.14.1+cpu-cp38-abi3-linux_x86_64.whl
pip install voicevox_core-0.14.1+cpu-cp38-abi3-linux_x86_64.whl
wget https://raw.githubusercontent.com/VOICEVOX/voicevox_core/406f6c41408836840b9a38489d0f670fb960f412/example/python/run.py
echo 'export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$PWD/voicevox_core"' >> ~/.bashrc
echo 'export JTALK_PATH="$LD_LIBRARY_PATH:$PWD/voicevox_core/open_jtalk_dic_utf_8-1.11"' >> ~/.bashrc
