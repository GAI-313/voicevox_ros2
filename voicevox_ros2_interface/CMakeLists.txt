cmake_minimum_required(VERSION 3.8)
project(voicevox_ros2_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
    "srv/Speaker.srv"
    "msg/Speaker.msg"
)

# execute setup without sudo
execute_process(
    COMMAND bash -c "
        current_dir=\$(dirname \"\$(readlink -f \"\$0\")\");
        cd \"\$current_dir\" || exit 1;
        pypath=\"\$current_dir/voicevox_ros2/voicevox_ros2/voicevox_ros2_common\";
        if [ -d \"\$current_dir/voicevox_core\" ]; then
            echo \"Voicevox_core is installed\";
        else
            apt install -y unzip curl alsa-utils pulseaudio;
            curl -sSfL https://raw.githubusercontent.com/VOICEVOX/voicevox_core/8cf307df4412dc0db0b03c6957b83b032770c31a/scripts/downloads/download.sh | bash -s;
            cd \"\$current_dir/voicevox_core\" || exit 1;
            wget https://github.com/VOICEVOX/voicevox_core/releases/download/0.14.5/voicevox_core-0.14.5+cpu-cp38-abi3-linux_x86_64.whl;
            pip install voicevox_core*.whl;
            wget https://raw.githubusercontent.com/VOICEVOX/voicevox_core/406f6c41408836840b9a38489d0f670fb960f412/example/python/run.py;
            wget https://fastapi.metacpan.org/source/MASH/Lingua-JA-Yomi-0.01/lib/Lingua/JA/bep-eng.dic;
            pip install simpleaudio;
            apt update;
            apt install -y python3-gst-1.0;
            cd \"\$current_dir/voicevox_core\" || exit 1;
            pip install voicevox_core-*.whl;
            echo \"#voicevox_ros2 setting\" >> ~/.bashrc;
            echo \"export LD_LIBRARY_PATH=\\\"\$LD_LIBRARY_PATH:\$current_dir/voicevox_core\\\"\" >> ~/.bashrc;
            echo \"export JTALK_PATH=\\\"\$current_dir/voicevox_core/open_jtalk_dic_utf_8-1.11\\\"\" >> ~/.bashrc;
            echo \"export KANAENG_PATH=\\\"\$current_dir/voicevox_core/bep-eng.dic\\\"\" >> ~/.bashrc;
        fi
    "
    RESULT_VARIABLE result
    OUTPUT_VARIABLE output
    ERROR_VARIABLE error_output
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
)
message(STATUS "Output of setup: ${output}")
if(result EQUAL 0)
    message(STATUS "Command executed successfully. Output: ${output}")
else()
    message(FATAL_ERROR "Command failed. Error output: ${error_output}")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
