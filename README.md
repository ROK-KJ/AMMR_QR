# AruCo Marker Detection with ZED2i  
ZED2i를 활용한 AruCo Marker 검출  

## Requirements  

### ZED Camera를 위한 ZED SDK 설치  
ZED2i 카메라를 사용하기 위해, CUDA 버전에 맞는 ZED SDK를 설치해야 합니다.  

1. 아래 링크를 이용해서 CUDA 버전에 맞는 ZED SDK 다운로드  
   * [ZED SDK 다운로드](https://www.stereolabs.com/en-kr/developers/release)  
   * 본 프로젝트에서는 다음 버전을 설치하였습니다.  
     - `ZED_SDK_Ubuntu22_cuda12.1_v4.2.5.zstd.run`

2. 설치한 run 파일에 실행 권한 부여  

   ```bash
   sudo chmod +x ZED_SDK_Ubuntu22_cuda12.1_v4.2.5.zstd.run

3. 아래 명령어로 설치 진행
   ```bash
   ./ZED_SDK_Ubuntu22_cuda12.1_v4.2.5.zstd.run

4. Python API 설치
   ```bash
   cd /usr/local/zed/
   python3 get_python_api.py

5. 4번 설치 확인 명령어
   ```bash
   haryeong@haryeong-ROG-Strix-G18-G814JVR-G814JVR:/usr/local/zed$ python3
   Python 3.10.12 (main, Feb  4 2025, 14:57:36) [GCC 11.4.0] on linux
   Type "help", "copyright", "credits" or "license" for more information.
   >>> import pyzed.sl as sl
   >>> print("ZED Python API is successfully installed!")
   ZED Python API is successfully installed!

### ROS2 WITH ZED START
1. zed-ros2-wrapper 클론
   ```bash
   cd ~/catkin_hr/src/
   git clone https://github.com/stereolabs/zed-ros2-wrapper.git

2. 패키지 의존성 업데이트 및 설치
   ```bash
   cd ~/catkin_hr/
   sudo apt update
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y

3. colon build 실행
   ```bash
   colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc)
   source install/setup.bash

4. 빌드가 완료되면 패키지가 정상적으로 등록되었는 지 확인
   ```bash
   ros2 pkg list | grep zed

5. 아래와 같이 뜨면 성공
   ```bash
   haryeong@haryeong-ROG-Strix-G18-G814JVR-G814JVR:~$ ros2 pkg list | grep zed
   zed_components
   zed_msgs
   zed_wrapper

6. ~./bashrc 설정 업데이트
   ```bash
   echo 'source ~/catkin_hr/install/setup.bash' >> ~/.bashrc
   source ~/.bashrc

7. ZED node 실행 명령어
   ```bash
   ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i

### ArUCo Marker Detect를 위한 코드 실행
```bash
python3 MarkerDetect.py
