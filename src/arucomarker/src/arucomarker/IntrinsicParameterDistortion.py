#!/usr/bin/env python3

import pyzed.sl as sl

# ZED 카메라 객체 생성
zed = sl.Camera()

# 카메라 열기
if zed.open() != sl.ERROR_CODE.SUCCESS:
    print("ZED 2i 연결 실패")
    exit()

# 공장 캘리브레이션된 카메라 파라미터 가져오기
calib_params = zed.get_camera_information().camera_configuration.calibration_parameters

left_cam_params = calib_params.left_cam  # 왼쪽 카메라 파라미터

# 내부 카메라 행렬 (K)
K = [
    [left_cam_params.fx, 0, left_cam_params.cx],
    [0, left_cam_params.fy, left_cam_params.cy],
    [0, 0, 1]
]

# 왜곡 계수 (D)
D = list(left_cam_params.disto)

# 결과 출력
print("내부 카메라 행렬 (K):")
for row in K:
    print(row)

print("\n왜곡 계수 (D):")
print(D)

# 카메라 종료
zed.close()
