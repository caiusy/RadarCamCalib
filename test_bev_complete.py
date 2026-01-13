#!/usr/bin/env python3
"""
完整的BEV系统验证测试
测试所有用户要求的功能
"""
import sys
sys.path.insert(0, '/Users/caius/Downloads/radardemo')

from calib_manager import CalibrationManager
import numpy as np

print("=" * 60)
print("BEV系统完整测试")
print("=" * 60)

# 1. 初始化
mgr = CalibrationManager()
mgr.update_camera_params(height=1.5, fx=1000, fy=1000, cx=640, cy=480)
mgr.update_radar_params(yaw=0, x_offset=3.5, y_offset=0)
print("\n1. ✅ CalibrationManager初始化")
print(f"   Camera: H={mgr.camera.height}, fx={mgr.camera.fx}, pitch={mgr.camera.pitch}")
print(f"   Radar: yaw={mgr.radar.yaw}, x={mgr.radar.x_offset}, y={mgr.radar.y_offset}")

# 2. 测试雷达投影
radar_x, radar_y = 50, 0
bev1 = mgr.radar_to_bev(radar_x, radar_y)
print(f"\n2. ✅ 雷达投影: ({radar_x}, {radar_y}) → BEV {bev1}")

# 3. 修改雷达参数，测试是否影响投影
mgr.update_radar_params(yaw=0, x_offset=4.0, y_offset=0)
bev2 = mgr.radar_to_bev(radar_x, radar_y)
print(f"\n3. {'✅' if bev1 != bev2 else '❌'} 雷达参数改变后投影变化:")
print(f"   X offset 3.5→4.0: BEV {bev1} → {bev2}")
print(f"   {'参数更新生效！' if bev1[0] != bev2[0] else '参数未生效！'}")

# 4. 计算pitch
class FakeLane:
    def __init__(self, start, end):
        self.start, self.end = start, end

lanes = [
    FakeLane((200, 400), (300, 200)),
    FakeLane((800, 400), (700, 200))
]
pitch = mgr.compute_pitch_from_lanes(lanes)
print(f"\n4. ✅ Pitch计算: {pitch:.4f} rad ({np.degrees(pitch):.2f}°)")
print(f"   Stored: {mgr.camera.pitch:.4f} rad")

# 5. 测试像素投影
pixel_u, pixel_v = 640, 400
image_bev = mgr.image_to_bev(pixel_u, pixel_v)
print(f"\n5. {'✅' if image_bev else '❌'} 像素投影:")
print(f"   Pixel ({pixel_u}, {pixel_v}) → BEV {image_bev}")
if image_bev:
    print(f"   投影成功！位置: ({image_bev[0]:.2f}m, {image_bev[1]:.2f}m)")
else:
    print(f"   ❌ 投影失败！返回None")

# 6. 修改相机参数后测试pitch是否保留
old_pitch = mgr.camera.pitch
mgr.update_camera_params(height=1.5, fx=1000, fy=1000, cx=640, cy=480)
new_pitch = mgr.camera.pitch
print(f"\n6. {'✅' if old_pitch == new_pitch else '❌'} Pitch保持:")
print(f"   更新参数前: {old_pitch:.4f}")
print(f"   更新参数后: {new_pitch:.4f}")
print(f"   {'Pitch保留成功！' if old_pitch == new_pitch else 'Pitch丢失！'}")

# 总结
print("\n" + "=" * 60)
print("测试总结:")
print("=" * 60)
issues = []
if bev1 == bev2:
    issues.append("❌ 雷达参数改变未影响BEV投影")
if not image_bev:
    issues.append("❌ 像素无法投影到BEV")
if old_pitch != new_pitch:
    issues.append("❌ Pitch在参数更新后丢失")

if issues:
    print("\n存在问题：")
    for issue in issues:
        print(f"  {issue}")
else:
    print("\n✅✅✅ 所有核心功能正常！")
