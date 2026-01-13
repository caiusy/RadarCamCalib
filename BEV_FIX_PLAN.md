# BEV系统修复计划

## 问题
1. ❌ 像素点颜色不对（只显示洋红色）
2. ❌ 点pitch后BEV点重复
3. ❌ 修改radar参数后BEV不更新
4. ❌ 缺少鼠标边界检查

## 修复方案

### 1. 颜色问题
- 检查`_onImageClicked`中的BEV投影逻辑
- 确保调用`addComparisonPair`时传递了正确参数

### 2. 点重复问题
- `_refreshBEV()` 应该先清空再重建
- `_onComputePitch()` 不应该重复添加雷达点

### 3. 参数更新问题  
- `_onParamChanged()` 需要：
  1. 清空当前BEV雷达点
  2. 用新参数重新投影所有雷达点
  3. 重新投影已有的点对

### 4. 边界检查
- 在`_onImageClicked`中添加：
  ```python
  if not (0 <= u <= image_width and 0 <= v <= image_height):
      return
  ```
