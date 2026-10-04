# LeatherCAD V0.19 — ChatGPT Work 交接说明

## 1. 任务目标
将本目录中的 **Python/OpenCV 原版 LeatherCAD V0.19** 上传到 GitHub 仓库 `yulaomao/leathercad-demo` 的 `main` 分支，并部署到 Render，最后完成线上功能验证。

**不要使用/恢复 Floot TypeScript 浏览器重写版，不要重新移植算法。** Floot 路线已经放弃，因为浏览器重写算法与已验证 Python 版本存在明显差异。

## 2. 当前技术栈
- 后端：Python + Flask
- 图像/标定：OpenCV contrib / ArUco `DICT_4X4_50`
- 数值/几何：NumPy + SciPy
- CAD/DXF：ezdxf
- 前端：HTML/CSS/JavaScript + SVG CAD 编辑器
- 云部署：Gunicorn + Render

## 3. 当前功能
- 手机照片导入
- 16 个 ArUco 标记检测与平面校正
- 自动分割 + 手动点选增加/删除区域
- 多物理零件自适应分离与点击选择
- 外轮廓/孔洞提取
- LINE / ARC / CIRCLE / SLOT 几何拟合
- manufactured-shape 工程轮廓规整
- CAD SVG 预览
- 控制点拖拽编辑：LINE、ARC、CIRCLE
- 修改前 ghost 几何显示
- 校正实物图 / Mask 透明叠加参考
- DXF 导出
- 撤销/状态保存

## 4. 标定板约定
- 4 张 A4，每张 4 个 ArUco，共 16 个
- OpenCV dictionary：`DICT_4X4_50`
- IDs：1–4、5–8、9–12、13–16
- marker：70×70 mm
- A4：210×297 mm
- 单页 marker 左上角位置：`(10,15) (130,15) (10,212) (130,212)` mm
- 配置文件：`markers_v3_70mm.json`

## 5. 已验证的真实照片基线
本地 Python/OpenCV 版本三张实拍图均可检测 **16/16 ArUco**。

物理零件数量：
- beige：2
- orange：1
- grey：1

V0.18/V0.19 方向的代表性几何基线：
- beige part1：LINE 12 / ARC 10；RMS ≈2.50 mm；P95 ≈4.91 mm；max ≈7.38 mm
- beige part2：LINE 6 / ARC 6；RMS ≈2.24 mm；P95 ≈4.18 mm；max ≈6.59 mm
- orange：LINE 14–15 / ARC 11–12 / SLOT 1；RMS ≈2.69–2.81 mm；P95 ≈5.16–5.41 mm；max ≈7.69–7.89 mm
- grey：LINE 8–9 / ARC 9–11 / CIRCLE 1 / SLOT 1；RMS ≈2.40–2.63 mm；P95 ≈4.46 mm；max ≈7.92 mm

重点：几何质量不能只看单元测试，必须看 `原图 → 校正图 → Mask → raw contour → final CAD overlay` 的视觉一致性。

## 6. V0.18/V0.19 关键算法，不要回退
- 自适应多尺度 erosion 分离意外接触的两个物理零件
- substantial core + 稳定尺度判定
- 距离/最近 seed 方式将原始前景重新分配给各零件
- CAD 阶段 robust boundary filtering
- manufactured corner cleanup
- near-linear chain straightening
- dense contour fidelity / 实体偏差约束
- 保留可信设计圆角，抑制织物、包边、毛刺形成的伪圆弧
- 闭合轮廓 seam-safe 处理

## 7. V0.19 控制点编辑
后端 `geometry_fit.py` 包含 `move_fitted_control(...)` 等逻辑：
- LINE：start/end
- ARC：start/end/center/radius
- CIRCLE：center/radius
- 拖动相邻端点时同步连接，保持闭合拓扑
- ARC 端点修改后重新构造真实圆弧，而不是折线近似

`app.py` 包含：
- `/api/drag_control/<pid>`
- `/api/meta/<pid>`
- `/api/save_fitted/<pid>`

## 8. Render 部署文件已经准备
根目录已有：
- `render.yaml`
- `Procfile`
- `requirements-cloud.txt`
- `.gitignore`
- `workspace/.gitkeep`

部署前请检查：
1. Render 使用 Python runtime；
2. 安装 `requirements-cloud.txt`；
3. Gunicorn 启动 Flask `app.py`；
4. 工作目录使用可写临时目录，避免依赖持久本地磁盘；
5. `/healthz` 返回正常；
6. 上传图片、自动处理、CAD 编辑、DXF 下载完整跑通。

## 9. GitHub 状态与已知问题
目标仓库：`yulaomao/leathercad-demo`。

普通 ChatGPT GitHub Connector 已测试：
- 可读取仓库元数据；
- `create_file` 返回 403 `Resource not accessible by integration`；
- 更底层 `create_blob` 同样返回 403。

因此不要继续尝试用当前只读 Connector 绕过。Work 应使用其浏览器/计算机/Git 能力完成仓库上传，或使用具有 GitHub 写权限的 Codex 工作流。

## 10. Work 应执行的任务
1. 检查本交接包代码结构和依赖，不改变核心算法。
2. 本地运行测试；修复仅限部署兼容性问题。
3. 将整个项目提交到 `yulaomao/leathercad-demo` 的 `main`。
4. 在 Render 创建/连接 Web Service。
5. 部署并等待 build/start 成功。
6. 打开线上 URL 验证首页和 `/healthz`。
7. 如测试照片可用，至少完成一次上传→处理→CAD→DXF 的端到端验证。
8. 最终报告 Git commit、Render URL、构建状态、验证结果及任何剩余限制。

## 11. 验收原则
- 不以“能打开网页”为完成标准。
- 不接受将 Python 算法替换成浏览器 TypeScript 近似实现。
- 不因部署方便而删除 SciPy/OpenCV/ezdxf 核心能力。
- 对算法质量的结论必须区分：自动测试、数值回归、人工视觉检查。

## Fixed real-photo regression images (included)
The handoff package now contains `test_images/` with the three original real photos used during V0.18/V0.19 development. Before and after deployment, run the real-photo regression and visually inspect the intermediate/final outputs. Expected results and Python baselines are in `test_images/EXPECTED_RESULTS.md`. Do not substitute the Floot/TypeScript port or synthetic-only tests for these images.
