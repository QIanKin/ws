# ws

个人工作区：多个项目放在同一仓库中，便于在不同设备上 `git clone` 后完整拉取（未使用 Git 子模块）。

## 目录

| 目录 | 说明 |
|------|------|
| `DSP-map` | DSP 地图相关 ROS 包 |
| `dynus` | Dynus 项目 |
| `MOCHA` | MOCHA 项目 |
| `RAST_corridor_planning` | 走廊规划（`include/dsp_map` 已内联拷贝自 `DSP-map`，不再依赖子模块） |
| `docs` | 文档 |

## 克隆

```bash
git clone https://github.com/QIanKin/ws.git
cd ws
```
