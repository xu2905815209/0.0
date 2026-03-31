# 奥凯杯项目说明（队友协作版）

## 1. 当前项目状态

- 当前日期：2026-03-31
- 项目仓库：`https://github.com/xu2905815209/0.0`
- 稳定基线版本：`v0.0.0`
- 主分支：`main`（用于保存稳定版本）
- 开发分支：`feature/0.1`（当前迭代分支）

说明：
- `v0.0.0` 是初始工程快照，可随时回退。
- 当前 0.1 功能开发统一在 `feature/0.1` 进行。

## 2. 目录说明（重点）

- `Core/`：核心业务代码（主要改这里）
- `Drivers/`：HAL/CMSIS 驱动库
- `MDK-ARM/`：Keil 工程相关文件
- `EWARM/`：IAR 工程相关文件
- `test2.ioc`：CubeMX 工程配置

## 3. 队友如何快速同步最新代码

### 第一次拉取

```bash
git clone https://github.com/xu2905815209/0.0.git
cd 0.0
git switch feature/0.1
```

### 每次开始开发前（先同步最新）

```bash
git switch feature/0.1
git pull
```

## 4. 开发提交规范（简化版）

每次改完建议按下面流程：

```bash
git add .
git commit -m "feat: 本次修改内容"
git push
```

提交信息建议：
- `feat:` 新功能
- `fix:` 修复问题
- `refactor:` 重构，不改功能
- `docs:` 文档更新

## 5. 版本回退方法

如果开发出现问题，可以随时回到 0.0 基线：

```bash
git switch --detach v0.0.0
```

如果只是放弃本地未提交改动：

```bash
git restore .
```

## 6. 当前协作约定

- 不要直接改 `main`，开发统一在 `feature/0.1`。
- 每次 push 前先 `git pull`，减少冲突。
- 优先修改 `Core/` 下的业务文件，驱动库尽量不改。
- 完成阶段目标后再合并回 `main` 并打 `v0.1.0` 标签。

## 7. 本周开发进度（模板）

请每次提交后更新这一节，方便队友快速了解当前状态。

### 本周目标

- [ ] 目标 1：
- [ ] 目标 2：
- [ ] 目标 3：

### 进行中

- 负责人：
- 分支：`feature/0.1`
- 当前任务：
- 当前阻塞：无 / （写具体问题）

### 最新提交记录（手动维护）

- 日期：YYYY-MM-DD
- 提交人：
- 提交说明：
- 提交哈希：
- 影响文件：

### 下一个计划

- [ ] 下一步 1
- [ ] 下一步 2

### 示例（可删除）

- 日期：2026-03-31
- 提交人：lin mo
- 提交说明：`docs: add teammate-oriented project README`
- 提交哈希：`327adc1`
- 影响文件：`README.md`
