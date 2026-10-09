# 数二精华课 — 考研数学二「题目 + 考点」精选学习站

公网地址：**https://xu2905815209.github.io/0.0/**

## 为什么做这个站
不是记录每一次提问，而是从提问里**筛选最值得练习的代表题与最核心的考点**。
每个单元都有：**完整题干 → 核心考点 → 第一眼怎么想 → 标准解法 → 易错点 → 交互自测**。题目与考点双向对应，但不自动收录所有聊天问题。

## 目录结构
- `index.html`：首页、目录、学习进度与快速复习
- `content/catalog.json`：精选题目、题干、标准 MathML 公式与核心考点映射
- `content/k001.html`：导数图像求函数最值的代表题 + 牛顿—莱布尼茨等考点
- `content/k002.html`：有理分式高阶导代表题 + 负幂求导/链式法则等考点
- `assets/site.js`：页面路由、交互与浏览器学习状态
- `assets/styles.css`：响应式页面样式
- `CLAUDE.md`：本工程最高优先级 AI 修改准则
- `PROJECT_STATE.md`：当前公开课程、清理记录与上线要求

## AI 开工顺序
1. 先阅读 `CLAUDE.md`
2. 再阅读 `PROJECT_STATE.md`
3. 检查目录和已有课程是否包含相关考点
4. 判断是否真的值得入库（不自动收录所有用户疑问）；若值得入库，必须同时有**题目与对应考点**
5. 修改对应源文件、验证、部署

## 本地预览
这是静态站点，无需框架安装。例如从 `math2-study-site/` 目录运行：
```sh
python3 -m http.server 8000
```
然后在浏览器打开 `http://localhost:8000/`。

## 部署
默认分支 `feature/0.1` 上更新 `math2-study-site/**` 会触发 GitHub Actions
`Deploy Math2 Study Site`，部署到固定 GitHub Pages 地址。

## 进度
掌握状态暂用 localStorage 保存；不同设备间尚未同步。
