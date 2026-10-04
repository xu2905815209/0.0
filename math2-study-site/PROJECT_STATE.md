# PROJECT_STATE.md

## Project
考研数学二「错疑学习站」

## Canonical site
- Entry: index.html
- AI rules: CLAUDE.md
- Source: GitHub repo `xu2905815209/0.0`, directory `math2-study-site/`
- Source branch: `feature/0.1`
- Deployment target: GitHub Pages public HTTPS website
- Canonical public URL: **https://xu2905815209.github.io/0.0/**

## Numbering
- Latest official question: **Q001**
- Next official question: **Q002**

## Q001
**两条红色公式从哪里来？什么时候用？**
1. 拉格朗日中值定理：`f(b)-f(a)=f'(ξ)(b-a)`
2. 牛顿—莱布尼茨：`f(b)-f(a)=∫_a^b f'(t)dt`
3. 给 `f'(x)` 图像、面积、一个 `f` 值时优先用牛顿—莱布尼茨
4. 定积分是带符号面积，轴下方为负

## Interaction
搜索、只看未掌握、标记掌握、掌握率、随机复习、随机抽查均已存在。

## Next task rule
下一个新的独立疑问原则上编号 **Q002**；若只是 Q001 追问，优先补充 Q001。

## Public deployment status
- GitHub Pages is enabled with **GitHub Actions** as the source.
- Workflow `Deploy Math2 Study Site` completed successfully.
- Deployment logs reported the environment URL:
  - **https://xu2905815209.github.io/0.0/**
- This URL is now the canonical public site.
- Vercel is not required for the current production site.

## Required next deployment action
- None for initial launch.
- For future content updates: update the same GitHub project, let the Pages workflow deploy, and verify the workflow succeeds.