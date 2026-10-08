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
- Latest official question: **Q003**
- Next official question: **Q004**

## Q001
**两条红色公式从哪里来？什么时候用？**
1. 拉格朗日中值定理：`f(b)-f(a)=f'(ξ)(b-a)`
2. 牛顿—莱布尼茨：`f(b)-f(a)=∫_a^b f'(t)dt`
3. 给 `f'(x)` 图像、面积、一个 `f` 值时优先用牛顿—莱布尼茨
4. 定积分是带符号面积，轴下方为负

## Q002
**为什么 1/x 的 n 阶导等于 (-1)^n n!/x^(n+1)？是泰勒展开吗？**
1. 不是由泰勒展开直接得到，最直接来源是把 1/x 写成 x^(-1) 后反复使用幂函数求导。
2. 一般式：`(1/x)^(n)=(-1)^n n!/x^(n+1)`。
3. `1/(1-x)` 的每次链式求导会出现两个负号并抵消，因此 n 阶导为 `n!/(1-x)^(n+1)`。
4. 原题先部分分式：`1/[x(1-x)] = 1/x + 1/(1-x)`，再分别求高阶导。
5. `1/x` 在 x=0 无定义，因此不能做以 0 为中心的麦克劳林展开；用泰勒解释反而绕远。
6. 考试策略：理解后练到接近背；考场记得就直接写，若突然忘记就求前 2～3 阶，10 秒内恢复规律，不应每次都从零慢慢归纳。

## Q003
**这道含连乘与 n 次根号的数列极限，考研数二多少分？**
1. 题目未注明卷面出处；不能将分值判断为某年真题的实际赋分。
2. 若按现行考研数二选择/填空题出题，每题 5 分。
3. 若改成独立解答题，可粗略估计约 10–12 分，但不存在固定分值；作为大题中的一问也合理。
4. 以最后因子为 n+(n-1)=2n-1 解读：L_n=(1/n)\sqrt[n]{n(n+1)...(2n-1)}。
5. 先化为 \sqrt[n]{\prod_{k=0}^{n-1}(1+k/n)}，然后取对数，再把 (1/n) Σ ln(1+k/n) 认成定积分 ∫_0^1 ln(1+x)dx=2ln2-1；最终极限为 4/e。
6. 本题增添交互：黎曼和逼近的 SVG 滑块示意图，以及“第一步怎么想”的自测题。
7. 手写最后因子存在轻微歧义，网页已说明解释前提。

## Interaction
搜索、只看未掌握、标记掌握、掌握率、随机复习、随机抽查均已存在。

## Next task rule
下一个新的独立疑问原则上编号 **Q004**；若是已收录题目的追问，优先补充原题，不另起编号。

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