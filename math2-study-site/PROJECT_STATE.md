# PROJECT_STATE.md

## Project
考研数学二「错疑学习站」

## Canonical site
- Entry: index.html
- AI rules: CLAUDE.md
- Source: GitHub repo `xu2905815209/0.0`, directory `math2-study-site/`
- Source branch: `feature/0.1`
- Deployment target: public HTTPS website

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
- Source has been pushed to GitHub.
- GitHub Pages workflow exists at `.github/workflows/math2-pages.yml`.
- GitHub Pages is now enabled (`has_pages: true`) with GitHub Actions as the source.
- Vercel connection is authenticated, but project creation currently returns HTTP 403 requiring authorization to the personal Vercel scope `aed29566-3294`.
- No production URL may be treated as canonical until a deployment is successfully verified.

## Required next deployment action
- Trigger the existing GitHub Pages workflow via this commit and verify the resulting public URL.
- If deployment succeeds, record the verified URL here as the canonical public site.
