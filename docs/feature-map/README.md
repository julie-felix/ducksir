# Feature map：教 agent 认路

这是给 **agent** 看的路标，不是给人看的产品文档。每个功能一页，回答四件事：

1. **入口**：从哪进（URL / 按钮 id / 快捷键）；
2. **怎么用**：输入是什么、产出是什么、状态挂在哪；
3. **怎么驱动**：用哪条命令/脚本能自动化（探针、curl、eval 表达式）；
4. **已知坑**：踩过的、下次会再踩的。

## 为什么要有这个

没有它的时候，agent（包括我自己）会在「这个功能在哪、怎么点进去」上花掉大量时间，
而且**不会承认自己不知道**——它会瞎猜，然后拿一个错误结论往下做。

这一层的价值不在「记录」，而在**把 UI 变成机器可用的语义**：
控件 id、`__probe` 字段、可断言的数字、驱动脚本名。

## 现有页面

| 页面 | 文件 | 驱动方式 |
|---|---|---|
| 宿主直播 | [host-live.md](host-live.md) | `tools/dev/probe.mjs` + `host-live.json` |
| 浏览器跑 bundle | [play.md](play.md) | `probes/play-bundle.json` |
| 工作台（工程仿真） | [studio.md](studio.md) | `probes/studio.json` |
| botlab 模板一览 | [botlab-templates.md](botlab-templates.md) | 查表用 |

## 写新页的模板

```markdown
# <页面名>

- 入口：`/ducksir-static/client/viewer/<x>.html?<参数>`
- 起服务：`python3 sim/host/serve.py --port 8899 --no-secret`
- 探针：`node tools/dev/probe.mjs --base http://127.0.0.1:8899 --probes tools/dev/probes/<x>.json`

## 控件
| id | 作用 | 备注 |

## `window.__probe.state()`
| 字段 | 含义 | 断言例子 |

## 已知坑
- …
```
