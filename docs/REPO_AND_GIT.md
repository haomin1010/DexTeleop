# 仓库与 Git

## 推荐结构

- `DexProj/`：主仓库
- `wuji-hand-teleop/`：submodule
- `wuji-retargeting/`：submodule
- `TJ/`：历史仓库，后续删除

## 初始化

```bash
git init
```

## 子模块

```bash
git submodule add <wuji-hand-teleop-repo-url> wuji-hand-teleop
git submodule add <wuji-retargeting-repo-url> wuji-retargeting
```

## 忽略规则

- 忽略运行产物 `data/`
- 忽略 Python cache
- 忽略 ROS2 build/install/log
- 保留 `config/`、`docs/`、`scripts/`、`dexproj/`

## 建议

- 主仓库只放 DexProj 编排层
- 子模块尽量保持上游原样
- 只有通用修补才改子模块
