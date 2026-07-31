# Task 4 报告：构建集成（sources.mk / subdir 文件 / makefile）

## 状态：DONE_WITH_CONCERNS

## 改动清单

| 文件 | 操作 | 说明 |
|---|---|---|
| `Debug/sources.mk` | 修改 | SUBDIRS 在 `ucdriver/uart \` 后追加 `ucdriver/ball \` 与 `ucdriver/stepper \`（续行符完整，L129-130） |
| `Debug/ucdriver/ball/subdir_vars.mk` | 新建 | 登记 `../ucdriver/ball/ball.c` → `ball.o`/`ball.d`，格式对齐 uart 模板 |
| `Debug/ucdriver/ball/subdir_rules.mk` | 新建 | 编译规则，-I 列表追加 `ucdriver/ball`、`ucdriver/stepper` 两个路径，其余照抄 uart |
| `Debug/ucdriver/stepper/subdir_vars.mk` | 新建 | 登记 `../ucdriver/stepper/stepper.c` → `stepper.o`/`stepper.d` |
| `Debug/ucdriver/stepper/subdir_rules.mk` | 新建 | 编译规则，-I 列表追加 `ucdriver/ball`、`ucdriver/stepper` |
| `Debug/makefile` | 修改 | ① ORDERED_OBJS 追加 `"./ucdriver/ball/ball.o"`、`"./ucdriver/stepper/stepper.o"`（L25-26）；② -include 追加 2 条 subdir_vars.mk（L46-47）；③ -include 追加 2 条 subdir_rules.mk（L56-57）；④ clean 两行末尾追加 `ball.o`/`stepper.o` 与 `ball.d`/`stepper.d`（L168-169） |

## 自审结果（Step 7）

- [x] `Debug/ucdriver/ball/`、`Debug/ucdriver/stepper/` 目录已创建（写入文件前 `New-Item -Force` 确认存在）
- [x] sources.mk SUBDIRS 续行符完整（末行为 `ucdriver/stepper \`，无多余空行破坏续行）
- [x] makefile 三处均含对应条目：ORDERED_OBJS 2 行、-include 4 条（2 vars + 2 rules）、clean 4 个目标（2 .o + 2 .d）——经 grep 全量核对
- [x] 两个 rules.mk 的 -I 列表均含 `ucdriver/ball` 与 `ucdriver/stepper` 两个新路径
- [x] vars 文件路径与源文件匹配：`ucdriver/ball/ball.c`、`ucdriver/stepper/stepper.c` 均存在（glob 确认）
- [x] 格式与 `Debug/ucdriver/uart/subdir_vars.mk`、`subdir_rules.mk` 模板一致（SHELL/`%` 模式规则/`d_raw` 中间文件/`-MT"$(@)"` 等）

## Commit

`b12c7f2` — `build: 注册 ball/stepper 模块到 CCS 构建系统`（6 files changed, 380 insertions）

- 仅提交 Debug/ 相关 6 个文件；未包含 `.superpowers/`、`docs/`、`empty.syscfg` 等其他工作区改动

## Concerns

1. **`.gitignore` 冲突**：仓库 `.gitignore:1` 为 `Debug/`，且 git 历史中从未追踪过 Debug/ 下任何文件（此前任务仅提交 `ucdriver/` 源码）。本次按 brief Step 8 要求使用 `git add -f` 强制添加并提交。影响：此后每次手动编辑 Debug/ 构建文件（本项目既定流程）都会产生 diff，版本控制行为与既有惯例不同。若不想长期追踪，可后续用 `git rm --cached` 撤出索引（不影响工作区），或修改 `.gitignore` 白名单。
2. **未做实际构建验证**：环境无法脱离 CCS 构建（AGENTS.md 已注明），未运行 make；语法层面已人工核对该类 makefile 的惯用格式（逐字对照 uart 模板）。
