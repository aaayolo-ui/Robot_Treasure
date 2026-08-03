# Robot_Treasure Git 使用说明

本文面向第一次接触 Git 的项目成员。看完后，你应该能够完成日常代码提交与同步。

## 1. Git 是什么

Git 是一个代码版本管理工具，可以记录每次修改，方便团队成员协作，也能在出现问题时查找历史版本。

## 2. Git 在本项目中的作用

|作用|说明|
|-|-|
|记录修改|保存 Robot_Treasure 项目每次有意义的代码变化|
|团队同步|把自己的代码分享给队友，并获取队友的最新代码|
|查看历史|了解某个功能何时修改、修改了什么|
|减少丢失|代码上传到 GitHub 后，不只保存在个人电脑中|

## 3. 本地 Git 仓库与 GitHub

|位置|作用|
|-|-|
|本地 Git 仓库|位于自己的电脑中，用于修改、检查和提交代码|
|GitHub 远程仓库|位于网络上，用于保存和共享团队代码|

两者不会自动保持一致，需要手动同步：

本地提交 → `git push` → GitHub

GitHub 最新代码 → `git pull` → 本地仓库

> `git commit` 只保存到本地仓库；执行 `git push` 后，提交才会上传到 GitHub。

## 4. 日常开发流程

开始开发前，建议先获取团队最新代码：

`git pull`

然后按照以下顺序操作：

修改代码

↓

`git status` 查看修改

↓

`git add 文件名` 添加指定文件，或使用 `git add .` 添加当前目录中的全部修改

↓

`git status` 再次确认准备提交的内容

↓

`git commit -m "提交说明"` 保存本地版本

↓

`git push` 上传到 GitHub

提交说明应简短、明确，例如：

```bash
git commit -m "完成电机驱动"
```

## 5. 常用 Git 命令

请在 Robot_Treasure 项目根目录打开终端并执行命令。

|命令|作用|完整示例|
|-|-|-|
|`git status`|查看文件修改和暂存状态|`git status`|
|`git add 文件名`|把指定文件添加到暂存区|`git add Core/Src/main.c`|
|`git add .`|把当前目录中的全部修改添加到暂存区|`git add .`|
|`git commit -m "提交说明"`|把暂存区内容保存为本地版本|`git commit -m "完成电机驱动"`|
|`git push`|把本地提交上传到 GitHub|`git push`|
|`git pull`|从 GitHub 获取并合并最新代码|`git pull`|
|`git log --oneline`|简要查看提交历史|`git log --oneline`|

> 使用 `git add .` 前要先执行 `git status`，确认没有把无关修改加入提交。

## 6. 使用 VS Code 图形化操作

不熟悉终端时，可以使用 VS Code 左侧的“源代码管理”功能：

修改文件

↓

打开“源代码管理”

↓

检查更改，并暂存需要提交的文件

↓

填写提交信息

↓

点击“提交”

↓

点击“同步更改”

“提交”相当于保存到本地 Git 仓库；“同步更改”会与 GitHub 同步。在点击前应确认网络正常，并留意 VS Code 的提示。

## 7. 团队协作

团队代码同步的基本过程：

队友修改代码

↓

队友执行 `git push` 上传到 GitHub

↓

其他成员执行 `git pull` 获取最新版本

建议每天开始开发前执行一次：

```bash
git pull
```

如果 `git pull` 提示冲突，不要随意删除代码。先停止提交，和修改同一文件的队友一起确认应该保留哪些内容。

## 8. 注意事项

不要提交以下文件：

|文件类型|示例或说明|
|-|-|
|Keil 编译生成文件|`Objects/`、`Listings/`、`Output/`、`Debug/`|
|HEX 固件文件|`*.hex`|
|AXF 调试文件|`*.axf`|
|临时或备份文件|`*.tmp`、`*.bak`|

这些文件已经通过项目根目录的 `.gitignore` 自动过滤，通常不会被 Git 添加。

每次提交前仍应执行：

```bash
git status
```

确认提交内容只包含本次需要共享的代码和文档。

多人协作、功能分支和 Pull Request 操作，请阅读：[07_团队协作与Git分支说明.md](07_团队协作与Git分支说明.md)。
