# GameOS Arcade

一个中文 macOS 风格的网页/桌面界面，内置 50 款游戏入口。界面包含顶部菜单栏、玻璃拟态窗口、完整 Dock 分类筛选、游戏搜索、游戏详情弹窗，以及可直接游玩的小游戏窗口。

每个游戏都会按类型进入对应的轻量玩法：

- 动作 / 冒险：移动接取目标，避开炸弹。
- 竞速：三车道躲障碍、吃星星冲线。
- 益智 / 策略 / 创意：翻牌配对。
- 休闲 / 模拟 / 音乐：限时点击目标。

浏览器会使用 `localStorage` 记录每款游戏的最高分、最近得分和通关状态。

## Getting Started

```bash
node index.js
```

打开浏览器访问：

```text
http://localhost:3000
```

也可以通过 `PORT` 环境变量指定端口：

```bash
PORT=8080 node index.js
```

## Validation

```bash
node scripts/validate.js
```

校验内容包括：

- `index.js`、`game-data.js`、`game-modes.js`、`app.js` 语法检查
- 游戏数量必须保持 50 款
- Dock 分类必须覆盖所有游戏类型
- `index.html` 必须按顺序加载数据、玩法和主应用脚本

也可以单独运行：

```bash
node --check index.js
node --check app.js
```

## Project Structure

```text
index.html        页面结构和脚本加载顺序
style.css         macOS 玻璃拟态界面、游戏窗口、移动端样式
game-data.js      50 款游戏数据、渐变色和分类
game-modes.js     可复用小游戏玩法
app.js            搜索、分类、详情、成绩和游戏启动逻辑
index.js          零依赖本地静态服务器
scripts/validate.js 轻量质量检查脚本
```

## Adding a Game

在 `game-data.js` 的 `gameData` 中追加一项：

```text
[名称, 类型, 简介, 图标]
```

注意事项：

- 类型会自动出现在 Dock 中。
- 游戏总数当前设计目标是 50 款；修改数量后需要同步调整说明和校验规则。
- 新类型默认使用限时点击玩法；如果需要独立规则，可在 `game-modes.js` 的 `getMode` 中映射。

## License

MIT
