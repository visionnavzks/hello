# GameOS Arcade

一个中文 macOS 风格的网页/桌面界面，内置 50 款游戏入口。界面包含顶部菜单栏、玻璃拟态窗口、完整 Dock 分类筛选、游戏搜索、游戏详情弹窗，以及可直接游玩的小游戏窗口。

50 个游戏都带有独立的可玩规则：目标物、危险物、胜利条件、生命值/时间限制都会写在游戏详情和开局提示中。每个游戏都会按类型进入对应玩法：

- 动作：左右移动，接住目标、躲开危险物。
- 冒险：方向键 / WASD 探索 5x5 地图，收集线索并避开陷阱。
- 竞速：三车道换道，收集补给并躲避障碍冲线。
- 益智：翻牌配对，完成全部线索组合。
- 策略：点击拦截来袭敌人，守住核心。
- 模拟：生产、补给、升级，完成经营订单。
- 音乐：按下或点击当前节拍键，避免漏拍。
- 创意：点击画布填色，避开褪色格。
- 休闲：限时点击目标，避开危险目标并保持连击。

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
- 每个游戏必须包含可玩规则，并且规则必须映射到受支持的玩法模式
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
game-data.js      50 款游戏数据、每款游戏的可玩规则、渐变色和分类
game-modes.js     动作/冒险/竞速/益智/策略/模拟/音乐/创意/休闲玩法引擎
app.js            搜索、分类、详情、成绩和游戏启动逻辑
index.js          零依赖本地静态服务器
scripts/validate.js 轻量质量检查脚本
```

## Adding a Game

在 `game-data.js` 的 `baseGames` 中追加一项：

```text
[名称, 类型, 简介, 图标, 目标物, 危险物, 胜利目标]
```

注意事项：

- 类型会自动出现在 Dock 中。
- 游戏总数当前设计目标是 50 款；修改数量后需要同步调整说明和校验规则。
- 类型会通过 `modeByType` 映射到玩法；新类型默认需要补充映射，否则校验会失败。
- 每款游戏必须有目标物、危险物和胜利目标，避免只新增图标入口。

## License

MIT
