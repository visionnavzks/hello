const games = [
  ['星际冲刺', '动作', '驾驶飞船穿越流星雨，挑战极限反应。', '🚀'],
  ['像素迷宫', '益智', '旋转房间、寻找钥匙，逃出复古迷宫。', '🧩'],
  ['霓虹赛车', '竞速', '在赛博城市霓虹赛道上漂移冲线。', '🏎️'],
  ['海岛探险', '冒险', '探索神秘岛屿，收集宝藏与线索。', '🏝️'],
  ['水果连连', '休闲', '连接相同水果，创造爽快连击。', '🍓'],
  ['方块塔防', '策略', '布置炮塔，守住你的水晶核心。', '🛡️'],
  ['泡泡射手', '休闲', '瞄准彩色泡泡，清空整个天空。', '🫧'],
  ['忍者跳跃', '动作', '墙跳、闪避飞镖，抵达屋顶终点。', '🥷'],
  ['森林拼图', '益智', '拼合森林碎片，唤醒沉睡的精灵。', '🌲'],
  ['极速雪橇', '竞速', '穿越雪山弯道，躲避冰柱障碍。', '🛷'],
  ['月球矿工', '模拟', '经营月球矿场，升级采集机器人。', '🌙'],
  ['魔法棋盘', '策略', '用法术棋子击败对手的王冠。', '♟️'],
  ['深海寻宝', '冒险', '驾驶潜艇寻找失落文明宝箱。', '🤿'],
  ['节奏鼓手', '音乐', '跟随节拍敲击鼓点，点燃舞台。', '🥁'],
  ['猫咪咖啡', '模拟', '经营温馨猫咖，招待不同客人。', '🐱'],
  ['糖果工厂', '休闲', '组合糖果流水线，生产甜蜜订单。', '🍬'],
  ['太空农场', '模拟', '在轨道温室种植外星作物。', '🪴'],
  ['龙之山谷', '冒险', '训练小龙，穿越火焰山谷。', '🐉'],
  ['弹珠大师', '休闲', '发射弹珠，击中机关获取高分。', '🔮'],
  ['机器人大乱斗', '动作', '组装机器人，在竞技场对战。', '🤖'],
  ['滑板街区', '竞速', '完成花式动作，征服城市街区。', '🛹'],
  ['宝石消消', '益智', '交换宝石，触发华丽连锁爆炸。', '💎'],
  ['幽灵旅馆', '冒险', '调查奇怪旅馆里的幽灵谜案。', '👻'],
  ['迷你高尔夫', '休闲', '掌握角度和力度，一杆进洞。', '⛳'],
  ['银河防线', '策略', '指挥舰队阻挡外星入侵。', '🛰️'],
  ['火山跑酷', '动作', '越过岩浆裂缝，逃离喷发火山。', '🌋'],
  ['厨房快手', '模拟', '快速备餐，满足络绎不绝的订单。', '🍳'],
  ['纸牌王国', '策略', '用卡牌建造王国并击退怪物。', '🃏'],
  ['雪人保龄', '休闲', '滚动雪球，击倒可爱的雪人瓶。', '🎳'],
  ['极地钓鱼', '模拟', '寻找最佳冰洞，钓起稀有鱼类。', '🎣'],
  ['沙漠越野', '竞速', '驾驶越野车冲过沙丘和峡谷。', '🚙'],
  ['汉字侦探', '益智', '根据线索拆字解谜，破解案件。', '🔎'],
  ['勇者地牢', '冒险', '深入地牢，收集装备挑战魔王。', '⚔️'],
  ['飞盘狗狗', '休闲', '投掷飞盘，让狗狗完成空中接力。', '🐶'],
  ['云端滑翔', '竞速', '乘滑翔翼穿过云层计时赛道。', '🪂'],
  ['时间旅者', '冒险', '穿梭不同年代，修复错乱时间线。', '⏳'],
  ['数字黑洞', '益智', '合并数字，制造越来越大的黑洞。', '🕳️'],
  ['星球绘师', '创意', '用色彩和地形设计专属星球。', '🎨'],
  ['怪兽牧场', '模拟', '孵化怪兽，训练并参加友谊赛。', '🥚'],
  ['空中花园', '休闲', '布置漂浮花园，让花朵持续盛开。', '🌷'],
  ['激光谜阵', '益智', '调整镜面，让激光点亮全部节点。', '🔦'],
  ['机甲竞速', '竞速', '驾驶机甲穿越未来工业赛道。', '🦾'],
  ['古堡守卫', '策略', '部署弓箭手与骑士保卫古堡。', '🏰'],
  ['蜜蜂快递', '动作', '帮助蜜蜂穿越花园完成配送。', '🐝'],
  ['梦境画廊', '创意', '收集梦境碎片，点亮互动展厅。', '🖼️'],
  ['行星弹球', '休闲', '利用引力弹射小球命中行星。', '🪐'],
  ['雪域救援', '冒险', '驾驶救援车寻找迷路登山者。', '🚑'],
  ['代码骑士', '益智', '排列指令卡，让骑士自动闯关。', '��'],
  ['迷你足球', '动作', '三分钟快节奏足球对决。', '⚽'],
  ['彩虹列车', '模拟', '规划轨道，连接五彩小镇。', '🚂'],
];

const gradients = [
  ['#60a5fa', '#a78bfa'],
  ['#fb7185', '#f97316'],
  ['#34d399', '#06b6d4'],
  ['#facc15', '#f472b6'],
  ['#c084fc', '#22d3ee'],
];

const grid = document.querySelector('#game-grid');
const search = document.querySelector('#search');
const detail = document.querySelector('#game-detail');
const closeDetail = document.querySelector('#close-detail');
const detailTitle = document.querySelector('#detail-title');
const detailIcon = document.querySelector('#detail-icon');
const detailType = document.querySelector('#detail-type');
const detailDescription = document.querySelector('#detail-description');
const playSelected = document.querySelector('#play-selected');
const filterButtons = document.querySelectorAll('.dock button');
const playerWindow = document.querySelector('#player-window');
const closePlayer = document.querySelector('#close-player');
const restartGame = document.querySelector('#restart-game');
const playerTitle = document.querySelector('#player-title');
const playerScore = document.querySelector('#player-score');
const playerTimer = document.querySelector('#player-timer');
const playerInstructions = document.querySelector('#player-instructions');
const gameStage = document.querySelector('#game-stage');
const gameStatus = document.querySelector('#game-status');
let activeFilter = '全部';
let selectedGame = null;
let runningGame = null;

function toGameObject(game, index) {
  const [name, type, description, icon] = game;
  const [c1, c2] = gradients[index % gradients.length];
  return { name, type, description, icon, c1, c2 };
}

function renderGames() {
  const term = search.value.trim().toLowerCase();
  const filteredGames = games
    .map(toGameObject)
    .filter(({ name, type, description }) => {
      const matchesFilter = activeFilter === '全部' || type === activeFilter;
      const matchesSearch = [name, type, description].join(' ').toLowerCase().includes(term);
      return matchesFilter && matchesSearch;
    });

  grid.textContent = '';

  filteredGames.forEach((game) => {
    const button = document.createElement('button');
    const icon = document.createElement('span');
    const title = document.createElement('strong');
    const type = document.createElement('span');

    button.className = 'game-card';
    button.type = 'button';
    button.style.setProperty('--c1', game.c1);
    button.style.setProperty('--c2', game.c2);
    icon.className = 'game-icon';
    icon.setAttribute('aria-hidden', 'true');
    icon.textContent = game.icon;
    title.textContent = game.name;
    type.textContent = game.type;
    button.append(icon, title, type);
    button.addEventListener('click', () => showDetail(game));
    grid.appendChild(button);
  });

  if (filteredGames.length === 0) {
    const empty = document.createElement('p');
    empty.className = 'empty';
    empty.textContent = '没有找到匹配的游戏。';
    grid.appendChild(empty);
  }
}

function showDetail(game) {
  selectedGame = game;
  detail.hidden = false;
  detailTitle.textContent = game.name;
  detailIcon.textContent = game.icon;
  detailIcon.style.setProperty('--c1', game.c1);
  detailIcon.style.setProperty('--c2', game.c2);
  detailType.textContent = `${game.type} 游戏`;
  detailDescription.textContent = `${game.description} 现在可以点击“开始游戏”直接玩。`;
}

function updateClock() {
  document.querySelector('#clock').textContent = new Intl.DateTimeFormat('zh-CN', {
    hour: '2-digit',
    minute: '2-digit',
    weekday: 'short',
  }).format(new Date());
}

function updateHud(score, time) {
  playerScore.textContent = `得分 ${score}`;
  playerTimer.textContent = `时间 ${time}`;
}

function cleanupRunningGame() {
  if (!runningGame) return;
  runningGame.timers.forEach((timer) => clearInterval(timer));
  runningGame.timeouts.forEach((timer) => clearTimeout(timer));
  runningGame.listeners.forEach(([target, event, handler]) => target.removeEventListener(event, handler));
  runningGame = null;
}

function makeGameState(game) {
  cleanupRunningGame();
  gameStage.textContent = '';
  gameStatus.textContent = '';
  runningGame = { game, timers: [], timeouts: [], listeners: [], over: false };
  return runningGame;
}

function addInterval(state, handler, delay) {
  const timer = setInterval(handler, delay);
  state.timers.push(timer);
  return timer;
}

function addTimeout(state, handler, delay) {
  const timer = setTimeout(handler, delay);
  state.timeouts.push(timer);
  return timer;
}

function addListener(state, target, event, handler) {
  target.addEventListener(event, handler);
  state.listeners.push([target, event, handler]);
}

function startCountdown(state, seconds, onEnd) {
  let time = seconds;
  updateHud(0, time);
  addInterval(state, () => {
    time -= 1;
    const score = Number(playerScore.textContent.replace(/\D/g, '')) || 0;
    updateHud(score, time);
    if (time <= 0 && !state.over) {
      onEnd();
    }
  }, 1000);
}

function finishGame(state, message, won) {
  if (state.over) return;
  state.over = true;
  gameStatus.textContent = message;
  gameStatus.style.color = won ? '#bbf7d0' : '#fecaca';
  state.timers.forEach((timer) => clearInterval(timer));
  state.timeouts.forEach((timer) => clearTimeout(timer));
}

function positionElement(element, x, y) {
  element.style.left = `${x}px`;
  element.style.top = `${y}px`;
}

function randomPosition(size = 80) {
  const width = Math.max(gameStage.clientWidth - size, 1);
  const height = Math.max(gameStage.clientHeight - size, 1);
  return [Math.random() * width, Math.random() * height];
}

function startTapGame(game) {
  const state = makeGameState(game);
  let score = 0;
  playerInstructions.textContent = `点击不断出现的 ${game.icon}，30 秒内拿到 15 分就赢。`;
  startCountdown(state, 30, () => finishGame(state, '时间到，再来一局！', false));

  function spawnTarget() {
    if (state.over) return;
    gameStage.textContent = '';
    const target = document.createElement('button');
    target.className = 'tap-target';
    target.type = 'button';
    target.textContent = game.icon;
    target.style.background = `linear-gradient(135deg, ${game.c1}, ${game.c2})`;
    positionElement(target, ...randomPosition(84));
    target.addEventListener('click', () => {
      score += 1;
      updateHud(score, playerTimer.textContent.replace(/\D/g, '') || 0);
      if (score >= 15) {
        finishGame(state, `通关！${game.name} 达成 15 连击。`, true);
      } else {
        spawnTarget();
      }
    });
    gameStage.appendChild(target);
  }

  spawnTarget();
}

function startMemoryGame(game) {
  const state = makeGameState(game);
  const symbols = [game.icon, '⭐', '��', '🎯', '🔥', '🌈'];
  const cards = [...symbols, ...symbols].sort(() => Math.random() - 0.5);
  const gridElement = document.createElement('div');
  let firstCard = null;
  let lock = false;
  let matched = 0;

  playerInstructions.textContent = `翻开卡片并配对相同图案，全部配对即可通关 ${game.name}。`;
  updateHud(0, '不限');
  gridElement.className = 'memory-grid';

  cards.forEach((symbol) => {
    const card = document.createElement('button');
    card.className = 'memory-card';
    card.type = 'button';
    card.dataset.symbol = symbol;
    card.textContent = '?';
    card.addEventListener('click', () => {
      if (lock || card.classList.contains('matched') || card === firstCard) return;
      card.classList.add('revealed');
      card.textContent = symbol;
      if (!firstCard) {
        firstCard = card;
        return;
      }
      if (firstCard.dataset.symbol === symbol) {
        card.classList.add('matched');
        firstCard.classList.add('matched');
        firstCard = null;
        matched += 1;
        updateHud(matched, '不限');
        if (matched === symbols.length) {
          finishGame(state, `通关！${game.name} 全部配对完成。`, true);
        }
        return;
      }
      lock = true;
      addTimeout(state, () => {
        card.classList.remove('revealed');
        firstCard.classList.remove('revealed');
        card.textContent = '?';
        firstCard.textContent = '?';
        firstCard = null;
        lock = false;
      }, 700);
    });
    gridElement.appendChild(card);
  });

  gameStage.appendChild(gridElement);
}

function rectsOverlap(a, b) {
  const ar = a.getBoundingClientRect();
  const br = b.getBoundingClientRect();
  return ar.left < br.right && ar.right > br.left && ar.top < br.bottom && ar.bottom > br.top;
}

function addArcadeControls(state, onLeft, onRight) {
  const controls = document.createElement('div');
  const left = document.createElement('button');
  const right = document.createElement('button');
  controls.className = 'arcade-controls';
  left.type = 'button';
  right.type = 'button';
  left.textContent = '←';
  right.textContent = '→';
  left.addEventListener('click', onLeft);
  right.addEventListener('click', onRight);
  controls.append(left, right);
  gameStage.appendChild(controls);
  addListener(state, document, 'keydown', (event) => {
    if (event.key === 'ArrowLeft' || event.key.toLowerCase() === 'a') onLeft();
    if (event.key === 'ArrowRight' || event.key.toLowerCase() === 'd') onRight();
  });
}

function startRacerGame(game) {
  const state = makeGameState(game);
  let lane = 1;
  let score = 0;
  let ticks = 0;
  const player = document.createElement('div');
  const entities = [];

  playerInstructions.textContent = `用 ←/→ 或 A/D 换道，躲开障碍，吃到 12 个 ${game.icon} 即可通关。`;
  updateHud(0, '冲刺');

  for (let index = 0; index < 3; index += 1) {
    const laneElement = document.createElement('div');
    laneElement.className = 'racer-lane';
    laneElement.style.left = `${index * 33.333}%`;
    gameStage.appendChild(laneElement);
  }

  player.className = 'racer-player';
  player.textContent = game.icon;
  gameStage.appendChild(player);

  function renderPlayer() {
    const width = gameStage.clientWidth;
    const x = (lane + 0.5) * (width / 3) - 22;
    positionElement(player, x, gameStage.clientHeight - 70);
  }

  function moveLeft() {
    lane = Math.max(0, lane - 1);
    renderPlayer();
  }

  function moveRight() {
    lane = Math.min(2, lane + 1);
    renderPlayer();
  }

  function spawnEntity() {
    const entity = document.createElement('div');
    const isCoin = Math.random() > 0.55;
    const entityLane = Math.floor(Math.random() * 3);
    entity.className = isCoin ? 'racer-coin' : 'racer-obstacle';
    entity.textContent = isCoin ? '⭐' : '🚧';
    entity.dataset.kind = isCoin ? 'coin' : 'obstacle';
    entity.dataset.y = '-48';
    entity.dataset.lane = String(entityLane);
    entity.style.left = `${(entityLane + 0.5) * (gameStage.clientWidth / 3) - 22}px`;
    entity.style.top = '-48px';
    entities.push(entity);
    gameStage.appendChild(entity);
  }

  addArcadeControls(state, moveLeft, moveRight);
  renderPlayer();
  addListener(state, window, 'resize', renderPlayer);

  addInterval(state, () => {
    if (state.over) return;
    ticks += 1;
    if (ticks % 8 === 0) spawnEntity();
    entities.slice().forEach((entity) => {
      const nextY = Number(entity.dataset.y) + 12;
      entity.dataset.y = String(nextY);
      entity.style.top = `${nextY}px`;
      if (rectsOverlap(player, entity)) {
        if (entity.dataset.kind === 'coin') {
          score += 1;
          updateHud(score, '冲刺');
          entity.remove();
          entities.splice(entities.indexOf(entity), 1);
          if (score >= 12) finishGame(state, `冲线成功！${game.name} 已通关。`, true);
        } else {
          finishGame(state, '撞到障碍，点击重开再试！', false);
        }
      } else if (nextY > gameStage.clientHeight + 48) {
        entity.remove();
        entities.splice(entities.indexOf(entity), 1);
      }
    });
  }, 80);
}

function startCatcherGame(game) {
  const state = makeGameState(game);
  let x = Math.max(gameStage.clientWidth / 2 - 40, 0);
  let score = 0;
  let lives = 3;
  let ticks = 0;
  const basket = document.createElement('div');
  const items = [];

  playerInstructions.textContent = `用 ←/→ 或 A/D 移动，接住 12 个 ${game.icon}，漏掉 3 个就失败。`;
  updateHud(0, `生命 ${lives}`);
  basket.className = 'catcher-basket';
  basket.textContent = '🧺';
  gameStage.appendChild(basket);

  function renderBasket() {
    positionElement(basket, x, gameStage.clientHeight - 62);
  }

  function move(delta) {
    x = Math.max(0, Math.min(gameStage.clientWidth - 80, x + delta));
    renderBasket();
  }

  function spawnItem() {
    const item = document.createElement('div');
    item.className = 'falling-item';
    item.textContent = Math.random() > 0.22 ? game.icon : '💣';
    item.dataset.kind = item.textContent === '💣' ? 'bad' : 'good';
    item.dataset.y = '-44';
    item.style.left = `${Math.random() * Math.max(gameStage.clientWidth - 44, 1)}px`;
    item.style.top = '-44px';
    items.push(item);
    gameStage.appendChild(item);
  }

  addArcadeControls(state, () => move(-34), () => move(34));
  renderBasket();
  addListener(state, window, 'resize', renderBasket);

  addInterval(state, () => {
    if (state.over) return;
    ticks += 1;
    if (ticks % 9 === 0) spawnItem();
    items.slice().forEach((item) => {
      const nextY = Number(item.dataset.y) + 9;
      item.dataset.y = String(nextY);
      item.style.top = `${nextY}px`;
      if (rectsOverlap(basket, item)) {
        if (item.dataset.kind === 'bad') {
          finishGame(state, '接到炸弹了，重开再来！', false);
        } else {
          score += 1;
          updateHud(score, `生命 ${lives}`);
          if (score >= 12) finishGame(state, `通关！${game.name} 收集完成。`, true);
        }
        item.remove();
        items.splice(items.indexOf(item), 1);
      } else if (nextY > gameStage.clientHeight + 48) {
        if (item.dataset.kind === 'good') {
          lives -= 1;
          updateHud(score, `生命 ${lives}`);
          if (lives <= 0) finishGame(state, '漏掉太多目标，点击重开再试。', false);
        }
        item.remove();
        items.splice(items.indexOf(item), 1);
      }
    });
  }, 80);
}

function getMode(game) {
  if (game.type === '竞速') return startRacerGame;
  if (['益智', '策略', '创意'].includes(game.type)) return startMemoryGame;
  if (['动作', '冒险'].includes(game.type)) return startCatcherGame;
  return startTapGame;
}

function startGame(game = selectedGame || toGameObject(games[0], 0)) {
  selectedGame = game;
  playerWindow.hidden = false;
  playerTitle.textContent = game.name;
  getMode(game)(game);
  gameStage.focus();
}

search.addEventListener('input', renderGames);
closeDetail.addEventListener('click', () => {
  detail.hidden = true;
});
playSelected.addEventListener('click', () => startGame());
closePlayer.addEventListener('click', () => {
  cleanupRunningGame();
  playerWindow.hidden = true;
});
restartGame.addEventListener('click', () => startGame());

filterButtons.forEach((button) => {
  button.addEventListener('click', () => {
    activeFilter = button.dataset.filter;
    filterButtons.forEach((item) => item.classList.toggle('active', item === button));
    renderGames();
  });
});

filterButtons[0].classList.add('active');
renderGames();
updateClock();
setInterval(updateClock, 30000);
