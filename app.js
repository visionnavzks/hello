const { games, gradients, categories } = window.GameOSData;
const gameModes = window.createGameModes({
  gameStage: document.querySelector('#game-stage'),
  playerInstructions: document.querySelector('#player-instructions'),
  updateHud,
  makeGameState,
  addInterval,
  addTimeout,
  addListener,
  startCountdown,
  finishGame,
  positionElement,
  randomPosition,
});

const SCORE_KEY = 'gameos-arcade-results';
const grid = document.querySelector('#game-grid');
const search = document.querySelector('#search');
const detail = document.querySelector('#game-detail');
const closeDetail = document.querySelector('#close-detail');
const detailTitle = document.querySelector('#detail-title');
const detailIcon = document.querySelector('#detail-icon');
const detailType = document.querySelector('#detail-type');
const detailDescription = document.querySelector('#detail-description');
const detailRecord = document.querySelector('#detail-record');
const playSelected = document.querySelector('#play-selected');
const dock = document.querySelector('#category-dock');
const playerWindow = document.querySelector('#player-window');
const closePlayer = document.querySelector('#close-player');
const restartGame = document.querySelector('#restart-game');
const backToLibrary = document.querySelector('#back-to-library');
const playerTitle = document.querySelector('#player-title');
const playerScore = document.querySelector('#player-score');
const playerTimer = document.querySelector('#player-timer');
const playerBest = document.querySelector('#player-best');
const playerInstructions = document.querySelector('#player-instructions');
const gameStage = document.querySelector('#game-stage');
const gameStatus = document.querySelector('#game-status');
const modeLabels = {
  dodge: '动作躲避',
  adventure: '探索收集',
  racer: '竞速冲线',
  puzzle: '益智配对',
  defense: '策略防守',
  simulation: '经营模拟',
  rhythm: '节奏敲击',
  creative: '创意填色',
  casual: '休闲挑战',
};
let activeFilter = '全部';
let selectedGame = null;
let runningGame = null;

function loadResults() {
  try {
    return JSON.parse(localStorage.getItem(SCORE_KEY)) || {};
  } catch (error) {
    return {};
  }
}

function saveResults(results) {
  try {
    localStorage.setItem(SCORE_KEY, JSON.stringify(results));
  } catch (error) {
    gameStatus.textContent = '本地成绩暂时无法保存，但游戏仍可继续。';
  }
}

function gameKey(game) {
  return `${game.type}:${game.name}`;
}

function getResult(game) {
  return loadResults()[gameKey(game)] || null;
}

function formatResult(result) {
  if (!result) return '暂无成绩';
  const outcome = result.won ? '已通关' : '未通关';
  return `${outcome} · 最高 ${result.bestScore} · 最近 ${result.lastScore}`;
}

function recordResult(game, score, won) {
  const results = loadResults();
  const key = gameKey(game);
  const previous = results[key] || { bestScore: 0, lastScore: 0, plays: 0, won: false };
  results[key] = {
    bestScore: Math.max(previous.bestScore || 0, score),
    lastScore: score,
    plays: (previous.plays || 0) + 1,
    won: Boolean(previous.won || won),
    updatedAt: new Date().toISOString(),
  };
  saveResults(results);
  return results[key];
}

function toGameObject(game, index) {
  if (!Array.isArray(game)) {
    const [c1, c2] = gradients[index % gradients.length];
    return { ...game, c1, c2 };
  }

  const [name, type, description, icon] = game;
  const [c1, c2] = gradients[index % gradients.length];
  return { name, type, description, icon, c1, c2 };
}

function getModeLabel(game) {
  return modeLabels[game.rules.mode] || '可玩挑战';
}

function formatObjective(game) {
  return `玩法：${getModeLabel(game)} · 目标：${game.rules.objective} · 收集：${game.rules.target} · 避开：${game.rules.hazard}`;
}

function renderCategories() {
  dock.textContent = '';
  categories.forEach((category) => {
    const button = document.createElement('button');
    button.type = 'button';
    button.dataset.filter = category;
    button.textContent = category;
    button.classList.toggle('active', category === activeFilter);
    button.addEventListener('click', () => {
      activeFilter = category;
      renderCategories();
      renderGames();
    });
    dock.appendChild(button);
  });
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

  document.querySelector('#game-count').textContent = `${games.length} 款游戏`;
  grid.textContent = '';

  filteredGames.forEach((game) => {
    const button = document.createElement('button');
    const icon = document.createElement('span');
    const title = document.createElement('strong');
    const type = document.createElement('span');
    const record = document.createElement('small');

    button.className = 'game-card';
    button.type = 'button';
    button.style.setProperty('--c1', game.c1);
    button.style.setProperty('--c2', game.c2);
    icon.className = 'game-icon';
    icon.setAttribute('aria-hidden', 'true');
    icon.textContent = game.icon;
    title.textContent = game.name;
    type.textContent = game.type;
    record.textContent = `${getModeLabel(game)} · ${formatResult(getResult(game))}`;
    button.append(icon, title, type, record);
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
  detailDescription.textContent = `${game.description} ${formatObjective(game)}。点击“开始游戏”即可操作。`;
  detailRecord.textContent = `成绩：${formatResult(getResult(game))}`;
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
  playerTimer.textContent = `目标 ${time}`;
}

function updatePlayerRecord(game) {
  playerBest.textContent = `成绩 ${formatResult(getResult(game))}`;
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
  gameStatus.style.color = '';
  runningGame = { game, timers: [], timeouts: [], listeners: [], over: false, score: 0, time: 0 };
  updatePlayerRecord(game);
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
  state.time = seconds;
  updateHud(state.score, state.time);
  addInterval(state, () => {
    state.time -= 1;
    updateHud(state.score, state.time);
    if (state.time <= 0 && !state.over) {
      onEnd();
    }
  }, 1000);
}

function finishGame(state, message, won) {
  if (state.over) return;
  state.over = true;
  const result = recordResult(state.game, state.score, won);
  gameStatus.textContent = `${message} ${formatResult(result)}`;
  gameStatus.style.color = won ? '#bbf7d0' : '#fecaca';
  state.timers.forEach((timer) => clearInterval(timer));
  state.timeouts.forEach((timer) => clearTimeout(timer));
  updatePlayerRecord(state.game);
  renderGames();
  if (selectedGame && selectedGame.name === state.game.name) {
    detailRecord.textContent = `成绩：${formatResult(result)}`;
  }
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

function startGame(game = selectedGame || toGameObject(games[0], 0)) {
  selectedGame = game;
  playerWindow.hidden = false;
  playerTitle.textContent = game.name;
  gameModes.getMode(game)(game);
  gameStage.focus();
}

function closePlayerWindow() {
  cleanupRunningGame();
  playerWindow.hidden = true;
}

search.addEventListener('input', renderGames);
closeDetail.addEventListener('click', () => {
  detail.hidden = true;
});
playSelected.addEventListener('click', () => startGame());
closePlayer.addEventListener('click', closePlayerWindow);
backToLibrary.addEventListener('click', closePlayerWindow);
restartGame.addEventListener('click', () => startGame());

renderCategories();
renderGames();
updateClock();
setInterval(updateClock, 30000);
