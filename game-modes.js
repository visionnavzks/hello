function createGameModes(context) {
  const {
    gameStage,
    playerInstructions,
    updateHud,
    makeGameState,
    addInterval,
    addTimeout,
    addListener,
    startCountdown,
    finishGame,
    positionElement,
    randomPosition,
  } = context;

  function startTapGame(game) {
    const state = makeGameState(game);
    let score = 0;
    state.score = score;
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
        state.score = score;
        updateHud(score, state.time);
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
    const symbols = [game.icon, '⭐', '💡', '🎯', '🔥', '🌈'];
    const cards = [...symbols, ...symbols].sort(() => Math.random() - 0.5);
    const gridElement = document.createElement('div');
    let firstCard = null;
    let lock = false;
    let matched = 0;

    state.score = matched;
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
          state.score = matched;
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
    left.setAttribute('aria-label', '向左移动');
    right.setAttribute('aria-label', '向右移动');
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

    state.score = score;
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
            state.score = score;
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

    state.score = score;
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
            state.score = score;
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

  return { getMode };
}

window.createGameModes = createGameModes;
