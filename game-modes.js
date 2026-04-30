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

  function rulesFor(game) {
    return game.rules || { target: game.icon, hazard: '障碍', objective: '完成挑战', winScore: 10, lives: 3, timeLimit: 35, speed: 8 };
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

  function addMovePad(state, moves) {
    const controls = document.createElement('div');
    controls.className = 'move-pad';
    [
      ['↑', '上', moves.up],
      ['←', '左', moves.left],
      ['→', '右', moves.right],
      ['↓', '下', moves.down],
    ].forEach(([label, name, handler]) => {
      const button = document.createElement('button');
      button.type = 'button';
      button.textContent = label;
      button.setAttribute('aria-label', `向${name}移动`);
      button.addEventListener('click', handler);
      controls.appendChild(button);
    });
    gameStage.appendChild(controls);
    addListener(state, document, 'keydown', (event) => {
      if (event.key === 'ArrowUp' || event.key.toLowerCase() === 'w') moves.up();
      if (event.key === 'ArrowDown' || event.key.toLowerCase() === 's') moves.down();
      if (event.key === 'ArrowLeft' || event.key.toLowerCase() === 'a') moves.left();
      if (event.key === 'ArrowRight' || event.key.toLowerCase() === 'd') moves.right();
    });
  }

  function startCasualGame(game) {
    const state = makeGameState(game);
    const rules = rulesFor(game);
    let score = 0;
    let combo = 0;

    playerInstructions.textContent = `${game.name}：点击 ${rules.target}，避开 ${rules.hazard}。${rules.timeLimit} 秒内拿到 ${rules.winScore} 分完成“${rules.objective}”。`;
    startCountdown(state, rules.timeLimit, () => finishGame(state, '时间到，挑战失败。', false));

    function spawnTarget() {
      if (state.over) return;
      gameStage.textContent = '';
      const target = document.createElement('button');
      const hazard = document.createElement('button');
      target.className = 'tap-target playable-pop';
      hazard.className = 'tap-target hazard-target playable-pop';
      target.type = 'button';
      hazard.type = 'button';
      target.textContent = game.icon;
      hazard.textContent = '⚠️';
      target.setAttribute('aria-label', `点击${rules.target}`);
      hazard.setAttribute('aria-label', `避开${rules.hazard}`);
      target.style.background = `linear-gradient(135deg, ${game.c1}, ${game.c2})`;
      hazard.style.background = 'linear-gradient(135deg, #ef4444, #7f1d1d)';
      positionElement(target, ...randomPosition(84));
      positionElement(hazard, ...randomPosition(84));
      target.addEventListener('click', () => {
        score += 1 + Math.min(combo, 3);
        combo += 1;
        state.score = score;
        updateHud(score, `连击 ${combo}`);
        if (score >= rules.winScore) {
          finishGame(state, `成功！${rules.objective}。`, true);
        } else {
          spawnTarget();
        }
      });
      hazard.addEventListener('click', () => {
        combo = 0;
        score = Math.max(0, score - 2);
        state.score = score;
        updateHud(score, '连击中断');
        spawnTarget();
      });
      gameStage.append(target, hazard);
    }

    spawnTarget();
  }

  function startPuzzleGame(game) {
    const state = makeGameState(game);
    const rules = rulesFor(game);
    const symbols = [game.icon, '⭐', '💡', '🎯', '🔥', '🌈'];
    const cards = [...symbols, ...symbols].sort(() => Math.random() - 0.5);
    const gridElement = document.createElement('div');
    let firstCard = null;
    let lock = false;
    let matched = 0;

    playerInstructions.textContent = `${game.name}：翻牌配对 ${rules.target} 和线索图案，配齐 ${symbols.length} 对即可${rules.objective}。`;
    updateHud(0, '配对 0');
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
          updateHud(matched, `配对 ${matched}`);
          if (matched === symbols.length) {
            finishGame(state, `通关！${rules.objective}。`, true);
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
        }, 650);
      });
      gridElement.appendChild(card);
    });

    gameStage.appendChild(gridElement);
  }

  function startRacerGame(game) {
    const state = makeGameState(game);
    const rules = rulesFor(game);
    let lane = 1;
    let score = 0;
    let lives = rules.lives;
    let ticks = 0;
    const player = document.createElement('div');
    const entities = [];

    playerInstructions.textContent = `${game.name}：←/→ 或 A/D 换道，收集 ${rules.target}，躲开 ${rules.hazard}，拿到 ${rules.winScore} 分${rules.objective}。`;
    updateHud(0, `生命 ${lives}`);

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
      positionElement(player, (lane + 0.5) * (width / 3) - 22, gameStage.clientHeight - 70);
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
      const isReward = Math.random() > 0.42;
      const entityLane = Math.floor(Math.random() * 3);
      entity.className = isReward ? 'racer-coin playable-pop' : 'racer-obstacle hazard-target playable-pop';
      entity.textContent = isReward ? game.icon : '🚧';
      entity.dataset.kind = isReward ? 'reward' : 'hazard';
      entity.dataset.y = '-48';
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
        const nextY = Number(entity.dataset.y) + rules.speed + 4;
        entity.dataset.y = String(nextY);
        entity.style.top = `${nextY}px`;
        if (rectsOverlap(player, entity)) {
          if (entity.dataset.kind === 'reward') {
            score += 1;
            state.score = score;
            updateHud(score, `生命 ${lives}`);
            if (score >= rules.winScore) finishGame(state, `冲线成功！${rules.objective}。`, true);
          } else {
            lives -= 1;
            updateHud(score, `生命 ${lives}`);
            if (lives <= 0) finishGame(state, `撞上${rules.hazard}，挑战失败。`, false);
          }
          entity.remove();
          entities.splice(entities.indexOf(entity), 1);
        } else if (nextY > gameStage.clientHeight + 48) {
          entity.remove();
          entities.splice(entities.indexOf(entity), 1);
        }
      });
    }, 80);
  }

  function startDodgeGame(game) {
    const state = makeGameState(game);
    const rules = rulesFor(game);
    let x = Math.max(gameStage.clientWidth / 2 - 40, 0);
    let score = 0;
    let lives = rules.lives;
    let ticks = 0;
    const player = document.createElement('div');
    const items = [];

    playerInstructions.textContent = `${game.name}：←/→ 或 A/D 移动，接住 ${rules.target}，躲开 ${rules.hazard}，达成 ${rules.objective}。`;
    updateHud(0, `生命 ${lives}`);
    player.className = 'catcher-basket';
    player.textContent = game.icon;
    gameStage.appendChild(player);

    function renderPlayer() {
      positionElement(player, x, gameStage.clientHeight - 62);
    }

    function move(delta) {
      x = Math.max(0, Math.min(gameStage.clientWidth - 80, x + delta));
      renderPlayer();
    }

    function spawnItem() {
      const item = document.createElement('div');
      const isReward = Math.random() > 0.28;
      item.className = isReward ? 'falling-item playable-pop' : 'falling-item hazard-target playable-pop';
      item.textContent = isReward ? game.icon : '💥';
      item.dataset.kind = isReward ? 'reward' : 'hazard';
      item.dataset.y = '-44';
      item.style.left = `${Math.random() * Math.max(gameStage.clientWidth - 44, 1)}px`;
      item.style.top = '-44px';
      items.push(item);
      gameStage.appendChild(item);
    }

    addArcadeControls(state, () => move(-36), () => move(36));
    renderPlayer();
    addListener(state, window, 'resize', renderPlayer);

    addInterval(state, () => {
      if (state.over) return;
      ticks += 1;
      if (ticks % 8 === 0) spawnItem();
      items.slice().forEach((item) => {
        const nextY = Number(item.dataset.y) + rules.speed + 2;
        item.dataset.y = String(nextY);
        item.style.top = `${nextY}px`;
        if (rectsOverlap(player, item)) {
          if (item.dataset.kind === 'hazard') {
            lives -= 1;
          } else {
            score += 1;
            state.score = score;
          }
          updateHud(score, `生命 ${lives}`);
          item.remove();
          items.splice(items.indexOf(item), 1);
          if (score >= rules.winScore) finishGame(state, `成功！${rules.objective}。`, true);
          if (lives <= 0) finishGame(state, `${rules.hazard} 太多，挑战失败。`, false);
        } else if (nextY > gameStage.clientHeight + 48) {
          if (item.dataset.kind === 'reward') lives -= 1;
          updateHud(score, `生命 ${lives}`);
          item.remove();
          items.splice(items.indexOf(item), 1);
          if (lives <= 0) finishGame(state, `漏掉太多 ${rules.target}，挑战失败。`, false);
        }
      });
    }, 80);
  }

  function startAdventureGame(game) {
    const state = makeGameState(game);
    const rules = rulesFor(game);
    const size = 5;
    const grid = document.createElement('div');
    let player = { x: 0, y: 0 };
    let score = 0;
    let lives = rules.lives;
    const rewards = new Set(['1,1', '3,0', '4,2', '2,3', '4,4', '0,4']);
    const hazards = new Set(['2,1', '1,3', '3,3']);

    playerInstructions.textContent = `${game.name}：用方向键/WASD 探索地图，收集 ${rules.target}，避开 ${rules.hazard}，最终${rules.objective}。`;
    updateHud(0, `生命 ${lives}`);
    grid.className = 'adventure-grid';
    gameStage.appendChild(grid);

    function keyOf(x, y) {
      return `${x},${y}`;
    }

    function render() {
      grid.textContent = '';
      for (let y = 0; y < size; y += 1) {
        for (let x = 0; x < size; x += 1) {
          const cell = document.createElement('div');
          const key = keyOf(x, y);
          cell.className = 'adventure-cell';
          if (player.x === x && player.y === y) {
            cell.classList.add('player-cell');
            cell.textContent = game.icon;
          } else if (rewards.has(key)) {
            cell.textContent = '✨';
          } else if (hazards.has(key)) {
            cell.textContent = '⚠️';
          }
          grid.appendChild(cell);
        }
      }
    }

    function move(dx, dy) {
      if (state.over) return;
      player = {
        x: Math.max(0, Math.min(size - 1, player.x + dx)),
        y: Math.max(0, Math.min(size - 1, player.y + dy)),
      };
      const key = keyOf(player.x, player.y);
      if (rewards.delete(key)) {
        score += 2;
        state.score = score;
      }
      if (hazards.delete(key)) {
        lives -= 1;
      }
      updateHud(score, `生命 ${lives}`);
      render();
      if (score >= rules.winScore || (player.x === 4 && player.y === 4 && score >= 8)) finishGame(state, `探索完成！${rules.objective}。`, true);
      if (lives <= 0) finishGame(state, `${rules.hazard} 阻止了探险。`, false);
    }

    addMovePad(state, {
      up: () => move(0, -1),
      down: () => move(0, 1),
      left: () => move(-1, 0),
      right: () => move(1, 0),
    });
    render();
  }

  function startDefenseGame(game) {
    const state = makeGameState(game);
    const rules = rulesFor(game);
    let score = 0;
    let core = rules.lives + 2;
    let ticks = 0;
    const enemies = [];

    playerInstructions.textContent = `${game.name}：点击来袭的 ${rules.hazard} 进行拦截，点到 ${rules.target} 可加分，守到 ${rules.winScore} 分完成“${rules.objective}”。`;
    updateHud(0, `核心 ${core}`);

    function spawnEnemy() {
      const enemy = document.createElement('button');
      const isBonus = Math.random() > 0.78;
      enemy.type = 'button';
      enemy.className = isBonus ? 'defense-enemy defense-bonus playable-pop' : 'defense-enemy playable-pop';
      enemy.textContent = isBonus ? game.icon : '👾';
      enemy.dataset.kind = isBonus ? 'bonus' : 'enemy';
      enemy.dataset.y = '-42';
      enemy.style.left = `${Math.random() * Math.max(gameStage.clientWidth - 52, 1)}px`;
      enemy.style.top = '-42px';
      enemy.addEventListener('click', () => {
        score += isBonus ? 2 : 1;
        state.score = score;
        updateHud(score, `核心 ${core}`);
        enemy.remove();
        enemies.splice(enemies.indexOf(enemy), 1);
        if (score >= rules.winScore) finishGame(state, `防守成功！${rules.objective}。`, true);
      });
      enemies.push(enemy);
      gameStage.appendChild(enemy);
    }

    addInterval(state, () => {
      if (state.over) return;
      ticks += 1;
      if (ticks % 10 === 0) spawnEnemy();
      enemies.slice().forEach((enemy) => {
        const nextY = Number(enemy.dataset.y) + rules.speed;
        enemy.dataset.y = String(nextY);
        enemy.style.top = `${nextY}px`;
        if (nextY > gameStage.clientHeight - 45) {
          if (enemy.dataset.kind === 'enemy') core -= 1;
          enemy.remove();
          enemies.splice(enemies.indexOf(enemy), 1);
          updateHud(score, `核心 ${core}`);
          if (core <= 0) finishGame(state, '核心被突破，防守失败。', false);
        }
      });
    }, 100);
  }

  function startSimulationGame(game) {
    const state = makeGameState(game);
    const rules = rulesFor(game);
    const panel = document.createElement('div');
    let score = 0;
    let energy = 6;
    let upgraded = false;

    playerInstructions.textContent = `${game.name}：经营资源，点击生产获得 ${rules.target}，能量耗尽或忽视 ${rules.hazard} 会失败。`;
    panel.className = 'sim-panel';
    gameStage.appendChild(panel);

    function render() {
      panel.textContent = '';
      updateHud(score, `能量 ${energy}`);
      [
        ['生产', `${game.icon} 生产 ${rules.target}`, () => {
          if (energy <= 0) return;
          score += upgraded ? 2 : 1;
          energy -= 1;
        }],
        ['补给', '恢复 2 点能量', () => {
          energy = Math.min(8, energy + 2);
        }],
        ['升级', '花 3 分升级效率', () => {
          if (score >= 3 && !upgraded) {
            score -= 3;
            upgraded = true;
          }
        }],
      ].forEach(([title, text, action]) => {
        const button = document.createElement('button');
        button.type = 'button';
        button.innerHTML = `<strong>${title}</strong><span>${text}</span>`;
        button.addEventListener('click', () => {
          action();
          state.score = score;
          render();
          if (score >= rules.winScore) finishGame(state, `经营成功！${rules.objective}。`, true);
          if (energy <= 0) finishGame(state, `${rules.hazard} 让系统停摆。`, false);
        });
        panel.appendChild(button);
      });
      const info = document.createElement('p');
      info.textContent = upgraded ? '已升级：每次生产 +2 分。' : '提示：先生产，分数足够后升级，再补给维持能量。';
      panel.appendChild(info);
    }

    render();
  }

  function startRhythmGame(game) {
    const state = makeGameState(game);
    const rules = rulesFor(game);
    const keys = ['A', 'S', 'D', 'F'];
    const panel = document.createElement('div');
    let score = 0;
    let lives = rules.lives;
    let current = keys[Math.floor(Math.random() * keys.length)];

    playerInstructions.textContent = `${game.name}：按下或点击当前节拍键，连中 ${rules.target}，漏拍会触发 ${rules.hazard}。`;
    panel.className = 'rhythm-panel';
    gameStage.appendChild(panel);
    startCountdown(state, rules.timeLimit, () => finishGame(state, '演出结束，分数不足。', false));

    function nextBeat() {
      current = keys[Math.floor(Math.random() * keys.length)];
      render();
    }

    function hit(key) {
      if (state.over) return;
      if (key === current) {
        score += 1;
        state.score = score;
        updateHud(score, `生命 ${lives}`);
        if (score >= rules.winScore) finishGame(state, `完美演出！${rules.objective}。`, true);
      } else {
        lives -= 1;
        updateHud(score, `生命 ${lives}`);
        if (lives <= 0) finishGame(state, `${rules.hazard} 太多，演出失败。`, false);
      }
      nextBeat();
    }

    function render() {
      panel.textContent = '';
      const beat = document.createElement('div');
      beat.className = 'rhythm-beat playable-pop';
      beat.textContent = current;
      panel.appendChild(beat);
      keys.forEach((key) => {
        const button = document.createElement('button');
        button.type = 'button';
        button.textContent = key;
        button.classList.toggle('active', key === current);
        button.addEventListener('click', () => hit(key));
        panel.appendChild(button);
      });
    }

    addListener(state, document, 'keydown', (event) => hit(event.key.toUpperCase()));
    addInterval(state, () => {
      if (state.over) return;
      lives -= 1;
      updateHud(score, `生命 ${lives}`);
      if (lives <= 0) finishGame(state, '连续漏拍，演出失败。', false);
      nextBeat();
    }, 2400);
    updateHud(0, `生命 ${lives}`);
    render();
  }

  function startCreativeGame(game) {
    const state = makeGameState(game);
    const rules = rulesFor(game);
    const grid = document.createElement('div');
    let painted = 0;
    const hazardCells = new Set([3, 8, 13]);

    playerInstructions.textContent = `${game.name}：点击格子绘制 ${rules.target}，避开 ${rules.hazard}，填满 12 格完成“${rules.objective}”。`;
    updateHud(0, '画布 0/12');
    grid.className = 'creative-grid';
    gameStage.appendChild(grid);

    for (let index = 0; index < 16; index += 1) {
      const cell = document.createElement('button');
      cell.type = 'button';
      cell.className = hazardCells.has(index) ? 'creative-cell creative-hazard' : 'creative-cell';
      cell.textContent = hazardCells.has(index) ? '✖' : '';
      cell.addEventListener('click', () => {
        if (state.over || cell.classList.contains('painted')) return;
        if (hazardCells.has(index)) {
          painted = Math.max(0, painted - 2);
          state.score = painted;
          updateHud(painted, '误触褪色');
          return;
        }
        cell.classList.add('painted');
        cell.textContent = game.icon;
        painted += 1;
        state.score = painted;
        updateHud(painted, `画布 ${painted}/12`);
        if (painted >= 12) finishGame(state, `创作完成！${rules.objective}。`, true);
      });
      grid.appendChild(cell);
    }
  }

  function getMode(game) {
    const mode = rulesFor(game).mode;
    if (mode === 'dodge') return startDodgeGame;
    if (mode === 'adventure') return startAdventureGame;
    if (mode === 'racer') return startRacerGame;
    if (mode === 'puzzle') return startPuzzleGame;
    if (mode === 'defense') return startDefenseGame;
    if (mode === 'simulation') return startSimulationGame;
    if (mode === 'rhythm') return startRhythmGame;
    if (mode === 'creative') return startCreativeGame;
    return startCasualGame;
  }

  return { getMode };
}

window.createGameModes = createGameModes;
