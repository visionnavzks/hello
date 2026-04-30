(() => {
  'use strict';

  const canvas = document.getElementById('game');
  const ctx = canvas.getContext('2d');
  const title = document.getElementById('gameTitle');
  const scoreEl = document.getElementById('score');
  const livesEl = document.getElementById('lives');
  const timeEl = document.getElementById('time');
  const overlay = document.getElementById('overlay');
  const gamesEl = document.getElementById('games');
  const searchEl = document.getElementById('search');
  const quickStart = document.getElementById('quickStart');
  const pauseBtn = document.getElementById('pauseBtn');
  const restartBtn = document.getElementById('restartBtn');
  const W = canvas.width;
  const H = canvas.height;
  const keys = new Set();
  const pointer = { x: W / 2, y: H / 2, down: false, moved: false };

  const palettes = [
    ['#40f9ff', '#ff3df2', '#ffe45c', '#6dff86'],
    ['#ff7a3d', '#ffd166', '#06d6a0', '#118ab2'],
    ['#f72585', '#7209b7', '#4cc9f0', '#b8f7d4'],
    ['#b9ff66', '#ff5c8a', '#7b61ff', '#fff275'],
    ['#00f5d4', '#fee440', '#f15bb5', '#9b5de5']
  ];
  const groups = [
    ['霓虹闪避', 'dodge', '躲避'], ['宝石迷阵', 'collect', '收集'], ['太空接物', 'catch', '接物'],
    ['银河射手', 'shooter', '射击'], ['贪吃长蛇', 'snake', '蛇'], ['极速乒乓', 'pong', '反弹'],
    ['砖块爆破', 'brick', '打砖块'], ['迷宫逃脱', 'maze', '迷宫'], ['记忆翻牌', 'memory', '记忆'], ['节奏打击', 'rhythm', '节奏']
  ];
  const variants = ['街区', '深海', '火星', '丛林', '赛博'];
  const games = groups.flatMap((group, gi) => variants.map((variant, vi) => ({
    id: gi * 5 + vi + 1,
    name: `${variant}${group[0]}`,
    mode: group[1],
    genre: group[2],
    level: vi + 1,
    speed: 1 + vi * 0.22 + gi * 0.015,
    palette: palettes[vi]
  })));

  let active = null;
  let cards = [];
  let last = 0;
  let paused = false;
  let shotLock = false;

  const clamp = (v, min, max) => Math.max(min, Math.min(max, v));
  const rnd = (min, max) => min + Math.random() * (max - min);
  const hit = (a, b) => a.x < b.x + b.w && a.x + a.w > b.x && a.y < b.y + b.h && a.y + a.h > b.y;
  const circleHit = (a, b) => Math.hypot(a.x - b.x, a.y - b.y) < a.r + b.r;
  const choice = arr => arr[Math.floor(Math.random() * arr.length)];

  class ArcadeGame {
    constructor(config) {
      this.config = config;
      this.score = 0;
      this.lives = 3;
      this.time = 0;
      this.message = '';
      this.over = false;
      this.win = false;
      this.flash = 0;
      this.cooldown = 0;
      this.init();
    }

    init() {
      const init = initializers[this.config.mode];
      if (init) init(this);
    }

    update(dt) {
      if (this.over || paused) return;
      this.time += dt;
      this.flash = Math.max(0, this.flash - dt);
      const update = updaters[this.config.mode];
      if (update) update(this, dt);
      if (this.lives <= 0) this.end(false, '游戏结束，按 R 重开');
      updateHud();
    }

    draw() {
      drawBackdrop(this.config.palette);
      const draw = drawers[this.config.mode];
      if (draw) draw(this);
      drawFrame(this);
    }

    damage() {
      this.lives -= 1;
      this.flash = 0.35;
    }

    add(points) {
      this.score += points;
      if (this.score >= this.target && !this.over) this.end(true, '通关！按 R 再玩一次');
    }

    end(win, message) {
      this.over = true;
      this.win = win;
      this.message = message;
      updateHud();
    }
  }

  const initializers = {
    dodge(g) {
      g.target = 25 + g.config.level * 12;
      g.player = { x: W / 2, y: H - 75, w: 28, h: 28, speed: 245 };
      g.items = Array.from({ length: 5 + g.config.level }, () => spawnCircle(g, 'good'));
      g.bad = Array.from({ length: 4 + g.config.level }, () => spawnCircle(g, 'bad'));
    },
    collect(g) {
      g.target = 14 + g.config.level * 5;
      g.player = { x: W / 2, y: H / 2, w: 30, h: 30, speed: 220 };
      g.items = Array.from({ length: 12 + g.config.level * 3 }, () => spawnCircle(g, 'good'));
      g.bad = Array.from({ length: 5 + g.config.level }, () => spawnCircle(g, 'bad'));
    },
    catch(g) {
      g.target = 18 + g.config.level * 7;
      g.player = { x: W / 2 - 45, y: H - 54, w: 90, h: 18, speed: 310 };
      g.falls = [];
      g.spawn = 0;
    },
    shooter(g) {
      g.target = 22 + g.config.level * 8;
      g.player = { x: W / 2 - 18, y: H - 58, w: 36, h: 32, speed: 280 };
      g.bullets = [];
      g.enemies = Array.from({ length: 9 + g.config.level * 2 }, (_, i) => ({
        x: 70 + (i % 8) * 82, y: 42 + Math.floor(i / 8) * 46, w: 34, h: 26, dx: 55 * g.config.speed
      }));
      g.enemyBullets = [];
      g.dir = 1;
      g.fire = 0;
    },
    snake(g) {
      g.grid = 20;
      g.cols = Math.floor(W / g.grid);
      g.rows = Math.floor(H / g.grid);
      g.target = 8 + g.config.level * 4;
      g.step = Math.max(0.07, 0.16 - g.config.level * 0.015);
      g.tick = 0;
      g.dir = { x: 1, y: 0 };
      g.next = { x: 1, y: 0 };
      g.snake = [{ x: 8, y: 12 }, { x: 7, y: 12 }, { x: 6, y: 12 }];
      g.food = randomCell(g);
      g.lives = 1;
    },
    pong(g) {
      g.target = 9 + g.config.level * 3;
      g.player = { x: 28, y: H / 2 - 45, w: 16, h: 90, speed: 315 };
      g.ai = { x: W - 44, y: H / 2 - 45, w: 16, h: 90 };
      g.ball = { x: W / 2, y: H / 2, r: 10, vx: 210 * g.config.speed, vy: 140 };
      g.enemyScore = 0;
    },
    brick(g) {
      g.player = { x: W / 2 - 55, y: H - 45, w: 110, h: 16, speed: 330 };
      g.ball = { x: W / 2, y: H - 75, r: 9, vx: 160, vy: -235 * g.config.speed };
      g.bricks = [];
      const rows = 3 + g.config.level;
      const cols = 10;
      for (let y = 0; y < rows; y += 1) {
        for (let x = 0; x < cols; x += 1) g.bricks.push({ x: 50 + x * 70, y: 54 + y * 28, w: 58, h: 18, hp: 1 });
      }
      g.target = g.bricks.length * 5;
    },
    maze(g) {
      g.cell = 32;
      g.cols = 21;
      g.rows = 13;
      g.player = { x: 1, y: 1 };
      g.exit = { x: g.cols - 2, y: g.rows - 2 };
      g.target = 6 + g.config.level * 3;
      g.gems = [];
      g.bad = [];
      g.walls = buildMaze(g);
      while (g.gems.length < g.target) {
        const c = randomMazeCell(g);
        if (!sameCell(c, g.player) && !sameCell(c, g.exit)) g.gems.push(c);
      }
      while (g.bad.length < 3 + g.config.level) g.bad.push({ ...randomMazeCell(g), dx: choice([-1, 1]), dy: choice([-1, 1]) });
      g.moveTimer = 0;
    },
    memory(g) {
      const pairs = 6 + g.config.level * 2;
      g.target = pairs * 10;
      const symbols = '★◆●▲■♣♥☀☂⚡♪☯✿⬢'.split('').slice(0, pairs);
      g.deck = symbols.concat(symbols).sort(() => Math.random() - 0.5).map((s, i) => ({ s, i, open: false, done: false }));
      g.first = null;
      g.wait = 0;
      g.lives = 12 + g.config.level * 2;
    },
    rhythm(g) {
      g.target = 20 + g.config.level * 6;
      g.notes = [];
      g.spawn = 0;
      g.combo = 0;
      g.hitZone = H - 80;
    }
  };

  const updaters = {
    dodge(g, dt) {
      movePlayer(g.player, dt);
      g.items.forEach(o => moveCircle(o, dt));
      g.bad.forEach(o => moveCircle(o, dt));
      g.items.forEach(o => { if (circleHit(center(g.player, 16), o)) { g.add(5); Object.assign(o, spawnCircle(g, 'good')); } });
      g.bad.forEach(o => { if (circleHit(center(g.player, 15), o)) { g.damage(); Object.assign(o, spawnCircle(g, 'bad')); } });
    },
    collect(g, dt) {
      movePlayer(g.player, dt);
      g.bad.forEach(o => moveCircle(o, dt));
      g.items = g.items.filter(o => {
        if (circleHit(center(g.player, 16), o)) { g.add(1); return false; }
        return true;
      });
      g.bad.forEach(o => { if (circleHit(center(g.player, 16), o)) { g.damage(); Object.assign(o, spawnCircle(g, 'bad')); } });
      if (!g.items.length) g.end(true, '宝石全收集，胜利！');
    },
    catch(g, dt) {
      movePaddle(g.player, dt);
      g.spawn -= dt;
      if (g.spawn <= 0) {
        g.falls.push({ x: rnd(24, W - 24), y: -20, r: rnd(9, 16), vy: rnd(110, 190) * g.config.speed, good: Math.random() > 0.25 });
        g.spawn = Math.max(0.18, 0.72 - g.config.level * 0.08);
      }
      g.falls = g.falls.filter(o => {
        o.y += o.vy * dt;
        const caught = hit(g.player, { x: o.x - o.r, y: o.y - o.r, w: o.r * 2, h: o.r * 2 });
        if (caught && o.good) g.add(2);
        if (caught && !o.good) g.damage();
        if (o.y > H + 20 && o.good) g.damage();
        return !caught && o.y <= H + 24;
      });
    },
    shooter(g, dt) {
      movePaddle(g.player, dt);
      g.cooldown -= dt;
      g.fire -= dt;
      if (actionPressed() && g.cooldown <= 0) {
        g.bullets.push({ x: g.player.x + g.player.w / 2 - 3, y: g.player.y - 10, w: 6, h: 16, vy: -420 });
        g.cooldown = 0.18;
      }
      const edge = g.enemies.some(e => e.x < 25 || e.x + e.w > W - 25);
      if (edge) g.dir *= -1;
      g.enemies.forEach(e => { e.x += e.dx * g.dir * dt; if (edge) e.y += 12; });
      if (g.fire <= 0 && g.enemies.length) {
        const e = choice(g.enemies);
        g.enemyBullets.push({ x: e.x + e.w / 2, y: e.y + e.h, w: 6, h: 13, vy: 170 * g.config.speed });
        g.fire = Math.max(0.32, 1.2 - g.config.level * 0.12);
      }
      g.bullets.forEach(b => { b.y += b.vy * dt; });
      g.enemyBullets.forEach(b => { b.y += b.vy * dt; });
      g.enemies = g.enemies.filter(e => {
        const b = g.bullets.find(bullet => hit(e, bullet));
        if (b) { b.dead = true; g.add(3); return false; }
        if (e.y + e.h > g.player.y) { g.damage(); return false; }
        return true;
      });
      g.bullets = g.bullets.filter(b => !b.dead && b.y > -20);
      g.enemyBullets = g.enemyBullets.filter(b => { if (hit(g.player, b)) { g.damage(); return false; } return b.y < H + 20; });
      if (!g.enemies.length) g.end(true, '敌机清空，胜利！');
    },
    snake(g, dt) {
      setSnakeDir(g);
      g.tick += dt;
      if (g.tick < g.step) return;
      g.tick = 0;
      g.dir = g.next;
      const head = { x: g.snake[0].x + g.dir.x, y: g.snake[0].y + g.dir.y };
      if (head.x < 0 || head.y < 0 || head.x >= g.cols || head.y >= g.rows || g.snake.some(c => sameCell(c, head))) { g.damage(); return; }
      g.snake.unshift(head);
      if (sameCell(head, g.food)) { g.add(1); g.food = randomCell(g); } else g.snake.pop();
    },
    pong(g, dt) {
      moveVertical(g.player, dt);
      g.ai.y += clamp(g.ball.y - (g.ai.y + g.ai.h / 2), -220 * dt, 220 * dt);
      g.ai.y = clamp(g.ai.y, 0, H - g.ai.h);
      const b = g.ball;
      b.x += b.vx * dt; b.y += b.vy * dt;
      if (b.y < b.r || b.y > H - b.r) b.vy *= -1;
      if (hit(g.player, ballRect(b))) { b.vx = Math.abs(b.vx) * 1.04; b.vy += (b.y - (g.player.y + g.player.h / 2)) * 5; g.add(1); }
      if (hit(g.ai, ballRect(b))) b.vx = -Math.abs(b.vx) * 1.03;
      if (b.x < -20) { g.enemyScore += 1; g.damage(); resetBall(g, 1); }
      if (b.x > W + 20) { g.add(3); resetBall(g, -1); }
    },
    brick(g, dt) {
      movePaddle(g.player, dt);
      const b = g.ball;
      b.x += b.vx * dt; b.y += b.vy * dt;
      if (b.x < b.r || b.x > W - b.r) b.vx *= -1;
      if (b.y < b.r) b.vy *= -1;
      if (hit(g.player, ballRect(b)) && b.vy > 0) { b.vy *= -1; b.vx += (b.x - (g.player.x + g.player.w / 2)) * 4; }
      g.bricks = g.bricks.filter(br => {
        if (hit(br, ballRect(b))) { b.vy *= -1; g.add(5); return false; }
        return true;
      });
      if (b.y > H + 20) { g.damage(); b.x = W / 2; b.y = H - 75; b.vx = rnd(-160, 160); b.vy = -230; }
      if (!g.bricks.length) g.end(true, '砖块全部击碎！');
    },
    maze(g, dt) {
      g.moveTimer -= dt;
      if (g.moveTimer <= 0) { moveMazePlayer(g); g.moveTimer = 0.1; }
      g.bad.forEach(e => {
        const dx = Math.abs(e.x - g.player.x) > Math.abs(e.y - g.player.y) ? Math.sign(g.player.x - e.x) : 0;
        const dy = dx === 0 ? Math.sign(g.player.y - e.y) : 0;
        if (Math.random() < 0.03 * g.config.level) tryEnemyStep(g, e, dx, dy);
        if (Math.random() < 0.02) tryEnemyStep(g, e, choice([-1, 0, 1]), choice([-1, 0, 1]));
        if (sameCell(e, g.player)) { g.damage(); g.player = { x: 1, y: 1 }; }
      });
      g.gems = g.gems.filter(c => { if (sameCell(c, g.player)) { g.add(1); return false; } return true; });
      if (!g.gems.length && sameCell(g.player, g.exit)) g.end(true, '找到出口，逃脱成功！');
    },
    memory(g, dt) {
      if (g.wait > 0) { g.wait -= dt; if (g.wait <= 0) g.deck.forEach(c => { if (!c.done) c.open = false; }); }
    },
    rhythm(g, dt) {
      g.spawn -= dt;
      if (g.spawn <= 0) {
        g.notes.push({ x: 130 + Math.floor(Math.random() * 4) * 140, y: -20, r: 18, hit: false });
        g.spawn = Math.max(0.22, 0.78 - g.config.level * 0.08);
      }
      g.notes.forEach(n => { n.y += (180 + g.config.level * 28) * dt; });
      if (actionPressed()) {
        const n = g.notes.find(note => Math.abs(note.y - g.hitZone) < 34);
        if (n) { n.hit = true; g.combo += 1; g.add(2 + Math.min(8, g.combo)); } else { g.combo = 0; g.damage(); }
      }
      g.notes = g.notes.filter(n => { if (n.hit) return false; if (n.y > H + 30) { g.combo = 0; g.damage(); return false; } return true; });
    }
  };

  const drawers = {
    dodge(g) { drawCircles(g.items, g.config.palette[2]); drawCircles(g.bad, g.config.palette[1]); drawPlayer(g.player, g.config.palette[3]); },
    collect(g) { drawCircles(g.items, g.config.palette[2], 'gem'); drawCircles(g.bad, g.config.palette[1]); drawPlayer(g.player, g.config.palette[0]); },
    catch(g) { drawPaddle(g.player, g.config.palette[0]); g.falls.forEach(o => drawCircle(o.x, o.y, o.r, o.good ? g.config.palette[2] : g.config.palette[1])); },
    shooter(g) {
      drawPlayer(g.player, g.config.palette[0], true);
      g.enemies.forEach(e => drawRect(e, g.config.palette[1]));
      [...g.bullets, ...g.enemyBullets].forEach(b => drawRect(b, b.vy < 0 ? g.config.palette[2] : '#ff4b4b'));
    },
    snake(g) {
      g.snake.forEach((c, i) => drawCell(c, i ? g.config.palette[0] : g.config.palette[3], g.grid));
      drawCell(g.food, g.config.palette[2], g.grid);
    },
    pong(g) { drawPaddle(g.player, g.config.palette[0]); drawPaddle(g.ai, g.config.palette[1]); drawCircle(g.ball.x, g.ball.y, g.ball.r, g.config.palette[2]); centerText(`${g.score} : ${g.enemyScore}`, W / 2, 48, 30, 'rgba(255,255,255,.35)'); },
    brick(g) { drawPaddle(g.player, g.config.palette[0]); drawCircle(g.ball.x, g.ball.y, g.ball.r, g.config.palette[2]); g.bricks.forEach((b, i) => drawRect(b, g.config.palette[i % g.config.palette.length])); },
    maze(g) {
      const ox = 64, oy = 52, s = g.cell;
      g.walls.forEach(c => drawRect({ x: ox + c.x * s, y: oy + c.y * s, w: s - 2, h: s - 2 }, '#2f214d'));
      g.gems.forEach(c => drawCircle(ox + c.x * s + s / 2, oy + c.y * s + s / 2, 8, g.config.palette[2]));
      g.bad.forEach(c => drawCircle(ox + c.x * s + s / 2, oy + c.y * s + s / 2, 11, g.config.palette[1]));
      drawRect({ x: ox + g.exit.x * s + 4, y: oy + g.exit.y * s + 4, w: s - 8, h: s - 8 }, g.config.palette[3]);
      drawRect({ x: ox + g.player.x * s + 6, y: oy + g.player.y * s + 6, w: s - 12, h: s - 12 }, g.config.palette[0]);
    },
    memory(g) {
      const cols = 7, w = 86, h = 72, gap = 12, ox = (W - cols * w - (cols - 1) * gap) / 2, oy = 70;
      g.deck.forEach((card, i) => {
        const x = ox + (i % cols) * (w + gap), y = oy + Math.floor(i / cols) * (h + gap);
        drawRect({ x, y, w, h }, card.open || card.done ? g.config.palette[0] : '#2c1a49');
        if (card.open || card.done) centerText(card.s, x + w / 2, y + h / 2 + 12, 34, '#101020');
      });
    },
    rhythm(g) {
      for (let i = 0; i < 4; i += 1) drawRect({ x: 95 + i * 140, y: 34, w: 70, h: H - 70 }, 'rgba(255,255,255,.05)');
      drawRect({ x: 90, y: g.hitZone - 8, w: 560, h: 16 }, g.config.palette[3]);
      g.notes.forEach(n => drawCircle(n.x, n.y, n.r, g.config.palette[2]));
      centerText(`COMBO ${g.combo}`, W - 120, 54, 24, g.config.palette[0]);
    }
  };

  function spawnCircle(g, type) {
    return { x: rnd(30, W - 30), y: rnd(35, H - 45), r: rnd(9, type === 'bad' ? 17 : 14), vx: rnd(-90, 90) * g.config.speed, vy: rnd(-90, 90) * g.config.speed };
  }
  function center(rect, r) { return { x: rect.x + rect.w / 2, y: rect.y + rect.h / 2, r }; }
  function moveCircle(o, dt) { o.x += o.vx * dt; o.y += o.vy * dt; if (o.x < o.r || o.x > W - o.r) o.vx *= -1; if (o.y < o.r || o.y > H - o.r) o.vy *= -1; }
  function movePlayer(p, dt) { movePaddle(p, dt); moveVertical(p, dt); }
  function movePaddle(p, dt) {
    const dir = (keys.has('ArrowRight') || keys.has('d') ? 1 : 0) - (keys.has('ArrowLeft') || keys.has('a') ? 1 : 0);
    p.x = clamp(pointer.moved ? pointer.x - p.w / 2 : p.x + dir * p.speed * dt, 0, W - p.w);
  }
  function moveVertical(p, dt) {
    const dir = (keys.has('ArrowDown') || keys.has('s') ? 1 : 0) - (keys.has('ArrowUp') || keys.has('w') ? 1 : 0);
    p.y = clamp(pointer.moved ? pointer.y - p.h / 2 : p.y + dir * p.speed * dt, 0, H - p.h);
  }
  function actionPressed() {
    const pressed = keys.has(' ') || pointer.down;
    if (pressed && !shotLock) { shotLock = true; return true; }
    if (!pressed) shotLock = false;
    return false;
  }
  function randomCell(g) { let c; do { c = { x: Math.floor(rnd(0, g.cols)), y: Math.floor(rnd(0, g.rows)) }; } while (g.snake?.some(s => sameCell(s, c))); return c; }
  function sameCell(a, b) { return a.x === b.x && a.y === b.y; }
  function setSnakeDir(g) {
    const desired = keys.has('ArrowUp') || keys.has('w') ? { x: 0, y: -1 } : keys.has('ArrowDown') || keys.has('s') ? { x: 0, y: 1 } : keys.has('ArrowLeft') || keys.has('a') ? { x: -1, y: 0 } : keys.has('ArrowRight') || keys.has('d') ? { x: 1, y: 0 } : g.next;
    if (desired.x !== -g.dir.x || desired.y !== -g.dir.y) g.next = desired;
  }
  function ballRect(b) { return { x: b.x - b.r, y: b.y - b.r, w: b.r * 2, h: b.r * 2 }; }
  function resetBall(g, dir) { g.ball.x = W / 2; g.ball.y = H / 2; g.ball.vx = dir * 210 * g.config.speed; g.ball.vy = rnd(-170, 170); }
  function buildMaze(g) {
    const walls = [];
    for (let y = 0; y < g.rows; y += 1) for (let x = 0; x < g.cols; x += 1) {
      if (x === 0 || y === 0 || x === g.cols - 1 || y === g.rows - 1 || (x % 2 === 0 && y % 2 === 0) || (Math.random() < 0.18 + g.config.level * 0.02 && !(x < 3 && y < 3) && !(x > g.cols - 4 && y > g.rows - 4))) walls.push({ x, y });
    }
    return walls.filter(w => !(w.x === 1 && w.y === 1) && !(w.x === g.cols - 2 && w.y === g.rows - 2));
  }
  function randomMazeCell(g) { let c; do { c = { x: Math.floor(rnd(1, g.cols - 1)), y: Math.floor(rnd(1, g.rows - 1)) }; } while (isWall(g, c)); return c; }
  function isWall(g, c) { return g.walls.some(w => sameCell(w, c)); }
  function moveMazePlayer(g) {
    const dx = (keys.has('ArrowRight') || keys.has('d') ? 1 : 0) - (keys.has('ArrowLeft') || keys.has('a') ? 1 : 0);
    const dy = (keys.has('ArrowDown') || keys.has('s') ? 1 : 0) - (keys.has('ArrowUp') || keys.has('w') ? 1 : 0);
    const next = { x: g.player.x + dx, y: g.player.y + dy };
    if ((dx || dy) && !isWall(g, next)) g.player = next;
  }
  function tryEnemyStep(g, e, dx, dy) { const next = { x: e.x + dx, y: e.y + dy }; if (!isWall(g, next)) { e.x = next.x; e.y = next.y; } }

  function drawBackdrop(p) {
    ctx.clearRect(0, 0, W, H);
    const gradient = ctx.createLinearGradient(0, 0, W, H);
    gradient.addColorStop(0, '#05020a'); gradient.addColorStop(1, '#160d26');
    ctx.fillStyle = gradient; ctx.fillRect(0, 0, W, H);
    ctx.strokeStyle = 'rgba(64,249,255,.08)'; ctx.lineWidth = 1;
    for (let x = 0; x < W; x += 40) { ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, H); ctx.stroke(); }
    for (let y = 0; y < H; y += 40) { ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(W, y); ctx.stroke(); }
    ctx.fillStyle = p[1] + '18'; ctx.beginPath(); ctx.arc(W - 90, 70, 120, 0, Math.PI * 2); ctx.fill();
  }
  function drawFrame(g) {
    if (g.flash) { ctx.fillStyle = `rgba(255,40,80,${g.flash})`; ctx.fillRect(0, 0, W, H); }
    if (paused) centerText('暂停', W / 2, H / 2, 54, '#ffe45c');
    if (g.over) { ctx.fillStyle = 'rgba(0,0,0,.62)'; ctx.fillRect(0, 0, W, H); centerText(g.win ? '胜利!' : '挑战失败', W / 2, H / 2 - 30, 54, g.win ? '#6dff86' : '#ff5c8a'); centerText(g.message, W / 2, H / 2 + 28, 24, '#fff'); }
  }
  function drawRect(r, color) { ctx.fillStyle = color; ctx.shadowColor = color; ctx.shadowBlur = 12; ctx.fillRect(r.x, r.y, r.w, r.h); ctx.shadowBlur = 0; }
  function drawCircle(x, y, r, color) { ctx.fillStyle = color; ctx.shadowColor = color; ctx.shadowBlur = 14; ctx.beginPath(); ctx.arc(x, y, r, 0, Math.PI * 2); ctx.fill(); ctx.shadowBlur = 0; }
  function drawCircles(items, color, shape) { items.forEach(o => shape === 'gem' ? drawDiamond(o.x, o.y, o.r, color) : drawCircle(o.x, o.y, o.r, color)); }
  function drawDiamond(x, y, r, color) { ctx.fillStyle = color; ctx.beginPath(); ctx.moveTo(x, y - r); ctx.lineTo(x + r, y); ctx.lineTo(x, y + r); ctx.lineTo(x - r, y); ctx.closePath(); ctx.fill(); }
  function drawPlayer(p, color, ship) { drawRect(p, color); if (ship) drawCircle(p.x + p.w / 2, p.y - 5, 7, '#fff'); }
  function drawPaddle(p, color) { drawRect(p, color); }
  function drawCell(c, color, size) { drawRect({ x: c.x * size + 1, y: c.y * size + 1, w: size - 2, h: size - 2 }, color); }
  function centerText(text, x, y, size, color) { ctx.fillStyle = color; ctx.font = `900 ${size}px system-ui, sans-serif`; ctx.textAlign = 'center'; ctx.textBaseline = 'middle'; ctx.fillText(text, x, y); }

  function selectGame(config) {
    active = new ArcadeGame(config);
    paused = false;
    pointer.moved = false;
    title.textContent = `${String(config.id).padStart(2, '0')} ${config.name}`;
    overlay.classList.add('hidden');
    cards.forEach(btn => btn.classList.toggle('active', Number(btn.dataset.id) === config.id));
    updateHud();
  }
  function updateHud() {
    scoreEl.textContent = active ? active.score : 0;
    livesEl.textContent = active ? active.lives : 0;
    timeEl.textContent = active ? Math.floor(active.time) : 0;
  }
  function renderLibrary(filter = '') {
    gamesEl.innerHTML = '';
    const q = filter.trim().toLowerCase();
    games.filter(g => !q || `${g.name} ${g.genre} ${g.mode}`.toLowerCase().includes(q)).forEach(g => {
      const btn = document.createElement('button');
      btn.className = 'game-card';
      btn.type = 'button';
      btn.dataset.id = g.id;
      btn.innerHTML = `<small>#${String(g.id).padStart(2, '0')} · ${g.genre} · 难度 ${g.level}</small><strong>${g.name}</strong><span>${modeText(g.mode)}</span>`;
      btn.addEventListener('click', () => selectGame(g));
      gamesEl.appendChild(btn);
    });
    cards = [...document.querySelectorAll('.game-card')];
  }
  function modeText(mode) {
    return {
      dodge: '躲开敌人并收集能量', collect: '限生命收集全部宝石', catch: '移动托盘接住奖励', shooter: '发射子弹清空敌阵', snake: '吃食物变长别撞墙', pong: '和电脑对打反弹球', brick: '控制挡板击碎砖块', maze: '收集宝石后抵达出口', memory: '翻开并配对所有卡牌', rhythm: '音符到判定线时点击'
    }[mode];
  }

  canvas.addEventListener('pointerdown', event => { setPointer(event); pointer.down = true; handleCanvasClick(); });
  canvas.addEventListener('pointermove', event => { setPointer(event); pointer.moved = true; });
  canvas.addEventListener('pointerup', () => { pointer.down = false; setTimeout(() => { pointer.moved = false; }, 200); });
  function setPointer(event) {
    const rect = canvas.getBoundingClientRect();
    pointer.x = clamp((event.clientX - rect.left) * W / rect.width, 0, W);
    pointer.y = clamp((event.clientY - rect.top) * H / rect.height, 0, H);
  }
  function handleCanvasClick() {
    if (!active || active.config.mode !== 'memory' || active.wait > 0 || active.over) return;
    const cols = 7, w = 86, h = 72, gap = 12, ox = (W - cols * w - (cols - 1) * gap) / 2, oy = 70;
    const index = active.deck.findIndex((card, i) => {
      const x = ox + (i % cols) * (w + gap), y = oy + Math.floor(i / cols) * (h + gap);
      return pointer.x >= x && pointer.x <= x + w && pointer.y >= y && pointer.y <= y + h && !card.open && !card.done;
    });
    if (index < 0) return;
    const card = active.deck[index];
    card.open = true;
    if (active.first === null) { active.first = index; return; }
    const first = active.deck[active.first];
    if (first.s === card.s) { first.done = true; card.done = true; active.add(10); } else { active.lives -= 1; active.wait = 0.65; }
    active.first = null;
    if (active.deck.every(c => c.done)) active.end(true, '记忆大师，全部配对！');
    updateHud();
  }

  window.addEventListener('keydown', event => {
    if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', ' '].includes(event.key)) event.preventDefault();
    keys.add(event.key.length === 1 ? event.key.toLowerCase() : event.key);
    if (event.key.toLowerCase() === 'p') paused = !paused;
    if (event.key.toLowerCase() === 'r' && active) selectGame(active.config);
  });
  window.addEventListener('keyup', event => keys.delete(event.key.length === 1 ? event.key.toLowerCase() : event.key));
  quickStart.addEventListener('click', () => selectGame(choice(games)));
  pauseBtn.addEventListener('click', () => { paused = !paused; });
  restartBtn.addEventListener('click', () => { if (active) selectGame(active.config); });
  searchEl.addEventListener('input', () => renderLibrary(searchEl.value));

  function loop(now) {
    const dt = Math.min(0.033, (now - last) / 1000 || 0);
    last = now;
    if (active) { active.update(dt); active.draw(); } else { drawBackdrop(palettes[0]); centerText('选择下方任意卡带开始', W / 2, H / 2, 32, '#ffe45c'); }
    requestAnimationFrame(loop);
  }

  renderLibrary();
  updateHud();
  requestAnimationFrame(loop);
})();
