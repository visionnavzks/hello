const canvas = document.querySelector('#world');
const context = canvas.getContext('2d');
const statusText = document.querySelector('#status');
const picker = document.querySelector('#block-picker');
const modeToggle = document.querySelector('#mode-toggle');
const timeToggle = document.querySelector('#time-toggle');
const regenerateButton = document.querySelector('#regenerate');

const tileSize = 24;
const columns = Math.floor(canvas.width / tileSize);
const rows = Math.floor(canvas.height / tileSize);
const baseGround = Math.floor(rows * 0.55);
const dirtDepth = 4;
const playerWidth = tileSize * 0.72;
const playerHeight = tileSize * 1.25;
const waterWaveInsetX = 4;
const waterWaveY = 7;
const waterWaveWidthOffset = 8;
const waterWaveHeight = 3;
const celestialBodyOffsetX = 110;
const celestialBodyY = 78;
const celestialBodyRadius = 34;
const playerHeadInsetX = 5;
const playerHeadHeight = 10;
const playerBodyBottomInset = 16;
const playerFootInsetX = 3;
const playerFootHeight = 8;
const playerFootWidthOffset = 6;

const blockTypes = {
  grass: { label: '草地', color: '#5fbf45', stroke: '#3d7f2f', solid: true },
  dirt: { label: '泥土', color: '#8d5a35', stroke: '#5b3823', solid: true },
  stone: { label: '石头', color: '#9ca3af', stroke: '#6b7280', solid: true },
  wood: { label: '木头', color: '#9a652e', stroke: '#68421f', solid: true },
  leaves: { label: '树叶', color: '#2f9e44', stroke: '#1f6f32', solid: true },
  water: { label: '水', color: '#3aa6ff', stroke: '#1c75c8', solid: false },
  sand: { label: '沙子', color: '#d9c16f', stroke: '#ad974c', solid: true },
};

const inventory = ['grass', 'dirt', 'stone', 'wood', 'leaves', 'water', 'sand'];
const keys = new Set();
let selectedBlock = 'grass';
let buildMode = 'place';
let isNight = false;
let player = { column: 5, row: 5 };
let world = createWorld();

function terrainHeight(column) {
  return baseGround + Math.round(Math.sin(column / 3) * 2 + Math.cos(column / 6) * 1.5);
}

function createWorld() {
  const nextWorld = Array.from({ length: rows }, () => Array(columns).fill(null));

  for (let column = 0; column < columns; column += 1) {
    const ground = terrainHeight(column);

    for (let row = ground; row < rows; row += 1) {
      if (row === ground) {
        nextWorld[row][column] = 'grass';
      } else if (row < ground + dirtDepth) {
        nextWorld[row][column] = 'dirt';
      } else {
        nextWorld[row][column] = 'stone';
      }
    }
  }

  addLake(nextWorld, Math.floor(columns * 0.64), 6);
  addTree(nextWorld, Math.floor(columns * 0.2));
  addTree(nextWorld, Math.floor(columns * 0.78));

  player = { column: 5, row: terrainHeight(5) - 1 };
  return nextWorld;
}

function addLake(targetWorld, startColumn, lakeWidth) {
  for (let column = startColumn; column < Math.min(columns, startColumn + lakeWidth); column += 1) {
    const ground = terrainHeight(column);
    targetWorld[ground][column] = 'water';
  }
}

function addTree(targetWorld, column) {
  const ground = terrainHeight(column);
  const trunkTop = ground - 4;

  for (let row = ground - 1; row >= trunkTop + 1; row -= 1) {
    setBlock(targetWorld, column, row, 'wood');
  }

  for (let row = trunkTop; row <= trunkTop + 1; row += 1) {
    for (let offset = -2; offset <= 2; offset += 1) {
      if (Math.abs(offset) < 2 || row === trunkTop + 1) {
        setBlock(targetWorld, column + offset, row, 'leaves');
      }
    }
  }
}

function setBlock(targetWorld, column, row, block) {
  if (isValidPosition(column, row)) {
    targetWorld[row][column] = block;
  }
}

function isValidPosition(column, row) {
  return column >= 0 && column < columns && row >= 0 && row < rows;
}

function isSolid(column, row) {
  if (!isValidPosition(column, row)) {
    return true;
  }

  const block = world[row][column];
  return block ? blockTypes[block].solid : false;
}

function movePlayer(deltaColumn, deltaRow) {
  const nextColumn = player.column + deltaColumn;
  const nextRow = player.row + deltaRow;

  if (!isSolid(nextColumn, nextRow)) {
    player = { column: nextColumn, row: nextRow };
  }
}

function updatePlayer() {
  if (keys.has('arrowleft') || keys.has('a')) {
    movePlayer(-1, 0);
  }

  if (keys.has('arrowright') || keys.has('d')) {
    movePlayer(1, 0);
  }

  if ((keys.has('arrowup') || keys.has('w')) && isSolid(player.column, player.row + 1)) {
    movePlayer(0, -1);
  }

  if (!isSolid(player.column, player.row + 1)) {
    movePlayer(0, 1);
  }
}

function drawBlock(column, row, type) {
  const block = blockTypes[type];
  const x = column * tileSize;
  const y = row * tileSize;

  context.fillStyle = block.color;
  context.fillRect(x, y, tileSize, tileSize);
  context.strokeStyle = block.stroke;
  context.lineWidth = 2;
  context.strokeRect(x + 1, y + 1, tileSize - 2, tileSize - 2);

  if (type === 'water') {
    context.fillStyle = 'rgba(255, 255, 255, 0.25)';
    context.fillRect(x + waterWaveInsetX, y + waterWaveY, tileSize - waterWaveWidthOffset, waterWaveHeight);
  }
}

function drawSky() {
  const gradient = context.createLinearGradient(0, 0, 0, canvas.height);
  gradient.addColorStop(0, isNight ? '#111827' : '#7dd3fc');
  gradient.addColorStop(1, isNight ? '#1f2937' : '#dbeafe');
  context.fillStyle = gradient;
  context.fillRect(0, 0, canvas.width, canvas.height);

  context.fillStyle = isNight ? '#f8fafc' : '#fde68a';
  context.beginPath();
  context.arc(canvas.width - celestialBodyOffsetX, celestialBodyY, celestialBodyRadius, 0, Math.PI * 2);
  context.fill();
}

function drawPlayer() {
  const x = player.column * tileSize + (tileSize - playerWidth) / 2;
  const y = player.row * tileSize + tileSize - playerHeight;

  context.fillStyle = '#f8d7a3';
  context.fillRect(x + playerHeadInsetX, y, playerWidth - playerHeadInsetX * 2, playerHeadHeight);
  context.fillStyle = '#ef4444';
  context.fillRect(x, y + playerHeadHeight, playerWidth, playerHeight - playerBodyBottomInset);
  context.fillStyle = '#1f2937';
  context.fillRect(
    x + playerFootInsetX,
    y + playerHeight - playerFootHeight,
    playerWidth - playerFootWidthOffset,
    playerFootHeight,
  );
}

function render() {
  drawSky();

  for (let row = 0; row < rows; row += 1) {
    for (let column = 0; column < columns; column += 1) {
      const type = world[row][column];
      if (type) {
        drawBlock(column, row, type);
      }
    }
  }

  drawPlayer();
  statusText.textContent = `模式 / Mode: ${buildMode === 'place' ? '放置 Place' : '挖掘 Mine'} · 选中 / Selected: ${blockTypes[selectedBlock].label}`;
}

function gameLoop() {
  updatePlayer();
  render();
  requestAnimationFrame(gameLoop);
}

function buildBlockPicker() {
  picker.innerHTML = '';

  for (const block of inventory) {
    const button = document.createElement('button');
    button.type = 'button';
    button.className = 'block-button';
    button.textContent = blockTypes[block].label;
    button.style.background = blockTypes[block].color;
    button.style.color = '#08111f';
    button.setAttribute('aria-pressed', String(block === selectedBlock));
    button.addEventListener('click', () => {
      selectedBlock = block;
      buildBlockPicker();
    });
    picker.append(button);
  }
}

canvas.addEventListener('click', (event) => {
  const rect = canvas.getBoundingClientRect();
  const scaleX = canvas.width / rect.width;
  const scaleY = canvas.height / rect.height;
  const column = Math.floor((event.clientX - rect.left) * scaleX / tileSize);
  const row = Math.floor((event.clientY - rect.top) * scaleY / tileSize);

  if (!isValidPosition(column, row)) {
    return;
  }

  if (column === player.column && row === player.row) {
    return;
  }

  world[row][column] = buildMode === 'place' ? selectedBlock : null;
  render();
});

window.addEventListener('keydown', (event) => {
  const key = event.key.toLowerCase();

  if (key === ' ') {
    event.preventDefault();
    toggleBuildMode();
    return;
  }

  keys.add(key);
});

window.addEventListener('keyup', (event) => {
  keys.delete(event.key.toLowerCase());
});

function toggleBuildMode() {
  buildMode = buildMode === 'place' ? 'mine' : 'place';
  modeToggle.textContent = `当前：${buildMode === 'place' ? '放置' : '挖掘'}`;
}

modeToggle.addEventListener('click', toggleBuildMode);

timeToggle.addEventListener('click', () => {
  isNight = !isNight;
  render();
});

regenerateButton.addEventListener('click', () => {
  world = createWorld();
  render();
});

buildBlockPicker();
gameLoop();
