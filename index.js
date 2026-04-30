#!/usr/bin/env node

const blocks = {
  air: '  ',
  grass: '🟩',
  dirt: '🟫',
  stone: '⬜',
  water: '🟦',
  wood: '🪵',
  leaves: '🟢',
  player: '🙂',
};

const width = 24;
const height = 12;
const world = Array.from({ length: height }, () => Array(width).fill(blocks.air));

function setBlock(x, y, block) {
  if (x >= 0 && x < width && y >= 0 && y < height) {
    world[y][x] = block;
  }
}

for (let x = 0; x < width; x += 1) {
  const ground = 6 + Math.round(Math.sin(x / 2) * 1.5);

  for (let y = 0; y < height; y += 1) {
    if (y < ground) {
      world[y][x] = blocks.air;
    } else if (y === ground) {
      world[y][x] = blocks.grass;
    } else if (y < ground + 3) {
      world[y][x] = blocks.dirt;
    } else {
      world[y][x] = blocks.stone;
    }
  }
}

const lakeY = Math.floor(height * 0.58);
const lakeStart = Math.floor(width * 0.62);
const lakeEnd = Math.min(width, lakeStart + 5);

for (let x = lakeStart; x < lakeEnd; x += 1) {
  setBlock(x, lakeY, blocks.water);
}

const treeX = Math.floor(width * 0.22);
const treeTop = Math.floor(height * 0.42);
setBlock(treeX - 1, treeTop, blocks.leaves);
setBlock(treeX, treeTop, blocks.leaves);
setBlock(treeX + 1, treeTop, blocks.leaves);
setBlock(treeX, treeTop + 1, blocks.wood);
setBlock(treeX, treeTop + 2, blocks.wood);

const playerX = Math.floor(width * 0.46);
const playerY = Math.floor(height * 0.42);
setBlock(playerX, playerY, blocks.player);

console.log('我的世界：迷你方块冒险');
console.log('='.repeat(width));
console.log(world.map((row) => row.join('')).join('\n'));
console.log('\n背包: 草方块 x8, 木头 x3, 石头 x12');
console.log('目标: 采集资源、搭建小屋、探索湖边。');
