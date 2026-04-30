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
const LAKE_Y_RATIO = 0.58;
const LAKE_START_RATIO = 0.62;
const TREE_X_RATIO = 0.22;
const TREE_TOP_RATIO = 0.42;
const PLAYER_X_RATIO = 0.46;
const PLAYER_Y_RATIO = 0.42;
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

const lakeY = Math.floor(height * LAKE_Y_RATIO);
const lakeStart = Math.floor(width * LAKE_START_RATIO);
const lakeEnd = Math.min(width, lakeStart + 5);

for (let x = lakeStart; x < lakeEnd; x += 1) {
  setBlock(x, lakeY, blocks.water);
}

const treeX = Math.floor(width * TREE_X_RATIO);
const treeTop = Math.floor(height * TREE_TOP_RATIO);
setBlock(treeX - 1, treeTop, blocks.leaves);
setBlock(treeX, treeTop, blocks.leaves);
setBlock(treeX + 1, treeTop, blocks.leaves);
setBlock(treeX, treeTop + 1, blocks.wood);
setBlock(treeX, treeTop + 2, blocks.wood);

const playerX = Math.floor(width * PLAYER_X_RATIO);
const playerY = Math.floor(height * PLAYER_Y_RATIO);
setBlock(playerX, playerY, blocks.player);

console.log('迷你方块冒险 / Mini Block Adventure');
console.log('='.repeat(width * 2));
console.log(world.map((row) => row.join('')).join('\n'));
console.log('\n背包 / Inventory: 草方块 Grass x8, 木头 Wood x3, 石头 Stone x12');
console.log('目标 / Goal: 采集资源、搭建小屋、探索湖边。Gather resources, build a hut, and explore the lake.');
