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
const BASE_GROUND_LEVEL = 6;
const TERRAIN_FREQUENCY_DIVISOR = 2;
const TERRAIN_AMPLITUDE = 1.5;
const DIRT_LAYER_DEPTH = 3;
const LAKE_START_RATIO = 0.62;
const LAKE_WIDTH = 5;
const TREE_X_RATIO = 0.22;
const TREE_LEAF_OFFSETS = [-1, 0, 1];
const TREE_TRUNK_HEIGHT = 2;
const PLAYER_X_RATIO = 0.46;
const ABOVE_GROUND_OFFSET = 1;
const BLOCK_RENDER_WIDTH = 2;
const world = Array.from({ length: height }, () => Array(width).fill(blocks.air));
const groundLevels = [];

function setBlock(x, y, block) {
  if (x >= 0 && x < width && y >= 0 && y < height) {
    world[y][x] = block;
  }
}

for (let x = 0; x < width; x += 1) {
  const ground = BASE_GROUND_LEVEL + Math.round(Math.sin(x / TERRAIN_FREQUENCY_DIVISOR) * TERRAIN_AMPLITUDE);
  groundLevels[x] = ground;

  for (let y = 0; y < height; y += 1) {
    if (y < ground) {
      world[y][x] = blocks.air;
    } else if (y === ground) {
      world[y][x] = blocks.grass;
    } else if (y < ground + DIRT_LAYER_DEPTH) {
      world[y][x] = blocks.dirt;
    } else {
      world[y][x] = blocks.stone;
    }
  }
}

const lakeStart = Math.floor(width * LAKE_START_RATIO);
const lakeEnd = Math.min(width, lakeStart + LAKE_WIDTH);

for (let x = lakeStart; x < lakeEnd; x += 1) {
  setBlock(x, groundLevels[x], blocks.water);
}

const treeX = Math.floor(width * TREE_X_RATIO);
const treeTop = groundLevels[treeX] - TREE_TRUNK_HEIGHT - ABOVE_GROUND_OFFSET;

for (const leafOffset of TREE_LEAF_OFFSETS) {
  setBlock(treeX + leafOffset, treeTop, blocks.leaves);
}

for (let trunkOffset = 1; trunkOffset <= TREE_TRUNK_HEIGHT; trunkOffset += 1) {
  setBlock(treeX, treeTop + trunkOffset, blocks.wood);
}

const playerX = Math.floor(width * PLAYER_X_RATIO);
const playerY = groundLevels[playerX] - ABOVE_GROUND_OFFSET;
setBlock(playerX, playerY, blocks.player);

console.log('迷你方块冒险 / Mini Block Adventure');
console.log('='.repeat(width * BLOCK_RENDER_WIDTH));
console.log(world.map((row) => row.join('')).join('\n'));
console.log('\n背包 / Inventory: 草方块 Grass x8, 木头 Wood x3, 石头 Stone x12');
console.log('目标 / Goal: 采集资源、搭建小屋、探索湖边。Gather resources, build a hut, and explore the lake.');
