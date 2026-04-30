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

for (let x = 15; x < 20; x += 1) {
  world[7][x] = blocks.water;
}

world[5][4] = blocks.leaves;
world[5][5] = blocks.leaves;
world[5][6] = blocks.leaves;
world[6][5] = blocks.wood;
world[7][5] = blocks.wood;
world[5][11] = blocks.player;

console.log('我的世界：迷你方块冒险');
console.log('='.repeat(24));
console.log(world.map((row) => row.join('')).join('\n'));
console.log('\n背包: 草方块 x8, 木头 x3, 石头 x12');
console.log('目标: 采集资源、搭建小屋、探索湖边。');
