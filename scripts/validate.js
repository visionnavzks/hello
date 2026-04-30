#!/usr/bin/env node

const fs = require('fs');
const path = require('path');
const vm = require('vm');
const { execFileSync } = require('child_process');

const ROOT = path.join(__dirname, '..');
const filesToCheck = ['index.js', 'game-data.js', 'game-modes.js', 'app.js'];

function read(file) {
  return fs.readFileSync(path.join(ROOT, file), 'utf8');
}

function checkSyntax(file) {
  execFileSync(process.execPath, ['--check', path.join(ROOT, file)], { stdio: 'pipe' });
}

function loadGameData() {
  const sandbox = { window: {} };
  vm.createContext(sandbox);
  vm.runInContext(read('game-data.js'), sandbox, { filename: 'game-data.js' });
  return sandbox.window.GameOSData;
}

filesToCheck.forEach(checkSyntax);

const html = read('index.html');
let previousScriptIndex = -1;
['game-data.js', 'game-modes.js', 'app.js'].forEach((script) => {
  const scriptIndex = html.indexOf(`src="${script}"`);
  if (scriptIndex === -1) {
    throw new Error(`index.html is missing ${script}`);
  }
  if (scriptIndex < previousScriptIndex) {
    throw new Error(`index.html loads ${script} out of order`);
  }
  previousScriptIndex = scriptIndex;
});

const { games, gradients, categories } = loadGameData();
const supportedModes = new Set([
  'dodge',
  'adventure',
  'racer',
  'puzzle',
  'defense',
  'simulation',
  'rhythm',
  'creative',
  'casual',
]);
const gameTypes = Array.from(new Set(games.map((game) => game.type)));
const missingCategories = gameTypes.filter((type) => !categories.includes(type));

if (games.length !== 50) {
  throw new Error(`Expected 50 games, found ${games.length}`);
}

if (!Array.isArray(gradients) || gradients.length === 0) {
  throw new Error('Expected at least one game gradient');
}

if (!categories.includes('全部')) {
  throw new Error('Categories must include 全部');
}

if (missingCategories.length > 0) {
  throw new Error(`Missing categories: ${missingCategories.join(', ')}`);
}

games.forEach((game, index) => {
  const label = game.name || `game #${index + 1}`;
  if (!game.id || !game.name || !game.type || !game.description || !game.icon) {
    throw new Error(`${label} is missing required display metadata`);
  }
  if (!game.rules) {
    throw new Error(`${label} is missing playable rules`);
  }
  ['mode', 'target', 'hazard', 'objective'].forEach((field) => {
    if (typeof game.rules[field] !== 'string' || game.rules[field].trim() === '') {
      throw new Error(`${label} is missing rules.${field}`);
    }
  });
  ['winScore', 'lives', 'timeLimit', 'speed'].forEach((field) => {
    if (!Number.isFinite(game.rules[field]) || game.rules[field] <= 0) {
      throw new Error(`${label} must have a positive numeric rules.${field}`);
    }
  });
  if (!supportedModes.has(game.rules.mode)) {
    throw new Error(`${label} uses unsupported mode ${game.rules.mode}`);
  }
});

console.log(`Validated ${games.length} games across ${gameTypes.length} categories.`);
