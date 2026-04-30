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
const gameTypes = Array.from(new Set(games.map((game) => game[1])));
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

console.log(`Validated ${games.length} games across ${gameTypes.length} categories.`);
