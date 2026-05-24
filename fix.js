const fs = require('fs');

let content = fs.readFileSync('multiTargetAStar.js', 'utf8');

// Fix 1: Initialize start mask correctly if it's on a target
content = content.replace(
    /let startState = \{ x: start\.x, y: start\.y, mask: 0 \};\s*let startKey = `\$\{start\.x\},\$\{start\.y\},0`;/,
    `let targetMap = new Map();
    targets.forEach((t, index) => {
        let key = \`\$\{t.x\},\$\{t.y\}\`;
        if (!targetMap.has(key)) {
            targetMap.set(key, index);
        }
    });

    let initialMask = 0;
    let startCoordKey = \`\$\{start.x\},\$\{start.y\}\`;
    if (targetMap.has(startCoordKey)) {
        initialMask |= (1 << targetMap.get(startCoordKey));
    }

    let startState = { x: start.x, y: start.y, mask: initialMask };
    let startKey = \`\$\{start.x\},\$\{start.y\},\$\{initialMask\}\`;`
);

// Fix 2: Remove the old targetMap initialization since we moved it up
content = content.replace(
    /\/\/ 预处理目标的坐标映射到其掩码位\s*let targetMap = new Map\(\);\s*targets\.forEach\(\(t, index\) => \{\s*targetMap\.set\(`\$\{t\.x\},\$\{t\.y\}`\, index\);\s*\}\);/,
    `// 预处理目标的坐标映射到其掩码位已在上方完成`
);

fs.writeFileSync('multiTargetAStar.js', content, 'utf8');
