/**
 * 优先队列（Priority Queue）实现，用于A*算法的Open List
 */
class PriorityQueue {
    constructor() {
        this.elements = [];
    }

    enqueue(element, priority) {
        // 使用二分查找插入以保持有序，提高性能
        let item = { element, priority };
        let low = 0, high = this.elements.length;
        while (low < high) {
            let mid = (low + high) >>> 1;
            if (this.elements[mid].priority < priority) {
                low = mid + 1;
            } else {
                high = mid;
            }
        }
        this.elements.splice(low, 0, item);
    }

    dequeue() {
        return this.elements.shift().element;
    }

    isEmpty() {
        return this.elements.length === 0;
    }
}

/**
 * 曼哈顿距离启发式函数
 */
function manhattan(p1, p2) {
    return Math.abs(p1.x - p2.x) + Math.abs(p1.y - p2.y);
}

/**
 * 检查坐标是否在网格内且不是障碍物
 */
function isValid(grid, x, y) {
    return x >= 0 && x < grid.length && y >= 0 && y < grid[0].length && grid[x][y] === 0;
}

/**
 * 场景1：寻找一组目标中距离最近的一个（到达任意一个目标即可）
 *
 * @param {Array<Array<number>>} grid - 二维网格，0表示可行走，1表示障碍物
 * @param {Object} start - 起点 {x, y}
 * @param {Array<Object>} targets - 目标点集合 [{x, y}, {x, y}, ...]
 * @returns {Array<Object>|null} - 返回到达最近目标的路径，或者 null 如果无解
 */
function findNearestTarget(grid, start, targets) {
    let pq = new PriorityQueue();
    let startKey = `${start.x},${start.y}`;

    pq.enqueue({ x: start.x, y: start.y }, 0);

    let cameFrom = new Map();
    let gScore = new Map();

    cameFrom.set(startKey, null);
    gScore.set(startKey, 0);

    let targetSet = new Set(targets.map(t => `${t.x},${t.y}`));

    const dirs = [[0, 1], [1, 0], [0, -1], [-1, 0]]; // 右，下，左，上

    while (!pq.isEmpty()) {
        let current = pq.dequeue();
        let currentKey = `${current.x},${current.y}`;

        // 如果到达任意一个目标
        if (targetSet.has(currentKey)) {
            // 回溯路径
            let path = [];
            let curr = currentKey;
            while (curr !== null) {
                let [x, y] = curr.split(',').map(Number);
                path.push({ x, y });
                curr = cameFrom.get(curr);
            }
            return path.reverse();
        }

        for (let [dx, dy] of dirs) {
            let nx = current.x + dx;
            let ny = current.y + dy;

            if (isValid(grid, nx, ny)) {
                let nextKey = `${nx},${ny}`;
                let tentativeG = gScore.get(currentKey) + 1;

                if (!gScore.has(nextKey) || tentativeG < gScore.get(nextKey)) {
                    gScore.set(nextKey, tentativeG);
                    cameFrom.set(nextKey, currentKey);

                    // 启发值：到所有目标中最小的曼哈顿距离
                    let h = Math.min(...targets.map(t => manhattan({x: nx, y: ny}, t)));
                    pq.enqueue({ x: nx, y: ny }, tentativeG + h);
                }
            }
        }
    }

    return null; // 未找到路径
}

/**
 * 场景2：寻找遍历一组所有目标的最短路径（TSP-like A*）
 * 状态空间包含了位置和已访问目标的掩码 (bitmask)
 *
 * @param {Array<Array<number>>} grid - 二维网格
 * @param {Object} start - 起点
 * @param {Array<Object>} targets - 需要全部遍历的目标点集合
 * @returns {Array<Object>|null} - 返回遍历所有目标的路径
 */
function findAllTargets(grid, start, targets) {
    if (targets.length > 31) {
        throw new Error("Target count exceeds bitmask capacity (max 31 targets).");
    }

    let pq = new PriorityQueue();
    let numTargets = targets.length;
    let allVisitedMask = (1 << numTargets) - 1;

    // 状态表示: {x, y, mask}
    // mask: bitmask，第i位为1表示第i个目标已访问
    let targetMap = new Map();
    targets.forEach((t, index) => {
        let key = `${t.x},${t.y}`;
        if (!targetMap.has(key)) {
            targetMap.set(key, index);
        }
    });

    let initialMask = 0;
    let startCoordKey = `${start.x},${start.y}`;
    if (targetMap.has(startCoordKey)) {
        initialMask |= (1 << targetMap.get(startCoordKey));
    }

    let startState = { x: start.x, y: start.y, mask: initialMask };
    let startKey = `${start.x},${start.y},${initialMask}`;

    pq.enqueue(startState, 0);

    let cameFrom = new Map();
    let gScore = new Map();

    cameFrom.set(startKey, null);
    gScore.set(startKey, 0);

    // 预处理目标的坐标映射到其掩码位已在上方完成

    const dirs = [[0, 1], [1, 0], [0, -1], [-1, 0]];

    while (!pq.isEmpty()) {
        let current = pq.dequeue();
        let currentKey = `${current.x},${current.y},${current.mask}`;

        // 如果所有目标都已访问
        if (current.mask === allVisitedMask) {
            let path = [];
            let currKey = currentKey;
            while (currKey !== null) {
                let parts = currKey.split(',');
                path.push({ x: Number(parts[0]), y: Number(parts[1]) });
                currKey = cameFrom.get(currKey);
            }
            return path.reverse();
        }

        for (let [dx, dy] of dirs) {
            let nx = current.x + dx;
            let ny = current.y + dy;

            if (isValid(grid, nx, ny)) {
                let nKeyStr = `${nx},${ny}`;
                let nextMask = current.mask;

                // 如果走到一个目标上，更新mask
                if (targetMap.has(nKeyStr)) {
                    let targetIndex = targetMap.get(nKeyStr);
                    nextMask |= (1 << targetIndex);
                }

                let nextStateKey = `${nx},${ny},${nextMask}`;
                let tentativeG = gScore.get(currentKey) + 1;

                if (!gScore.has(nextStateKey) || tentativeG < gScore.get(nextStateKey)) {
                    gScore.set(nextStateKey, tentativeG);
                    cameFrom.set(nextStateKey, currentKey);

                    // 启发式：计算到最远的未访问目标的曼哈顿距离
                    // 这是一个Admissible Heuristic，因为必定至少要走这么远才能到达最远的那个目标
                    let h = 0;
                    for (let i = 0; i < numTargets; i++) {
                        if ((nextMask & (1 << i)) === 0) {
                            let dist = manhattan({x: nx, y: ny}, targets[i]);
                            if (dist > h) h = dist;
                        }
                    }

                    pq.enqueue({ x: nx, y: ny, mask: nextMask }, tentativeG + h);
                }
            }
        }
    }

    return null;
}

// 导出模块
module.exports = {
    findNearestTarget,
    findAllTargets
};

// ==========================================
// 演示与测试代码 (如果直接运行此脚本)
// ==========================================
if (require.main === module) {
    const grid = [
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 1, 1, 1, 1, 0, 1, 1, 0],
        [0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
        [0, 1, 1, 1, 0, 1, 1, 1, 1, 0],
        [0, 1, 0, 0, 0, 0, 0, 0, 1, 0],
        [0, 1, 0, 1, 1, 1, 1, 0, 1, 0],
        [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
        [0, 1, 1, 1, 0, 1, 1, 1, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    ];

    const start = { x: 0, y: 0 };
    const targets = [
        { x: 2, y: 8 },
        { x: 8, y: 0 },
        { x: 8, y: 9 }
    ];

    function printGridWithPath(grid, start, targets, path) {
        let output = "";
        let pathSet = new Set(path.map(p => `${p.x},${p.y}`));
        let targetSet = new Set(targets.map(t => `${t.x},${t.y}`));

        for (let r = 0; r < grid.length; r++) {
            let rowStr = "";
            for (let c = 0; c < grid[0].length; c++) {
                let key = `${r},${c}`;
                if (start.x === r && start.y === c) {
                    rowStr += " S "; // Start
                } else if (targetSet.has(key)) {
                    rowStr += " T "; // Target
                } else if (pathSet.has(key)) {
                    rowStr += " * "; // Path
                } else if (grid[r][c] === 1) {
                    rowStr += "███"; // Obstacle
                } else {
                    rowStr += " . "; // Empty
                }
            }
            output += rowStr + "\n";
        }
        console.log(output);
    }

    console.log("==========================================");
    console.log("场景1：寻找距离最近的一个目标");
    console.log("==========================================");
    let path1 = findNearestTarget(grid, start, targets);
    if (path1) {
        console.log(`找到了路径！步数：${path1.length - 1}`);
        printGridWithPath(grid, start, targets, path1);
    } else {
        console.log("未找到路径。");
    }

    console.log("==========================================");
    console.log("场景2：寻找遍历所有目标的最短路径");
    console.log("==========================================");
    let path2 = findAllTargets(grid, start, targets);
    if (path2) {
        console.log(`找到了遍历所有目标的路径！步数：${path2.length - 1}`);
        printGridWithPath(grid, start, targets, path2);
    } else {
        console.log("未找到路径。");
    }
}
