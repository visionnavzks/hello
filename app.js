const games = [
  ['星际冲刺', '动作', '驾驶飞船穿越流星雨，挑战极限反应。', '🚀'],
  ['像素迷宫', '益智', '旋转房间、寻找钥匙，逃出复古迷宫。', '🧩'],
  ['霓虹赛车', '竞速', '在赛博城市霓虹赛道上漂移冲线。', '🏎️'],
  ['海岛探险', '冒险', '探索神秘岛屿，收集宝藏与线索。', '🏝️'],
  ['水果连连', '休闲', '连接相同水果，创造爽快连击。', '🍓'],
  ['方块塔防', '策略', '布置炮塔，守住你的水晶核心。', '🛡️'],
  ['泡泡射手', '休闲', '瞄准彩色泡泡，清空整个天空。', '🫧'],
  ['忍者跳跃', '动作', '墙跳、闪避飞镖，抵达屋顶终点。', '🥷'],
  ['森林拼图', '益智', '拼合森林碎片，唤醒沉睡的精灵。', '🌲'],
  ['极速雪橇', '竞速', '穿越雪山弯道，躲避冰柱障碍。', '🛷'],
  ['月球矿工', '模拟', '经营月球矿场，升级采集机器人。', '🌙'],
  ['魔法棋盘', '策略', '用法术棋子击败对手的王冠。', '♟️'],
  ['深海寻宝', '冒险', '驾驶潜艇寻找失落文明宝箱。', '🤿'],
  ['节奏鼓手', '音乐', '跟随节拍敲击鼓点，点燃舞台。', '🥁'],
  ['猫咪咖啡', '模拟', '经营温馨猫咖，招待不同客人。', '🐱'],
  ['糖果工厂', '休闲', '组合糖果流水线，生产甜蜜订单。', '🍬'],
  ['太空农场', '模拟', '在轨道温室种植外星作物。', '🪴'],
  ['龙之山谷', '冒险', '训练小龙，穿越火焰山谷。', '🐉'],
  ['弹珠大师', '休闲', '发射弹珠，击中机关获取高分。', '🔮'],
  ['机器人大乱斗', '动作', '组装机器人，在竞技场对战。', '🤖'],
  ['滑板街区', '竞速', '完成花式动作，征服城市街区。', '🛹'],
  ['宝石消消', '益智', '交换宝石，触发华丽连锁爆炸。', '💎'],
  ['幽灵旅馆', '冒险', '调查奇怪旅馆里的幽灵谜案。', '👻'],
  ['迷你高尔夫', '休闲', '掌握角度和力度，一杆进洞。', '⛳'],
  ['银河防线', '策略', '指挥舰队阻挡外星入侵。', '🛰️'],
  ['火山跑酷', '动作', '越过岩浆裂缝，逃离喷发火山。', '🌋'],
  ['厨房快手', '模拟', '快速备餐，满足络绎不绝的订单。', '🍳'],
  ['纸牌王国', '策略', '用卡牌建造王国并击退怪物。', '🃏'],
  ['雪人保龄', '休闲', '滚动雪球，击倒可爱的雪人瓶。', '🎳'],
  ['极地钓鱼', '模拟', '寻找最佳冰洞，钓起稀有鱼类。', '🎣'],
  ['沙漠越野', '竞速', '驾驶越野车冲过沙丘和峡谷。', '🚙'],
  ['汉字侦探', '益智', '根据线索拆字解谜，破解案件。', '🔎'],
  ['勇者地牢', '冒险', '深入地牢，收集装备挑战魔王。', '⚔️'],
  ['飞盘狗狗', '休闲', '投掷飞盘，让狗狗完成空中接力。', '🐶'],
  ['云端滑翔', '竞速', '乘滑翔翼穿过云层计时赛道。', '🪂'],
  ['时间旅者', '冒险', '穿梭不同年代，修复错乱时间线。', '⏳'],
  ['数字黑洞', '益智', '合并数字，制造越来越大的黑洞。', '🕳️'],
  ['星球绘师', '创意', '用色彩和地形设计专属星球。', '🎨'],
  ['怪兽牧场', '模拟', '孵化怪兽，训练并参加友谊赛。', '🥚'],
  ['空中花园', '休闲', '布置漂浮花园，让花朵持续盛开。', '🌷'],
  ['激光谜阵', '益智', '调整镜面，让激光点亮全部节点。', '🔦'],
  ['机甲竞速', '竞速', '驾驶机甲穿越未来工业赛道。', '🦾'],
  ['古堡守卫', '策略', '部署弓箭手与骑士保卫古堡。', '🏰'],
  ['蜜蜂快递', '动作', '帮助蜜蜂穿越花园完成配送。', '🐝'],
  ['梦境画廊', '创意', '收集梦境碎片，点亮互动展厅。', '🖼️'],
  ['行星弹球', '休闲', '利用引力弹射小球命中行星。', '🪐'],
  ['雪域救援', '冒险', '驾驶救援车寻找迷路登山者。', '🚑'],
  ['代码骑士', '益智', '排列指令卡，让骑士自动闯关。', '💻'],
  ['迷你足球', '动作', '三分钟快节奏足球对决。', '⚽'],
  ['彩虹列车', '模拟', '规划轨道，连接五彩小镇。', '🚂'],
];

const gradients = [
  ['#60a5fa', '#a78bfa'],
  ['#fb7185', '#f97316'],
  ['#34d399', '#06b6d4'],
  ['#facc15', '#f472b6'],
  ['#c084fc', '#22d3ee'],
];

const grid = document.querySelector('#game-grid');
const search = document.querySelector('#search');
const detail = document.querySelector('#game-detail');
const closeDetail = document.querySelector('#close-detail');
const detailTitle = document.querySelector('#detail-title');
const detailIcon = document.querySelector('#detail-icon');
const detailType = document.querySelector('#detail-type');
const detailDescription = document.querySelector('#detail-description');
const filterButtons = document.querySelectorAll('.dock button');
let activeFilter = '全部';

function renderGames() {
  const term = search.value.trim().toLowerCase();
  const filteredGames = games.filter(([name, type, description]) => {
    const matchesFilter = activeFilter === '全部' || type === activeFilter;
    const matchesSearch = [name, type, description].join(' ').toLowerCase().includes(term);
    return matchesFilter && matchesSearch;
  });

  grid.innerHTML = '';

  filteredGames.forEach((game, index) => {
    const [name, type, description, icon] = game;
    const [c1, c2] = gradients[index % gradients.length];
    const button = document.createElement('button');
    button.className = 'game-card';
    button.type = 'button';
    button.style.setProperty('--c1', c1);
    button.style.setProperty('--c2', c2);
    button.innerHTML = `
      <span class="game-icon" aria-hidden="true">${icon}</span>
      <strong>${name}</strong>
      <span>${type}</span>
    `;
    button.addEventListener('click', () => showDetail({ name, type, description, icon, c1, c2 }));
    grid.appendChild(button);
  });

  if (filteredGames.length === 0) {
    grid.innerHTML = '<p class="empty">没有找到匹配的游戏。</p>';
  }
}

function showDetail(game) {
  detail.hidden = false;
  detailTitle.textContent = game.name;
  detailIcon.textContent = game.icon;
  detailIcon.style.setProperty('--c1', game.c1);
  detailIcon.style.setProperty('--c2', game.c2);
  detailType.textContent = `${game.type} 游戏`;
  detailDescription.textContent = game.description;
}

function updateClock() {
  document.querySelector('#clock').textContent = new Intl.DateTimeFormat('zh-CN', {
    hour: '2-digit',
    minute: '2-digit',
    weekday: 'short',
  }).format(new Date());
}

search.addEventListener('input', renderGames);
closeDetail.addEventListener('click', () => {
  detail.hidden = true;
});

filterButtons.forEach((button) => {
  button.addEventListener('click', () => {
    activeFilter = button.dataset.filter;
    filterButtons.forEach((item) => item.classList.toggle('active', item === button));
    renderGames();
  });
});

filterButtons[0].classList.add('active');
renderGames();
updateClock();
setInterval(updateClock, 30000);
