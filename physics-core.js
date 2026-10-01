/*
 * 蓋橋遊戲 共用物理核心 (classic / mobile / sandbox 共用)
 * 方法：Verlet 位置預測 + XPBD 距離約束；桿件內力由約束乘數 λ 回推。
 * 用法：BridgePhysics.advance(nodes, members, { settleOnly, falling, load })
 */
const BridgePhysics = (() => {
  const CFG = {
    SOLVER_SUBSTEPS: 12,
    SOLVER_ITERATIONS: 2,
    VELOCITY_RETENTION: 0.9985,
    SETTLE_RETENTION: 0.965,
    FREEFALL_RETENTION: 0.9998,
    FREEFALL_GRAVITY_SCALE: 2.2,
    MAX_STEP_SPEED: 1.5,
    MAX_FREEFALL_STEP_SPEED: 2.6,
    SETTLE_TICKS: 150,
    FORCE_SMOOTHING: 0.4,
    OVERLOAD_GRACE_TICKS: 3,
    GRAVITY_ACCEL: 200,
    MEMBER_COMPLIANCE: 1 / 14500,
    LIVE_LOAD_FACTOR: 420,
    TENSILE_LIMIT: 7200,
    ROAD_TENSILE_LIMIT: 7800,
    COMPRESSION_BASE_CAPACITY: 13000,
    BUCKLING_LENGTH_SCALE: 75,
    FIXED_DELTA: 1 / 60
  };

  const compressionLimit = (len) => {
    const s = len / CFG.BUCKLING_LENGTH_SCALE;
    return CFG.COMPRESSION_BASE_CAPACITY / (1 + s * s);
  };

  const payloadFor = (weightTons) => CFG.LIVE_LOAD_FACTOR * weightTons;

  /**
   * 推進一個固定時間步。
   * opts.settleOnly : 僅預沉降（強阻尼、無載重、不判定斷裂）
   * opts.falling    : 卡車已墜落（較大重力、較低阻尼）
   * opts.load       : { left, right, frac, payload } 車輪載重分配到路段兩端節點
   * 回傳 { peakRatio, weakest }：最大應力比與應斷裂的桿件（呼叫端決定是否真的斷）
   */
  function advance(nodes, members, opts = {}) {
    const settleOnly = !!opts.settleOnly;
    const falling = !settleOnly && !!opts.falling;
    const load = settleOnly ? null : (opts.load || null);
    const h = CFG.FIXED_DELTA / CFG.SOLVER_SUBSTEPS;
    const retain = settleOnly ? CFG.SETTLE_RETENTION : (falling ? CFG.FREEFALL_RETENTION : CFG.VELOCITY_RETENTION);
    const speedCap = falling ? CFG.MAX_FREEFALL_STEP_SPEED : CFG.MAX_STEP_SPEED;
    const g = falling ? CFG.GRAVITY_ACCEL * CFG.FREEFALL_GRAVITY_SCALE : CFG.GRAVITY_ACCEL;

    const links = [];
    const liveIds = new Set();
    for (const m of members.values()) {
      if (m.ruptured) continue;
      links.push(m);
      liveIds.add(m.n1);
      liveIds.add(m.n2);
    }
    const movers = [];
    for (const id of liveIds) {
      const n = nodes.get(id);
      if (!n.isFixed) movers.push(n);
    }

    const compliance = CFG.MEMBER_COMPLIANCE / (h * h);
    for (const m of links) m.lambdaSum = 0;

    for (let step = 0; step < CFG.SOLVER_SUBSTEPS; step++) {
      for (const n of movers) {
        let fy = g * n.mass;
        if (load) {
          if (n === load.left) fy += load.payload * (1 - load.frac);
          if (n === load.right) fy += load.payload * load.frac;
        }
        let vx = (n.x - n.px) * retain;
        let vy = (n.y - n.py) * retain;
        const sp = Math.hypot(vx, vy);
        if (sp > speedCap) { vx *= speedCap / sp; vy *= speedCap / sp; }
        n.px = n.x; n.py = n.y;
        n.x += vx;
        n.y += vy + (fy / n.mass) * h * h;
      }

      for (const m of links) m.lambda = 0;
      for (let it = 0; it < CFG.SOLVER_ITERATIONS; it++) {
        for (const m of links) {
          const a = nodes.get(m.n1);
          const b = nodes.get(m.n2);
          const wa = a.isFixed ? 0 : 1 / a.mass;
          const wb = b.isFixed ? 0 : 1 / b.mass;
          const dx = a.x - b.x;
          const dy = a.y - b.y;
          const dist = Math.hypot(dx, dy) || 1e-6;
          const dLambda = (-(dist - m.restLength) - compliance * m.lambda) / (wa + wb + compliance);
          m.lambda += dLambda;
          const ux = dx / dist, uy = dy / dist;
          a.x += wa * dLambda * ux; a.y += wa * dLambda * uy;
          b.x -= wb * dLambda * ux; b.y -= wb * dLambda * uy;
        }
      }
      for (const m of links) m.lambdaSum += m.lambda;
    }

    // 軸力 = -λ/h²（拉力為正），再做時間平滑避免瞬間尖峰誤判
    for (const m of links) {
      const instant = -(m.lambdaSum / CFG.SOLVER_SUBSTEPS) / (h * h);
      m.axialForce += (instant - m.axialForce) * CFG.FORCE_SMOOTHING;
    }
    if (settleOnly) return { peakRatio: 0, weakest: null };

    let peakRatio = 0;
    let weakest = null;
    let weakestRatio = 1.0;
    for (const m of links) {
      const cap = (m.axialForce >= 0)
        ? (m.isRoad ? CFG.ROAD_TENSILE_LIMIT : CFG.TENSILE_LIMIT)
        : compressionLimit(m.restLength);
      m.stressRatio = Math.abs(m.axialForce) / cap;
      if (m.stressRatio > peakRatio) peakRatio = m.stressRatio;
      m.overTicks = (m.stressRatio > 1.0) ? (m.overTicks || 0) + 1 : 0;
      if (m.overTicks >= CFG.OVERLOAD_GRACE_TICKS && m.stressRatio > weakestRatio) {
        weakest = m;
        weakestRatio = m.stressRatio;
      }
    }
    return { peakRatio, weakest };
  }

  /** 啟動前先讓結構在重力下沉降到平衡，再清空內力狀態 */
  function presettle(nodes, members) {
    for (let i = 0; i < CFG.SETTLE_TICKS; i++) advance(nodes, members, { settleOnly: true });
    for (const n of nodes.values()) { n.px = n.x; n.py = n.y; }
    for (const m of members.values()) { m.axialForce = 0; m.stressRatio = 0; m.overTicks = 0; }
  }

  return { CFG, advance, presettle, payloadFor, compressionLimit };
})();

if (typeof module !== 'undefined' && module.exports) module.exports = BridgePhysics;
