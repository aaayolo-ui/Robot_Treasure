// Static audit of generated artifacts. Browser interaction is checked separately.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const base = path.resolve(__dirname, '..');
const read = name => JSON.parse(fs.readFileSync(path.join(base, name), 'utf8'));
const report = read('report.json'), annotations = read('map_annotations.json');
assert.deepEqual(report.annotations, annotations);
assert.deepEqual(report.map, read('../TreasureRoutePlanner/map.json'));
assert.deepEqual(report.map, read('qa/reference-map.json'));
const html = fs.readFileSync(path.join(base, 'replay.html'), 'utf8');
let scripts = 0;
for (const match of html.matchAll(/<script([^>]*)>([\s\S]*?)<\/script>/g)) {
  if (match[1].includes('application/json')) assert.deepEqual(JSON.parse(match[2]), report);
  else { new vm.Script(match[2]); scripts++; }
}
assert.ok(scripts > 0);
assert.equal(annotations.nodes.length, 88);
assert.equal(annotations.segments.length, 106);
assert.equal(Object.keys(annotations.original_node_mapping).length, 121);
const nodes = new Map(annotations.nodes.map(n => [n.id, n]));
const edgeKey = (a, b) => [a, b].sort().join('|');
const edges = new Set();
for (const segment of annotations.segments) {
  assert.equal(segment.calibrated, false);
  for (let i = 1; i < segment.original_nodes.length; i++) {
    const key = edgeKey(segment.original_nodes[i - 1], segment.original_nodes[i]);
    assert.ok(!edges.has(key), `Duplicate original edge ${key}`);
    edges.add(key);
  }
}
const separation = f => Math.hypot(f[2] - f[9], f[3] - f[10]) * report.map.grid_spacing_m;
const radius = report.config.capture_center_distance_m;
assert.equal(report.config.turn90_s, .5);
assert.equal(report.config.uturn_s, .5);
assert.equal(report.config.pickup_s, 0);
assert.equal(report.replays.length, 7);
assert.equal(report.replays.at(-1).thief_policy, 'refuge');
const cases = report.replays.map(match => {
  assert.ok(Math.abs(match.police_sensor_range_m - .9) < 1e-8);
  assert.ok(match.police_speed_m_s >= match.thief_speed_m_s);
  assert.equal(match.vision, 'limited');
  const contacts = match.frames.slice(0, -1).filter(f => separation(f) < radius - .0001);
  assert.equal(contacts.length, 0, `Continued contact: ${match.label}`);
  const firstDetection = match.events.find(e => e.role === 'P' && e.event === 'detected');
  const abandoned = firstDetection ? match.frames.filter(f => f[0] > firstDetection.t + .0001 && f[6] === 0).length : 0;
  assert.equal(abandoned, 0);
  for (const choice of match.hide_decisions) {
    const n = nodes.get(choice.node_id);
    assert.ok(n?.is_track_landmark);
    assert.equal(n.original_node, choice.node);
    assert.ok(['corner', 't_junction', 'cross'].includes(n.geometry_type));
    assert.ok(choice.escape_exits.length >= 2);
    assert.ok(choice.road_span_m <= .9 + 1e-8);
  }
  const hiding = match.frames.filter(f => f[12] === 5);
  for (const f of hiding) assert.ok(match.hide_decisions.some(d => d.t <= f[0] + .001 && d.node === `(${f[9]},${f[10]})`));
  for (const plan of match.escape_plans) {
    assert.equal(new Set(plan.nodes).size, plan.nodes.length, 'Repeated node within escape plan');
    for (let i = 1; i < plan.nodes.length; i++) assert.ok(edges.has(edgeKey(plan.nodes[i - 1], plan.nodes[i])));
  }
  const visits = [];
  for (const f of match.frames) {
    if (Math.abs(f[9] - Math.round(f[9])) > 1e-5 || Math.abs(f[10] - Math.round(f[10])) > 1e-5) continue;
    const id = `${f[9]},${f[10]}`;
    if (visits.at(-1)?.id !== id) visits.push({id, t:f[0]});
  }
  let pingPong = 0;
  for (let i = 3; i < visits.length; i++) {
    if (visits[i].t - visits[i-3].t <= 5 && visits[i].id === visits[i-2].id && visits[i-1].id === visits[i-3].id) pingPong++;
  }
  assert.equal(pingPong, 0, 'Sampled A-B-A-B within five seconds');
  if (match.end_reason.startsWith('capture_')) {
    assert.equal(match.winner, 'police');
    assert.ok(separation(match.frames.at(-1)) <= radius + .0001);
  }
  return {scenario:match.label, duration_s:match.duration_s, winner:match.winner,
    continued_contact_frames:contacts.length, pursuit_abandonments:abandoned,
    sampled_short_ping_pong:pingPong, hiding_frames_checked:hiding.length,
    hides_checked:match.hide_decisions.length, escape_plans_checked:match.escape_plans.length};
});
const audit = {nodes:88, segments:106, original_nodes:121, original_map_unchanged:true,
  embedded_export_matches:true, executable_scripts_parsed:scripts,
  note:'Seed 7 only. Sampled frame checks supplement continuous collision unit tests; they do not prove optimal play or physical performance.', cases};
fs.writeFileSync(path.join(__dirname, 'refuge-labels-audit.json'), JSON.stringify(audit, null, 2) + '\n');
console.log(JSON.stringify(audit, null, 2));
