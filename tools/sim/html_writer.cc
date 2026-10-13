#include "html_writer.h"

#include <fstream>
#include <iostream>
#include <sstream>

namespace sim {

namespace {

using micromouse::Direction;
using micromouse::directionLetter;  // navigation/pose.h

std::string jsonEscape(const std::string& s) {
  std::string out;
  out.reserve(s.size());
  for (const char c : s) {
    // '/' is also escaped so an embedded string (e.g. a maze path) can
    // never spell "</script" and close the surrounding <script> block.
    if (c == '"' || c == '\\' || c == '/') out += '\\';
    out += c;
  }
  return out;
}

std::string wallCodesToJson(const std::vector<uint8_t>& codes) {
  std::ostringstream os;
  os << '[';
  for (size_t i = 0; i < codes.size(); ++i) {
    if (i) os << ',';
    os << static_cast<int>(codes[i]);
  }
  os << ']';
  return os.str();
}

std::string floodToJson(const std::vector<int16_t>& flood) {
  std::ostringstream os;
  os << '[';
  for (size_t i = 0; i < flood.size(); ++i) {
    if (i) os << ',';
    os << flood[i];
  }
  os << ']';
  return os.str();
}

std::string stepToJson(const TraceStep& st) {
  std::ostringstream os;
  os << "{\"state\":\"" << st.state << "\",\"r\":"
     << static_cast<int>(st.cell.row) << ",\"c\":"
     << static_cast<int>(st.cell.col) << ",\"h\":\""
     << directionLetter(st.heading) << "\",\"moves\":" << st.moves
     << ",\"planned\":"
     << (st.hasPlanned ? "\"" + std::string(1, directionLetter(st.planned)) +
                             "\""
                       : "null")
     << ",\"walls\":" << wallCodesToJson(st.wallCodes)
     << ",\"flood\":" << floodToJson(st.flood) << ",\"fault\":"
     << (st.hasFault ? "{\"code\":" + std::to_string(st.faultCode) +
                           ",\"name\":\"" + jsonEscape(st.faultName) + "\"}"
                     : "null")
     << "}";
  return os.str();
}

std::string dataToJson(const SimData& d) {
  std::ostringstream os;
  os << "{\"rows\":" << static_cast<int>(d.rows)
     << ",\"cols\":" << static_cast<int>(d.cols) << ",\"start\":{\"r\":"
     << static_cast<int>(d.start.row) << ",\"c\":"
     << static_cast<int>(d.start.col) << "},\"goal\":{\"r\":"
     << static_cast<int>(d.goal.row) << ",\"c\":"
     << static_cast<int>(d.goal.col)
     << "},\"trueWalls\":" << wallCodesToJson(d.trueWalls) << ",\"steps\":[";
  for (size_t i = 0; i < d.steps.size(); ++i) {
    if (i) os << ',';
    os << stepToJson(d.steps[i]);
  }
  os << "],\"meta\":{\"mazeFile\":\"" << jsonEscape(d.mazeFile)
     << "\",\"noise\":" << d.noise << ",\"seed\":" << d.seed
     << ",\"deadSensor\":\"" << jsonEscape(d.deadSensor)
     << "\",\"outcome\":\"" << jsonEscape(d.outcomeText) << "\"}}";
  return os.str();
}

// clang-format off
constexpr char kHead[] = R"HTML(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>Micromouse simulation</title>
<style>
  body { font-family: sans-serif; background: #fafafa; color: #222; margin: 16px; }
  #meta { font-size: 13px; color: #555; margin-bottom: 8px; }
  #wrap { display: flex; gap: 20px; align-items: flex-start; flex-wrap: wrap; }
  canvas { border: 1px solid #999; background: #fff; }
  .side { min-width: 260px; }
  #status { font-size: 14px; line-height: 1.6; background: #fff; border: 1px solid #ddd;
            border-radius: 6px; padding: 10px; margin-bottom: 10px; white-space: pre-line; }
  .controls button { font-size: 14px; padding: 4px 10px; margin-right: 6px; }
  .controls input[type=range] { width: 200px; vertical-align: middle; }
  .row { margin-bottom: 8px; }
  .legend { font-size: 12px; color: #444; margin-top: 10px; }
  .legend span { display: inline-block; width: 14px; height: 3px; margin-right: 4px; vertical-align: middle; }
</style>
</head>
<body>
<h2>Micromouse exploration - simulation</h2>
<div id="meta"></div>
<div id="wrap">
  <canvas id="c"></canvas>
  <div class="side">
    <div id="status"></div>
    <div class="controls">
      <div class="row">
        <button id="playBtn">Play</button>
        <button id="prevBtn">&lt; Step</button>
        <button id="nextBtn">Step &gt;</button>
      </div>
      <div class="row">speed <input id="speed" type="range" min="40" max="1000" value="250"></div>
      <div class="row">
        step <span id="stepLabel">0</span> / <span id="stepTotal">0</span><br>
        <input id="scrub" type="range" min="0" max="0" value="0">
      </div>
    </div>
    <div class="legend">
      <div><span style="background:#bbb"></span> true wall (not yet discovered)</div>
      <div><span style="background:#111"></span> discovered wall</div>
      <div><span style="background:#4a90d9;height:14px;border-radius:7px;width:6px"></span> known-open side</div>
    </div>
  </div>
</div>
<script>
const DATA = )HTML";

constexpr char kTail[] = R"HTML(;

const cellPx = 48, margin = 26;
const rows = DATA.rows, cols = DATA.cols;
const canvas = document.getElementById('c');
canvas.width = cols * cellPx + margin * 2;
canvas.height = rows * cellPx + margin * 2;
const ctx = canvas.getContext('2d');

document.getElementById('meta').textContent =
  'maze: ' + DATA.meta.mazeFile + '  |  noise=' + DATA.meta.noise +
  '  seed=' + DATA.meta.seed +
  (DATA.meta.deadSensor ? '  dead-sensor=' + DATA.meta.deadSensor : '') +
  '  |  ' + rows + 'x' + cols;

function cellX(c) { return margin + c * cellPx; }
function cellY(r) { return margin + r * cellPx; }
// Up, Right, Down, Left, matching the C++ side encoding.
const DIRS = ['U', 'R', 'D', 'L'];

function edge(r, c, dirIdx) {
  const x = cellX(c), y = cellY(r);
  switch (DIRS[dirIdx]) {
    case 'U': return [x, y, x + cellPx, y];
    case 'R': return [x + cellPx, y, x + cellPx, y + cellPx];
    case 'D': return [x, y + cellPx, x + cellPx, y + cellPx];
    default:  return [x, y, x, y + cellPx];
  }
}

function idx(r, c, dirIdx) { return (r * cols + c) * 4 + dirIdx; }

function drawLine(x1, y1, x2, y2, color, width) {
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.lineCap = 'square';
  ctx.beginPath();
  ctx.moveTo(x1, y1);
  ctx.lineTo(x2, y2);
  ctx.stroke();
}

function trailUpTo(i) {
  const seen = [];
  const key = (r, c) => r + ',' + c;
  const set = new Set();
  set.add(key(DATA.start.r, DATA.start.c));
  seen.push([DATA.start.r, DATA.start.c]);
  for (let j = 0; j <= i; ++j) {
    const s = DATA.steps[j];
    if (s.state === 'MOVE' || s.state === 'GOAL_REACHED') {
      const k = key(s.r, s.c);
      if (!set.has(k)) { set.add(k); seen.push([s.r, s.c]); }
    }
  }
  return seen;
}

function draw(i) {
  const step = DATA.steps[i];
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  // Start / goal highlight.
  ctx.fillStyle = '#e4f7e4';
  ctx.fillRect(cellX(DATA.start.c), cellY(DATA.start.r), cellPx, cellPx);
  ctx.fillStyle = '#fff2cc';
  ctx.fillRect(cellX(DATA.goal.c), cellY(DATA.goal.r), cellPx, cellPx);

  // True walls (background, light grey) for every side.
  for (let r = 0; r < rows; ++r) {
    for (let c = 0; c < cols; ++c) {
      for (let d = 0; d < 4; ++d) {
        if (DATA.trueWalls[idx(r, c, d)] === 2) {
          const [x1, y1, x2, y2] = edge(r, c, d);
          drawLine(x1, y1, x2, y2, '#bbb', 3);
        }
      }
    }
  }

  // Trail of visited cells.
  const trail = trailUpTo(i);
  ctx.fillStyle = 'rgba(74,144,217,0.25)';
  for (const [r, c] of trail) {
    ctx.beginPath();
    ctx.arc(cellX(c) + cellPx / 2, cellY(r) + cellPx / 2, 5, 0, Math.PI * 2);
    ctx.fill();
  }

  // Discovered walls / known-open marks, and the flood distance numbers.
  ctx.font = '13px sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  for (let r = 0; r < rows; ++r) {
    for (let c = 0; c < cols; ++c) {
      for (let d = 0; d < 4; ++d) {
        const code = step.walls[idx(r, c, d)];
        const [x1, y1, x2, y2] = edge(r, c, d);
        if (code === 2) {
          drawLine(x1, y1, x2, y2, '#111', 3.5);
        } else if (code === 1) {
          ctx.fillStyle = '#4a90d9';
          ctx.beginPath();
          ctx.arc((x1 + x2) / 2, (y1 + y2) / 2, 2.5, 0, Math.PI * 2);
          ctx.fill();
        }
      }
      const dist = step.flood[r * cols + c];
      if (dist >= 0 && !(r === step.r && c === step.c)) {
        ctx.fillStyle = '#888';
        ctx.fillText(String(dist), cellX(c) + cellPx / 2, cellY(r) + cellPx / 2);
      }
    }
  }

  // Start / goal labels.
  ctx.fillStyle = '#2a7a2a';
  ctx.fillText('S', cellX(DATA.start.c) + 10, cellY(DATA.start.r) + 10);
  ctx.fillStyle = '#9a7a00';
  ctx.fillText('G', cellX(DATA.goal.c) + 10, cellY(DATA.goal.r) + 10);

  // Robot: a triangle pointing in the heading direction.
  const cx = cellX(step.c) + cellPx / 2, cy = cellY(step.r) + cellPx / 2;
  const rot = { U: 0, R: Math.PI / 2, D: Math.PI, L: -Math.PI / 2 }[step.h];
  ctx.save();
  ctx.translate(cx, cy);
  ctx.rotate(rot);
  ctx.fillStyle = step.state === 'FAULT' ? '#c0392b' : '#222';
  ctx.beginPath();
  ctx.moveTo(0, -14);
  ctx.lineTo(10, 10);
  ctx.lineTo(-10, 10);
  ctx.closePath();
  ctx.fill();
  ctx.restore();

  updateStatus(i);
}

function updateStatus(i) {
  const step = DATA.steps[i];
  const lines = [];
  lines.push('step ' + i + ' / ' + (DATA.steps.length - 1));
  lines.push('state: ' + step.state);
  lines.push('pose: (' + step.r + ',' + step.c + ') facing ' + step.h);
  lines.push('planned direction: ' + (step.planned || '-'));
  lines.push('moves so far: ' + step.moves);
  if (step.fault) {
    lines.push('OUTCOME: FAULT ' + step.fault.name + ' (code ' + step.fault.code + ')');
  } else if (step.state === 'GOAL_REACHED') {
    lines.push('OUTCOME: GOAL_REACHED after ' + step.moves + ' moves');
  } else {
    lines.push('outcome: (running)');
  }
  document.getElementById('status').textContent = lines.join('\n');
  document.getElementById('stepLabel').textContent = i;
  document.getElementById('scrub').value = i;
}

let cur = 0, playing = false, timer = null;
const total = DATA.steps.length;
document.getElementById('stepTotal').textContent = total - 1;
document.getElementById('scrub').max = total - 1;

function show(i) {
  cur = Math.max(0, Math.min(total - 1, i));
  draw(cur);
}
function stop() {
  playing = false;
  document.getElementById('playBtn').textContent = 'Play';
  if (timer) { clearInterval(timer); timer = null; }
}
function play() {
  if (cur >= total - 1) cur = 0;
  playing = true;
  document.getElementById('playBtn').textContent = 'Pause';
  const speed = () => 1100 - Number(document.getElementById('speed').value);
  const tick = () => {
    if (cur >= total - 1) { stop(); return; }
    show(cur + 1);
  };
  timer = setInterval(tick, speed());
}
document.getElementById('playBtn').onclick = () => { playing ? stop() : play(); };
document.getElementById('nextBtn').onclick = () => { stop(); show(cur + 1); };
document.getElementById('prevBtn').onclick = () => { stop(); show(cur - 1); };
document.getElementById('scrub').oninput = (e) => { stop(); show(Number(e.target.value)); };
document.getElementById('speed').onchange = () => { if (playing) { stop(); play(); } };

show(0);
</script>
</body>
</html>
)HTML";
// clang-format on

}  // namespace

bool writeHtml(const std::string& path, const SimData& data) {
  std::ofstream out(path, std::ios::binary);
  if (!out) {
    std::cerr << "error: cannot write " << path << "\n";
    return false;
  }
  out << kHead << dataToJson(data) << kTail;
  return static_cast<bool>(out);
}

}  // namespace sim
