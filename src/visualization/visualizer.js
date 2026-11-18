const canvas = document.getElementById("canvas");
const ctx = canvas.getContext("2d");

let viewX = 0,
  viewY = 0;
let scale = 5; // pixels per physics unit

function resizeCanvas() {
  canvas.width = window.innerWidth;
  canvas.height = window.innerHeight;
}

window.addEventListener("resize", resizeCanvas);
window.addEventListener("keydown", (e) => {
  if (e.key === "r") {
    viewX = 0;
    viewY = 0;
    scale = 5;
  }
  if (e.key === "-") {
    scale *= 0.8;
  }
  if (e.key === "=") {
    scale *= 1.25;
  }
});
resizeCanvas();

function worldToScreen(x, y) {
  return {
    x: (x - viewX) * scale + canvas.width / 2,
    y: canvas.height / 2 - (y - viewY) * scale, // flip y-axis
  };
}

function drawParticle(x, y, boundary = false) {
  if (!Number.isFinite(x) || !Number.isFinite(y)) {
    return;
  }
  const screen = worldToScreen(x, y);
  ctx.beginPath();
  ctx.arc(screen.x, screen.y, 3, 0, Math.PI * 2);
  ctx.fillStyle = boundary ? "#E53935" : "#4CAF50";
  ctx.fill();
}

function drawSpring(p1, p2) {
  if (
    !Number.isFinite(p1.x) ||
    !Number.isFinite(p1.y) ||
    !Number.isFinite(p2.x) ||
    !Number.isFinite(p2.y)
  ) {
    return;
  }
  const s1 = worldToScreen(p1.x, p1.y);
  const s2 = worldToScreen(p2.x, p2.y);
  ctx.beginPath();
  ctx.moveTo(s1.x, s1.y);
  ctx.lineTo(s2.x, s2.y);
  ctx.strokeStyle = "#333";
  ctx.lineWidth = 1;
  ctx.stroke();
}

function render(state) {
  document.getElementById("info").innerHTML = `
    <div>FPS: <span id="fps">${fps}</span></div>
    <div>Particles: ${state.particles.length}</div>
  `;

  // clear canvas
  ctx.fillStyle = "#0a0a0a";
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  // draw world bounds
  const bounds = { min: { x: -50, y: -50 }, max: { x: 50, y: 50 } };
  const tl = worldToScreen(bounds.min.x, bounds.max.y);
  const br = worldToScreen(bounds.max.x, bounds.min.y);
  ctx.strokeStyle = "#444";
  ctx.lineWidth = 2;
  ctx.strokeRect(tl.x, tl.y, br.x - tl.x, br.y - tl.y);

  const boundaryPoints = new Set();
  state.bodies.forEach((body) => {
    body.springs.forEach(([a, b]) =>
      drawSpring(state.particles[a], state.particles[b]),
    );
    body.boundary.forEach((p) => {
      boundaryPoints.add(p);
      drawParticle(state.particles[p].x, state.particles[p].y, true);
    });
  });
  state.particles.forEach((p, i) => {
    if (!boundaryPoints.has(i)) {
      drawParticle(p.x, p.y);
    }
  });
}

const shapeSelect = document.getElementById("shape");
function syncFields() {
  const isRect = shapeSelect.value === "rect";
  document
    .querySelectorAll(".rect-only")
    .forEach((e) => (e.style.display = isRect ? "" : "none"));
  document
    .querySelectorAll(".circ-only")
    .forEach((e) => (e.style.display = isRect ? "none" : ""));
}
shapeSelect.addEventListener("change", syncFields);
syncFields();

const num = (id) => parseFloat(document.getElementById(id).value);

document.getElementById("spawnBtn").addEventListener("click", async () => {
  const type = shapeSelect.value;
  const body =
    type === "rect"
      ? {
          type,
          x: num("f-x"),
          y: num("f-y"),
          w: num("f-w"),
          h: num("f-h"),
          mass: num("f-mass"),
          restitution: num("f-rest"),
          subdiv: num("f-subdiv"),
          angleDeg: num("f-angle"),
        }
      : {
          type,
          x: num("f-x"),
          y: num("f-y"),
          radius: num("f-radius"),
          mass: num("f-mass"),
          restitution: num("f-rest"),
          num: num("f-num"),
        };
  try {
    const r = await fetch("/spawn", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    });
    if (!r.ok) console.error("spawn rejected:", r.status);
  } catch (e) {
    console.error("spawn failed:", e);
  }
});

let lastFrameTime = Date.now();
let fps = 0;

async function pollState() {
  try {
    const response = await fetch("physics_state.json?t=" + Date.now(), {
      cache: "no-store",
      headers: {
        "Cache-Control": "no-cache",
      },
    });
    const state = await response.json();

    render(state);

    const now = Date.now();
    fps = Math.round(1000 / (now - lastFrameTime));
    lastFrameTime = now;
    document.getElementById("fps").textContent = fps;
  } catch (e) {
    console.error("failed to fetch state:", e);
  }

  requestAnimationFrame(pollState);
}

pollState();
