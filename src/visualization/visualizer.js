const canvas = document.getElementById("canvas");
const ctx = canvas.getContext("2d");

let viewX = 0,
  viewY = 0;
let scale = 5; // pixels per physis unit

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

function drawParticle(x, y) {
  if (!Number.isFinite(x) || !Number.isFinite(y)) {
    return;
  }
  const screen = worldToScreen(x, y);
  ctx.beginPath();
  ctx.arc(screen.x, screen.y, 3, 0, Math.PI * 2);
  ctx.fillStyle = "#4CAF50";
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

  // draw springs for each softbody
  state.rects.forEach((rect) => {
    const startIdx = rect.start_idx;
    const nx = rect.nx;
    const ny = rect.ny;

    for (let i = 0; i < nx; i++) {
      for (let j = 0; j < ny; j++) {
        const idx = startIdx + i * ny + j;
        const p = state.particles[idx];

        // structural springs
        if (j + 1 < ny) {
          const next = state.particles[idx + 1];
          drawSpring(p, next);
        }
        if (i + 1 < nx) {
          const next = state.particles[idx + ny];
          drawSpring(p, next);
        }
        // shear springs
        if (i + 1 < nx && j + 1 < ny) {
          const next = state.particles[idx + ny + 1];
          drawSpring(p, next);
        }
        if (i + 1 < nx && j > 0) {
          const next = state.particles[idx + ny - 1];
          drawSpring(p, next);
        }
      }
    }
  });

  // draw particles on top
  state.particles.forEach((p) => drawParticle(p.x, p.y));
}

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
