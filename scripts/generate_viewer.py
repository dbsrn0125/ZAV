import numpy as np
import base64
import json
import os
import glob

scans = sorted(glob.glob('/root/zenith_ws/scans/corridor_*.pcd'))
pcd_path = scans[-1] if scans else '/root/zenith_ws/src/FAST-LIVO2/Log/PCD/all_downsampled_points.pcd'
traj_path = '/root/zenith_ws/src/FAST-LIVO2/Log/mat_out.txt'
out_html = '/root/zenith_ws/scans/corridor_scan_viewer.html'

print(f"Loading PCD: {pcd_path}")

# 1. Parse PCD
with open(pcd_path, 'rb') as f:
    while True:
        line = f.readline().decode('ascii', errors='ignore')
        if line.startswith('DATA'):
            break
    raw_data = f.read()

points = np.frombuffer(raw_data, dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')])
x_all, y_all, z_all = points['x'], points['y'], points['z']
rgb_all = points['rgb']

# Subsample for WebGL responsiveness if over 80,000 points
total_pts = len(x_all)
step = max(1, total_pts // 80000)
x, y, z = x_all[::step], y_all[::step], z_all[::step]
rgb = rgb_all[::step]

r = ((rgb >> 16) & 0xFF).astype(np.uint8)
g = ((rgb >> 8) & 0xFF).astype(np.uint8)
b = (rgb & 0xFF).astype(np.uint8)

# Stats
n_pts = len(x)
cx, cy, cz = float(np.median(x)), float(np.median(y)), float(np.median(z))
min_x, max_x = float(x_all.min()), float(x_all.max())
min_y, max_y = float(y_all.min()), float(y_all.max())
min_z, max_z = float(z_all.min()), float(z_all.max())
span_x = max_x - min_x
span_y = max_y - min_y
span_z = max_z - min_z

# 2. Parse Trajectory
traj_data = np.loadtxt(traj_path)
t_dur = float(traj_data[-1, 0] - traj_data[0, 0])
tx, ty, tz = traj_data[:, 4], traj_data[:, 5], traj_data[:, 6]
traj_len = float(np.sum(np.sqrt(np.diff(tx)**2 + np.diff(ty)**2 + np.diff(tz)**2)))
drift = float(np.sqrt((tx[-1]-tx[0])**2 + (ty[-1]-ty[0])**2 + (tz[-1]-tz[0])**2))

# Subsample trajectory to ~500 points
t_step = max(1, len(tx) // 500)
traj_sub = traj_data[::t_step, 4:7].astype('<f4')
n_traj = len(traj_sub)

# Turbo / Jet gradient for height (Z)
z_norm = np.clip((z - min_z) / (span_z + 1e-5), 0.0, 1.0)
z_r = (np.clip(1.5 - np.abs(4.0 * z_norm - 3.0), 0, 1) * 255).astype(np.uint8)
z_g = (np.clip(1.5 - np.abs(4.0 * z_norm - 2.0), 0, 1) * 255).astype(np.uint8)
z_b = (np.clip(1.5 - np.abs(4.0 * z_norm - 1.0), 0, 1) * 255).astype(np.uint8)

pos_bytes = np.column_stack([x, y, z]).astype('<f4').tobytes()
rgb_bytes = np.column_stack([r, g, b]).astype(np.uint8).tobytes()
z_rgb_bytes = np.column_stack([z_r, z_g, z_b]).astype(np.uint8).tobytes()
traj_bytes = traj_sub.tobytes()

b64_pos = base64.b64encode(pos_bytes).decode('ascii')
b64_rgb = base64.b64encode(rgb_bytes).decode('ascii')
b64_zrgb = base64.b64encode(z_rgb_bytes).decode('ascii')
b64_traj = base64.b64encode(traj_bytes).decode('ascii')

html_content = f"""<!DOCTYPE html>
<html lang="ko">
<head>
  <meta charset="UTF-8">
  <title>Zenith Drone 160m Corridor 3D Scan</title>
  <script src="https://www.gstatic.com/antigravity/web/dev/tailwindcss.min.js"></script>
</head>
<body class="bg-transparent text-[var(--foreground)] antialiased p-2">
  <div class="w-full h-[540px] relative rounded-2xl overflow-hidden border border-slate-700/80 shadow-2xl bg-[#090d16] select-none flex flex-col">
    <!-- Header / Stats Bar -->
    <div class="absolute top-3 left-3 z-20 flex flex-wrap gap-2 items-center bg-slate-900/90 backdrop-blur-md border border-slate-700/60 rounded-xl px-3 py-2 shadow-lg">
      <div class="flex items-center space-x-2 mr-1">
        <div class="w-2.5 h-2.5 rounded-full bg-emerald-400 animate-pulse"></div>
        <span class="font-bold text-xs tracking-wide text-white">복도 160m 스캔 결과</span>
      </div>
      <div class="h-3.5 w-px bg-slate-700"></div>
      <div class="px-2 py-0.5 bg-slate-800/90 rounded text-[11px] font-mono text-cyan-400">
        포인트: <span class="font-semibold text-white">{total_pts:,}개</span>
      </div>
      <div class="px-2 py-0.5 bg-slate-800/90 rounded text-[11px] font-mono text-amber-400">
        복도 규모: <span class="font-semibold text-white">{span_x:.1f}m × {span_y:.1f}m</span>
      </div>
      <div class="px-2 py-0.5 bg-slate-800/90 rounded text-[11px] font-mono text-purple-400">
        보행 궤적: <span class="font-semibold text-white">{traj_len:.1f}m ({t_dur:.0f}초)</span>
      </div>
      <div class="px-2 py-0.5 bg-emerald-950/80 border border-emerald-500/40 rounded text-[11px] font-mono text-emerald-400">
        루프 오차: <span class="font-semibold text-white">{drift*100:.1f}cm ({drift/traj_len*100:.2f}%)</span>
      </div>
    </div>

    <!-- Controls Panel -->
    <div class="absolute top-3 right-3 z-20 flex flex-col gap-1.5 bg-slate-900/90 backdrop-blur-md border border-slate-700/60 rounded-xl p-2.5 shadow-lg text-[11px]">
      <div class="font-semibold text-slate-300 flex items-center justify-between">
        <span>뷰어 조작</span>
        <span class="text-[9px] text-slate-400 ml-2">좌클릭: 회전 | 우클릭: 이동 | 휠: 줌</span>
      </div>
      
      <!-- Color Mode -->
      <div class="flex rounded-md bg-slate-800 p-0.5 border border-slate-700">
        <button id="btn-color-rgb" class="flex-1 py-0.5 px-2 rounded font-medium text-center bg-cyan-600 text-white transition">실사 RGB</button>
        <button id="btn-color-z" class="flex-1 py-0.5 px-2 rounded font-medium text-center text-slate-400 hover:text-white transition">고도 히트맵(Z)</button>
      </div>

      <!-- Point Size -->
      <div class="flex items-center justify-between gap-1.5 mt-0.5">
        <span class="text-slate-400 text-[10px]">포인트 크기:</span>
        <input id="slider-size" type="range" min="1" max="5" step="0.5" value="2.0" class="w-20 accent-cyan-500 cursor-pointer">
        <span id="label-size" class="font-mono text-cyan-400 w-4 text-right">2.0</span>
      </div>

      <!-- Trajectory Toggle -->
      <div class="flex items-center justify-between">
        <span class="text-slate-400 text-[10px]">이동 궤적 (빨간선):</span>
        <input id="check-traj" type="checkbox" checked class="w-3.5 h-3.5 accent-red-500 rounded cursor-pointer">
      </div>

      <!-- Camera Presets -->
      <div class="grid grid-cols-4 gap-1 mt-1">
        <button id="cam-iso" class="px-1.5 py-0.5 bg-slate-800 hover:bg-slate-700 rounded text-slate-300 hover:text-white transition text-center">입체</button>
        <button id="cam-top" class="px-1.5 py-0.5 bg-slate-800 hover:bg-slate-700 rounded text-slate-300 hover:text-white transition text-center">평면</button>
        <button id="cam-front" class="px-1.5 py-0.5 bg-slate-800 hover:bg-slate-700 rounded text-slate-300 hover:text-white transition text-center">정면</button>
        <button id="cam-side" class="px-1.5 py-0.5 bg-slate-800 hover:bg-slate-700 rounded text-slate-300 hover:text-white transition text-center">측면</button>
      </div>
    </div>

    <!-- WebGL Canvas -->
    <canvas id="glcanvas" class="w-full h-full block"></canvas>
  </div>

  <script>
    const b64Pos = "{b64_pos}";
    const b64Rgb = "{b64_rgb}";
    const b64Zrgb = "{b64_zrgb}";
    const b64Traj = "{b64_traj}";
    const nPts = {n_pts};
    const nTraj = {n_traj};

    function b64ToBuffer(b64) {{
      const binStr = atob(b64);
      const len = binStr.length;
      const bytes = new Uint8Array(len);
      for (let i = 0; i < len; i++) bytes[i] = binStr.charCodeAt(i);
      return bytes.buffer;
    }}

    const posFloat = new Float32Array(b64ToBuffer(b64Pos));
    const rgbBytes = new Uint8Array(b64ToBuffer(b64Rgb));
    const zrgbBytes = new Uint8Array(b64ToBuffer(b64Zrgb));
    const trajFloat = new Float32Array(b64ToBuffer(b64Traj));

    const canvas = document.getElementById("glcanvas");
    const gl = canvas.getContext("webgl", {{ antialias: true, alpha: false }});

    gl.enable(gl.DEPTH_TEST);
    gl.clearColor(0.04, 0.06, 0.11, 1.0);

    const vsPoints = `
      attribute vec3 aPos;
      attribute vec3 aColor;
      uniform mat4 uMVP;
      uniform float uPointSize;
      varying vec3 vColor;
      void main() {{
        gl_Position = uMVP * vec4(aPos, 1.0);
        gl_PointSize = uPointSize;
        vColor = aColor;
      }}
    `;
    const fsPoints = `
      precision mediump float;
      varying vec3 vColor;
      void main() {{
        vec2 pt = gl_PointCoord - vec2(0.5);
        if (dot(pt, pt) > 0.25) discard;
        gl_FragColor = vec4(vColor, 1.0);
      }}
    `;

    const vsLine = `
      attribute vec3 aPos;
      uniform mat4 uMVP;
      void main() {{
        gl_Position = uMVP * vec4(aPos, 1.0);
      }}
    `;
    const fsLine = `
      precision mediump float;
      uniform vec4 uColor;
      void main() {{
        gl_FragColor = uColor;
      }}
    `;

    function createShader(gl, type, source) {{
      const s = gl.createShader(type);
      gl.shaderSource(s, source);
      gl.compileShader(s);
      return s;
    }}
    function createProg(gl, vs, fs) {{
      const p = gl.createProgram();
      gl.attachShader(p, createShader(gl, gl.VERTEX_SHADER, vs));
      gl.attachShader(p, createShader(gl, gl.FRAGMENT_SHADER, fs));
      gl.linkProgram(p);
      return p;
    }}

    const progPoints = createProg(gl, vsPoints, fsPoints);
    const progLine = createProg(gl, vsLine, fsLine);

    const bufPos = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, bufPos);
    gl.bufferData(gl.ARRAY_BUFFER, posFloat, gl.STATIC_DRAW);

    const bufRgb = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, bufRgb);
    gl.bufferData(gl.ARRAY_BUFFER, rgbBytes, gl.STATIC_DRAW);

    const bufZrgb = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, bufZrgb);
    gl.bufferData(gl.ARRAY_BUFFER, zrgbBytes, gl.STATIC_DRAW);

    const bufTraj = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, bufTraj);
    gl.bufferData(gl.ARRAY_BUFFER, trajFloat, gl.STATIC_DRAW);

    let colorMode = 'rgb';
    let pointSize = 2.0;
    let showTraj = true;

    let cx = {cx:.2f}, cy = {cy:.2f}, cz = {cz:.2f};
    let radius = 45.0;
    let yaw = 0.6;
    let pitch = 0.6;

    let isDragging = false, isPanning = false;
    let lastX = 0, lastY = 0;

    canvas.addEventListener('mousedown', (e) => {{
      isDragging = (e.button === 0 && !e.shiftKey);
      isPanning = (e.button === 2 || (e.button === 0 && e.shiftKey));
      lastX = e.clientX;
      lastY = e.clientY;
    }});
    window.addEventListener('mouseup', () => {{ isDragging = false; isPanning = false; }});
    window.addEventListener('contextmenu', (e) => e.preventDefault());

    canvas.addEventListener('mousemove', (e) => {{
      const dx = e.clientX - lastX;
      const dy = e.clientY - lastY;
      lastX = e.clientX;
      lastY = e.clientY;

      if (isDragging) {{
        yaw += dx * 0.008;
        pitch = Math.max(-1.5, Math.min(1.5, pitch + dy * 0.008));
        requestAnimationFrame(render);
      }} else if (isPanning) {{
        const panSpeed = radius * 0.0015;
        const sinY = Math.sin(yaw), cosY = Math.cos(yaw);
        cx -= (dx * cosY - dy * sinY * Math.sin(pitch)) * panSpeed;
        cy -= (-dx * sinY - dy * cosY * Math.sin(pitch)) * panSpeed;
        cz += dy * Math.cos(pitch) * panSpeed;
        requestAnimationFrame(render);
      }}
    }});

    canvas.addEventListener('wheel', (e) => {{
      radius = Math.max(2.0, Math.min(150.0, radius * (1 + e.deltaY * 0.001)));
      requestAnimationFrame(render);
      e.preventDefault();
    }}, {{ passive: false }});

    let touchDist = 0;
    canvas.addEventListener('touchstart', (e) => {{
      if (e.touches.length === 1) {{
        lastX = e.touches[0].clientX;
        lastY = e.touches[0].clientY;
        isDragging = true;
      }} else if (e.touches.length === 2) {{
        touchDist = Math.hypot(e.touches[0].clientX - e.touches[1].clientX, e.touches[0].clientY - e.touches[1].clientY);
      }}
    }});
    canvas.addEventListener('touchmove', (e) => {{
      if (e.touches.length === 1 && isDragging) {{
        const dx = e.touches[0].clientX - lastX;
        const dy = e.touches[0].clientY - lastY;
        lastX = e.touches[0].clientX;
        lastY = e.touches[0].clientY;
        yaw += dx * 0.008;
        pitch = Math.max(-1.5, Math.min(1.5, pitch + dy * 0.008));
        requestAnimationFrame(render);
      }} else if (e.touches.length === 2) {{
        const d = Math.hypot(e.touches[0].clientX - e.touches[1].clientX, e.touches[0].clientY - e.touches[1].clientY);
        if (touchDist > 0) {{
          radius = Math.max(2.0, Math.min(150.0, radius * (touchDist / d)));
          requestAnimationFrame(render);
        }}
        touchDist = d;
      }}
    }});

    function mat4Perspective(fov, aspect, near, far) {{
      const f = 1.0 / Math.tan(fov / 2);
      const nf = 1 / (near - far);
      return [
        f / aspect, 0, 0, 0,
        0, f, 0, 0,
        0, 0, (far + near) * nf, -1,
        0, 0, 2 * far * near * nf, 0
      ];
    }}

    function mat4LookAt(eye, target, up) {{
      let z0 = eye[0] - target[0], z1 = eye[1] - target[1], z2 = eye[2] - target[2];
      let len = Math.hypot(z0, z1, z2) || 1;
      z0 /= len; z1 /= len; z2 /= len;

      let x0 = up[1] * z2 - up[2] * z1;
      let x1 = up[2] * z0 - up[0] * z2;
      let x2 = up[0] * z1 - up[1] * z0;
      len = Math.hypot(x0, x1, x2) || 1;
      x0 /= len; x1 /= len; x2 /= len;

      let y0 = z1 * x2 - z2 * x1;
      let y1 = z2 * x0 - z0 * x2;
      let y2 = z0 * x1 - z1 * x0;

      return [
        x0, y0, z0, 0,
        x1, y1, z1, 0,
        x2, y2, z2, 0,
        -(x0 * eye[0] + x1 * eye[1] + x2 * eye[2]),
        -(y0 * eye[0] + y1 * eye[1] + y2 * eye[2]),
        -(z0 * eye[0] + z1 * eye[1] + z2 * eye[2]),
        1
      ];
    }}

    function mat4Multiply(a, b) {{
      let out = new Float32Array(16);
      for (let i = 0; i < 4; i++) {{
        for (let j = 0; j < 4; j++) {{
          out[j * 4 + i] =
            a[i] * b[j * 4] +
            a[4 + i] * b[j * 4 + 1] +
            a[8 + i] * b[j * 4 + 2] +
            a[12 + i] * b[j * 4 + 3];
        }}
      }}
      return out;
    }}

    function render() {{
      const w = canvas.clientWidth * window.devicePixelRatio;
      const h = canvas.clientHeight * window.devicePixelRatio;
      if (canvas.width !== w || canvas.height !== h) {{
        canvas.width = w; canvas.height = h;
        gl.viewport(0, 0, w, h);
      }}

      gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

      const eyeX = cx + radius * Math.cos(pitch) * Math.sin(yaw);
      const eyeY = cy - radius * Math.cos(pitch) * Math.cos(yaw);
      const eyeZ = cz + radius * Math.sin(pitch);
      const proj = mat4Perspective(45 * Math.PI / 180, canvas.width / canvas.height, 0.1, 500.0);
      const view = mat4LookAt([eyeX, eyeY, eyeZ], [cx, cy, cz], [0, 0, 1]);
      const mvp = mat4Multiply(proj, view);

      gl.useProgram(progPoints);
      gl.uniformMatrix4fv(gl.getUniformLocation(progPoints, "uMVP"), false, mvp);
      gl.uniform1f(gl.getUniformLocation(progPoints, "uPointSize"), pointSize * window.devicePixelRatio);

      const aPos = gl.getAttribLocation(progPoints, "aPos");
      gl.enableVertexAttribArray(aPos);
      gl.bindBuffer(gl.ARRAY_BUFFER, bufPos);
      gl.vertexAttribPointer(aPos, 3, gl.FLOAT, false, 0, 0);

      const aColor = gl.getAttribLocation(progPoints, "aColor");
      gl.enableVertexAttribArray(aColor);
      gl.bindBuffer(gl.ARRAY_BUFFER, colorMode === 'rgb' ? bufRgb : bufZrgb);
      gl.vertexAttribPointer(aColor, 3, gl.UNSIGNED_BYTE, true, 0, 0);

      gl.drawArrays(gl.POINTS, 0, nPts);

      if (showTraj && nTraj > 1) {{
        gl.useProgram(progLine);
        gl.uniformMatrix4fv(gl.getUniformLocation(progLine, "uMVP"), false, mvp);
        gl.uniform4f(gl.getUniformLocation(progLine, "uColor"), 1.0, 0.25, 0.25, 1.0);

        const aLinePos = gl.getAttribLocation(progLine, "aPos");
        gl.enableVertexAttribArray(aLinePos);
        gl.bindBuffer(gl.ARRAY_BUFFER, bufTraj);
        gl.vertexAttribPointer(aLinePos, 3, gl.FLOAT, false, 0, 0);

        gl.drawArrays(gl.LINE_STRIP, 0, nTraj);
      }}
    }}

    const btnRgb = document.getElementById("btn-color-rgb");
    const btnZ = document.getElementById("btn-color-z");
    btnRgb.onclick = () => {{
      colorMode = 'rgb';
      btnRgb.className = "flex-1 py-0.5 px-2 rounded font-medium text-center bg-cyan-600 text-white transition";
      btnZ.className = "flex-1 py-0.5 px-2 rounded font-medium text-center text-slate-400 hover:text-white transition";
      render();
    }};
    btnZ.onclick = () => {{
      colorMode = 'z';
      btnZ.className = "flex-1 py-0.5 px-2 rounded font-medium text-center bg-cyan-600 text-white transition";
      btnRgb.className = "flex-1 py-0.5 px-2 rounded font-medium text-center text-slate-400 hover:text-white transition";
      render();
    }};

    const sliderSize = document.getElementById("slider-size");
    const labelSize = document.getElementById("label-size");
    sliderSize.oninput = (e) => {{
      pointSize = parseFloat(e.target.value);
      labelSize.innerText = pointSize.toFixed(1);
      render();
    }};

    const checkTraj = document.getElementById("check-traj");
    checkTraj.onchange = (e) => {{
      showTraj = e.target.checked;
      render();
    }};

    document.getElementById("cam-iso").onclick = () => {{ yaw = 0.6; pitch = 0.6; radius = 45.0; render(); }};
    document.getElementById("cam-top").onclick = () => {{ yaw = 0.0; pitch = 1.48; radius = 55.0; render(); }};
    document.getElementById("cam-front").onclick = () => {{ yaw = 0.0; pitch = 0.05; radius = 45.0; render(); }};
    document.getElementById("cam-side").onclick = () => {{ yaw = Math.PI / 2; pitch = 0.05; radius = 45.0; render(); }};

    window.addEventListener('resize', render);
    setTimeout(render, 50);
  </script>
</body>
</html>
"""

with open(out_html, 'w', encoding='utf-8') as f:
    f.write(html_content)

print(f"Generated 160m corridor viewer HTML at: {out_html} (Size: {len(html_content)/1024:.1f} KB)")
