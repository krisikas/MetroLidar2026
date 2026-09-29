/**
 * MetroLidar2026: Dynamic WebGL 3D Viewer.
 * Visualizes 3D point cloud, rails, extruded clearance envelope, and obstacles.
 * 100% pure vanilla WebGL, zero external dependencies.
 */

class MetroWebGLViewer {
  constructor(canvasId) {
    this.canvas = document.getElementById(canvasId);
    this.gl = this.canvas.getContext('webgl', { antialias: true, alpha: false });
    if (!this.gl) {
      console.error('WebGL not supported');
      return;
    }

    this.cameraMode = 'orbit'; // 'orbit' by default for interactive 3D inspection
    this.orbitAngles = { yaw: -1.57, pitch: 0.35, dist: 28.0, target: [0, -15, 0] };
    this.showRails = true;
    this.showEnvelope = true;
    this.showPoints = true;

    this.obstacles = [];
    this.pathPoints = [];
    this.cloudPoints = [];

    this.geometryDirty = true;
    this.pointsDirty = true;
    this.lineVertexCount = 0;
    this.pointVertexCount = 0;

    this.initShaders();
    this.initBuffers();
    this.initInteractions();

    this.resize();
    window.addEventListener('resize', () => this.resize());

    this.animate = this.animate.bind(this);
    requestAnimationFrame(this.animate);
  }

  resize() {
    if (!this.gl || !this.canvas) return;
    const dpr = window.devicePixelRatio || 1;
    const rect = this.canvas.getBoundingClientRect();
    const rawW = rect.width || this.canvas.clientWidth || 800;
    const rawH = rect.height || this.canvas.clientHeight || 500;
    const w = Math.floor(rawW * dpr);
    const h = Math.floor(rawH * dpr);
    if (this.canvas.width !== w || this.canvas.height !== h) {
      this.canvas.width = Math.max(100, w);
      this.canvas.height = Math.max(100, h);
      this.gl.viewport(0, 0, this.canvas.width, this.canvas.height);
    }
  }

  setCameraMode(mode) {
    this.cameraMode = mode;
  }

  resetCamera() {
    this.orbitAngles = { yaw: -1.57, pitch: 0.35, dist: 28.0, target: [0, -15, 0] };
  }

  updateState(data) {
    if (data.obstacles) {
      this.obstacles = data.obstacles;
      this.geometryDirty = true;
    }
    if (data.path_points && data.path_points.length > 0) {
      this.pathPoints = data.path_points;
      this.geometryDirty = true;
    }
    if (data.cloud_points && data.cloud_points.length > 0) {
      this.cloudPoints = data.cloud_points;
      this.pointsDirty = true;
    }
  }

  initShaders() {
    const gl = this.gl;
    const vsSource = `
      attribute vec3 aPosition;
      attribute vec4 aColor;
      uniform mat4 uMVP;
      varying vec4 vColor;
      void main() {
        gl_Position = uMVP * vec4(aPosition, 1.0);
        // Оптимальный размер дисков точек лидара
        gl_PointSize = clamp(360.0 / gl_Position.w, 3.5, 14.0);
        vColor = aColor;
      }
    `;

    const fsSource = `
      precision mediump float;
      uniform float uIsPoint;
      varying vec4 vColor;
      void main() {
        if (uIsPoint > 0.5) {
          vec2 coord = gl_PointCoord - vec2(0.5);
          float dist = length(coord);
          if (dist > 0.5) discard;
          float alpha = smoothstep(0.5, 0.40, dist) * vColor.a;
          gl_FragColor = vec4(vColor.rgb, alpha);
        } else {
          gl_FragColor = vColor;
        }
      }
    `;

    const vs = gl.createShader(gl.VERTEX_SHADER);
    gl.shaderSource(vs, vsSource);
    gl.compileShader(vs);

    const fs = gl.createShader(gl.FRAGMENT_SHADER);
    gl.shaderSource(fs, fsSource);
    gl.compileShader(fs);

    this.program = gl.createProgram();
    gl.attachShader(this.program, vs);
    gl.attachShader(this.program, fs);
    gl.linkProgram(this.program);

    this.aPosition = gl.getAttribLocation(this.program, 'aPosition');
    this.aColor = gl.getAttribLocation(this.program, 'aColor');
    this.uMVP = gl.getUniformLocation(this.program, 'uMVP');
    this.uIsPoint = gl.getUniformLocation(this.program, 'uIsPoint');
  }

  initBuffers() {
    const gl = this.gl;
    this.lineBuffer = gl.createBuffer();
    this.pointBuffer = gl.createBuffer();
  }

  initInteractions() {
    let isDragging = false;
    let dragButton = 0; // 0: левая, 1: средняя, 2: правая
    let lastX = 0, lastY = 0;

    this.canvas.style.cursor = 'grab';

    // Отключаем стандартное контекстное меню браузера для работы перемещения ПКМ
    this.canvas.addEventListener('contextmenu', (e) => e.preventDefault());

    this.canvas.addEventListener('mousedown', (e) => {
      isDragging = true;
      dragButton = e.button;
      lastX = e.clientX;
      lastY = e.clientY;
      this.canvas.style.cursor = (e.button === 2) ? 'crosshair' : 'grabbing';
    });

    window.addEventListener('mouseup', () => {
      isDragging = false;
      if (this.canvas) this.canvas.style.cursor = 'grab';
    });

    window.addEventListener('mousemove', (e) => {
      if (!isDragging) return;
      const dx = e.clientX - lastX;
      const dy = e.clientY - lastY;
      lastX = e.clientX;
      lastY = e.clientY;

      // ПКМ (кнопка 2) = Вращение (крутиться).
      // ЛКМ (кнопка 0) и СКМ (кнопка 1) = Перемещение (панорамирование).
      const isRotate = (dragButton === 2) || (dragButton === 0 && (e.altKey || e.ctrlKey));

      if (this.cameraMode === 'orbit') {
        if (isRotate) {
          // Орбитальное вращение (ПКМ)
          this.orbitAngles.yaw += dx * 0.008;
          this.orbitAngles.pitch = Math.max(-0.35, Math.min(1.45, this.orbitAngles.pitch + dy * 0.008));
        } else {
          // Четкое, отзывчивое панорамирование (ЛКМ)
          const yaw = this.orbitAngles.yaw;
          const pitch = this.orbitAngles.pitch;
          const d = this.orbitAngles.dist;

          const cosY = Math.cos(yaw);
          const sinY = Math.sin(yaw);
          const cosP = Math.cos(pitch);
          const sinP = Math.sin(pitch);

          // Базис камеры в мировых координатах
          const rx = -cosY;
          const ry = sinY;
          const ux = -sinP * sinY;
          const uy = -sinP * cosY;
          const uz = cosP;

          // Быстрый, заметный шаг перемещения
          const panFactor = Math.max(0.08, d * 0.005);

          this.orbitAngles.target[0] -= (rx * dx + ux * (-dy)) * panFactor;
          this.orbitAngles.target[1] -= (ry * dx + uy * (-dy)) * panFactor;
          this.orbitAngles.target[2] -= uz * (-dy) * panFactor;
        }
      }
      // В режиме FPV (вид из кабины) камера жестко зафиксирована на месте машиниста
    });

    this.canvas.addEventListener('wheel', (e) => {
      e.preventDefault();
      if (this.cameraMode === 'orbit') {
        this.orbitAngles.dist = Math.max(3.0, Math.min(150.0, this.orbitAngles.dist + e.deltaY * 0.06));
      }
    }, { passive: false });
  }

  buildLineGeometry() {
    const lines = []; // [x, y, z, r, g, b, a, ...]
    const halfGauge = 0.760; // 1520 mm / 2

    // Helper: compute track frame (center, normal, heading) along path points
    let waypoints = this.pathPoints;
    if (!waypoints || waypoints.length < 2) {
      // Fallback straight path
      waypoints = [];
      for (let s = 2.0; s <= 90.0; s += 2.0) {
        waypoints.push([0.0, -s, -1.15]);
      }
    }

    const nWp = waypoints.length;
    const normals = [];
    for (let i = 0; i < nWp; ++i) {
      let dx = 0.0, dy = -1.0;
      if (i + 1 < nWp) {
        dx = waypoints[i + 1][0] - waypoints[i][0];
        dy = waypoints[i + 1][1] - waypoints[i][1];
      } else if (i > 0) {
        dx = waypoints[i][0] - waypoints[i - 1][0];
        dy = waypoints[i][1] - waypoints[i - 1][1];
      }
      const yaw = Math.atan2(dx, -dy);
      // Unit normal perpendicular to track: (cos(yaw), sin(yaw), 0)
      normals.push([Math.cos(yaw), Math.sin(yaw)]);
    }

    // 1. Dynamic Extruded Dual Rails along Path
    if (this.showRails && nWp >= 2) {
      for (let i = 0; i < nWp - 1; ++i) {
        const p1 = waypoints[i];
        const p2 = waypoints[i + 1];
        const n1 = normals[i];
        const n2 = normals[i + 1];

        // Left rail (x - halfGauge * cos, y - halfGauge * sin, z)
        const lx1 = p1[0] - halfGauge * n1[0];
        const ly1 = p1[1] - halfGauge * n1[1];
        const lz1 = p1[2];

        const lx2 = p2[0] - halfGauge * n2[0];
        const ly2 = p2[1] - halfGauge * n2[1];
        const lz2 = p2[2];

        lines.push(lx1, ly1, lz1, 0.85, 0.90, 0.95, 0.95);
        lines.push(lx2, ly2, lz2, 0.85, 0.90, 0.95, 0.95);

        // Right rail
        const rx1 = p1[0] + halfGauge * n1[0];
        const ry1 = p1[1] + halfGauge * n1[1];
        const rz1 = p1[2];

        const rx2 = p2[0] + halfGauge * n2[0];
        const ry2 = p2[1] + halfGauge * n2[1];
        const rz2 = p2[2];

        lines.push(rx1, ry1, rz1, 0.85, 0.90, 0.95, 0.95);
        lines.push(rx2, ry2, rz2, 0.85, 0.90, 0.95, 0.95);

        // Sleepers (every step)
        const sx1 = p1[0] - 1.35 * n1[0];
        const sy1 = p1[1] - 1.35 * n1[1];
        const sx2 = p1[0] + 1.35 * n1[0];
        const sy2 = p1[1] + 1.35 * n1[1];
        lines.push(sx1, sy1, p1[2] - 0.10, 0.30, 0.35, 0.40, 0.70);
        lines.push(sx2, sy2, p1[2] - 0.10, 0.30, 0.35, 0.40, 0.70);

        // Contact rail on left side (offset 1.25m, elevated 0.20m above rail crown)
        const kx1 = p1[0] - 1.25 * n1[0];
        const ky1 = p1[1] - 1.25 * n1[1];
        const kz1 = p1[2] + 0.20;

        const kx2 = p2[0] - 1.25 * n2[0];
        const ky2 = p2[1] - 1.25 * n2[1];
        const kz2 = p2[2] + 0.20;

        lines.push(kx1, ky1, kz1, 0.95, 0.60, 0.15, 0.85);
        lines.push(kx2, ky2, kz2, 0.95, 0.60, 0.15, 0.85);
      }
    }

    // 2. Dynamic Extruded Clearance Envelope («Габарит М») along Path
    if (this.showEnvelope && nWp >= 2) {
      // 2D profile offsets (u, v) relative to (track_center, rail_crown)
      const profile = [
        [-1.05, 0.12], [1.05, 0.12],
        [1.05, 0.45],  [1.30, 0.45],
        [1.30, 1.25],  [1.35, 1.25],
        [1.35, 2.45],  [1.05, 2.80],
        [-1.05, 2.80], [-1.35, 2.45],
        [-1.35, 1.25], [-1.30, 1.25],
        [-1.30, 0.45], [-1.05, 0.45],
        [-1.05, 0.12]
      ];

      // Draw cross-section rings every ~4 waypoints
      const ringInterval = 4;
      for (let i = 1; i < nWp; i += ringInterval) {
        const wp = waypoints[i];
        const n = normals[i];

        for (let j = 0; j < profile.length - 1; ++j) {
          const u1 = profile[j][0], v1 = profile[j][1];
          const u2 = profile[j + 1][0], v2 = profile[j + 1][1];

          const p1x = wp[0] + u1 * n[0];
          const p1y = wp[1] + u1 * n[1];
          const p1z = wp[2] + v1;

          const p2x = wp[0] + u2 * n[0];
          const p2y = wp[1] + u2 * n[1];
          const p2z = wp[2] + v2;

          lines.push(p1x, p1y, p1z, 0.15, 0.80, 0.40, 0.60);
          lines.push(p2x, p2y, p2z, 0.15, 0.80, 0.40, 0.60);
        }
      }

      // Longitudinal spine lines along key envelope edges
      const spineKeyIndices = [0, 1, 3, 5, 6, 7, 8, 9, 11, 13];
      for (const idx of spineKeyIndices) {
        const u = profile[idx][0];
        const v = profile[idx][1];

        for (let i = 0; i < nWp - 1; ++i) {
          const p1 = waypoints[i];
          const p2 = waypoints[i + 1];
          const n1 = normals[i];
          const n2 = normals[i + 1];

          const p1x = p1[0] + u * n1[0];
          const p1y = p1[1] + u * n1[1];
          const p1z = p1[2] + v;

          const p2x = p2[0] + u * n2[0];
          const p2y = p2[1] + u * n2[1];
          const p2z = p2[2] + v;

          lines.push(p1x, p1y, p1z, 0.15, 0.70, 0.35, 0.35);
          lines.push(p2x, p2y, p2z, 0.15, 0.70, 0.35, 0.35);
        }
      }
    }

    // 3. Track Center Ribbon (Azure Blue Line)
    if (nWp >= 2) {
      for (let i = 0; i < nWp - 1; ++i) {
        const p1 = waypoints[i];
        const p2 = waypoints[i + 1];
        lines.push(p1[0], p1[1], p1[2], 0.20, 0.70, 1.0, 0.90);
        lines.push(p2[0], p2[1], p2[2], 0.20, 0.70, 1.0, 0.90);
      }
    }

    // 4. Detected Obstacles (3D Bounding Boxes with Threat Color Coding)
    for (const obs of this.obstacles) {
      const cx = Number(obs.x) || 0.0;
      const cy = Number(obs.y) || -10.0;
      const cz = Number(obs.z) || -1.0;
      const hx = Math.max(0.20, (Number(obs.size_x) || 0.5) * 0.5);
      const hy = Math.max(0.20, (Number(obs.size_y) || 0.5) * 0.5);
      const hz = Math.max(0.15, (Number(obs.size_z) || 0.5) * 0.5);

      let r = 0.95, g = 0.20, b = 0.20, a = 1.0;
      if (obs.threat === 1) { // Warning
        r = 0.95; g = 0.65; b = 0.10;
      } else if (obs.threat === 0) { // Caution
        r = 0.30; g = 0.75; b = 1.0;
      }

      const corners = [
        [cx - hx, cy - hy, cz - hz],
        [cx + hx, cy - hy, cz - hz],
        [cx + hx, cy + hy, cz - hz],
        [cx - hx, cy + hy, cz - hz],
        [cx - hx, cy - hy, cz + hz],
        [cx + hx, cy - hy, cz + hz],
        [cx + hx, cy + hy, cz + hz],
        [cx - hx, cy + hy, cz + hz]
      ];

      const edges = [
        [0, 1], [1, 2], [2, 3], [3, 0],
        [4, 5], [5, 6], [6, 7], [7, 4],
        [0, 4], [1, 5], [2, 6], [3, 7]
      ];

      for (const [i1, i2] of edges) {
        lines.push(corners[i1][0], corners[i1][1], corners[i1][2], r, g, b, a);
        lines.push(corners[i2][0], corners[i2][1], corners[i2][2], r, g, b, a);
      }

      // Cross-lines on top and bottom face for visibility
      lines.push(corners[0][0], corners[0][1], corners[0][2], r, g, b, 0.6);
      lines.push(corners[2][0], corners[2][1], corners[2][2], r, g, b, 0.6);
      lines.push(corners[4][0], corners[4][1], corners[4][2], r, g, b, 0.6);
      lines.push(corners[6][0], corners[6][1], corners[6][2], r, g, b, 0.6);
    }

    return new Float32Array(lines);
  }

  buildPointGeometry() {
    if (!this.showPoints || !this.cloudPoints || this.cloudPoints.length === 0) {
      return new Float32Array(0);
    }
    const pts = [];
    const obsList = this.obstacles || [];

    for (const p of this.cloudPoints) {
      const x = p[0], y = p[1], z = p[2];

      // Проверка на принадлежность или близость к обнаруженному препятствию
      let isNearObstacle = false;
      for (const obs of obsList) {
        const hx = Math.max(0.4, Number(obs.size_x) * 0.5 + 0.35);
        const hy = Math.max(0.6, Number(obs.size_y) * 0.5 + 0.50);
        const hz = Math.max(0.4, Number(obs.size_z) * 0.5 + 0.35);
        if (Math.abs(y - Number(obs.y)) <= hy &&
            Math.abs(x - Number(obs.x)) <= hx &&
            Math.abs(z - Number(obs.z)) <= hz) {
          isNearObstacle = true;
          break;
        }
      }

      if (isNearObstacle) {
        // Ярко-оранжевый/красный цвет для точек препятствий
        pts.push(x, y, z, 1.0, 0.25, 0.20, 1.0);
      } else {
        // Исходная естественная расцветка по высоте (высотный спектральный градиент)
        const t = Math.max(0.0, Math.min(1.0, (z + 2.0) / 4.0));
        const r = 0.20 + 0.60 * t;
        const g = 0.40 + 0.40 * (1.0 - Math.abs(t - 0.5) * 2.0);
        const b = 0.70 - 0.50 * t;
        pts.push(x, y, z, r, g, b, 0.80);
      }
    }
    return new Float32Array(pts);
  }

  animate() {
    try {
      this.render();
    } catch (err) {
      console.warn('WebGL render issue:', err);
    }
    requestAnimationFrame(this.animate);
  }

  render() {
    const gl = this.gl;
    if (!gl) return;

    gl.clearColor(0.05, 0.07, 0.10, 1.0);
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
    gl.enable(gl.DEPTH_TEST);
    gl.enable(gl.BLEND);
    gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);

    const w = this.canvas.width || 800;
    const h = this.canvas.height || 500;
    const aspect = (h > 0) ? (w / h) : 1.6;
    const proj = this.mat4Perspective(50 * (Math.PI / 180), aspect, 0.5, 250.0);

    let eye, target, up = [0, 0, 1];
    if (this.cameraMode === 'fpv') {
      const zBase = (this.pathPoints && this.pathPoints.length > 0) ? this.pathPoints[0][2] : -1.25;
      eye = [0.0, 0.5, zBase + 1.65];
      if (this.pathPoints && this.pathPoints.length > 8) {
        const lookIdx = Math.min(this.pathPoints.length - 1, Math.floor(this.pathPoints.length * 0.40));
        const lookPt = this.pathPoints[lookIdx];
        target = [lookPt[0], lookPt[1], lookPt[2] + 1.10];
      } else {
        target = [0.0, -50.0, zBase + 1.30];
      }
    } else {
      const yaw = this.orbitAngles.yaw;
      const pitch = this.orbitAngles.pitch;
      const d = this.orbitAngles.dist;
      const tx = this.orbitAngles.target[0];
      const ty = this.orbitAngles.target[1];
      const tz = this.orbitAngles.target[2];

      eye = [
        tx + d * Math.cos(pitch) * Math.sin(yaw),
        ty + d * Math.cos(pitch) * Math.cos(yaw),
        tz + d * Math.sin(pitch)
      ];
      target = [tx, ty, tz];
    }

    const view = this.mat4LookAt(eye, target, up);
    const mvp = this.mat4Multiply(proj, view);

    gl.useProgram(this.program);
    gl.uniformMatrix4fv(this.uMVP, false, mvp);

    const stride = 7 * 4;

    // 1. Draw Lines (Rails, Envelope, Obstacle Boxes)
    if (this.geometryDirty) {
      const lineData = this.buildLineGeometry();
      this.lineVertexCount = lineData.length / 7;
      if (lineData.length > 0) {
        gl.bindBuffer(gl.ARRAY_BUFFER, this.lineBuffer);
        gl.bufferData(gl.ARRAY_BUFFER, lineData, gl.DYNAMIC_DRAW);
      }
      this.geometryDirty = false;
    }

    if (this.lineVertexCount > 0) {
      gl.uniform1f(this.uIsPoint, 0.0);
      gl.bindBuffer(gl.ARRAY_BUFFER, this.lineBuffer);
      gl.enableVertexAttribArray(this.aPosition);
      gl.vertexAttribPointer(this.aPosition, 3, gl.FLOAT, false, stride, 0);
      gl.enableVertexAttribArray(this.aColor);
      gl.vertexAttribPointer(this.aColor, 4, gl.FLOAT, false, stride, 3 * 4);
      gl.lineWidth(2.0);
      gl.drawArrays(gl.LINES, 0, this.lineVertexCount);
    }

    // 2. Draw Point Cloud Points
    if (this.pointsDirty) {
      const ptData = this.buildPointGeometry();
      this.pointVertexCount = ptData.length / 7;
      if (ptData.length > 0) {
        gl.bindBuffer(gl.ARRAY_BUFFER, this.pointBuffer);
        gl.bufferData(gl.ARRAY_BUFFER, ptData, gl.DYNAMIC_DRAW);
      }
      this.pointsDirty = false;
    }

    if (this.showPoints && this.pointVertexCount > 0) {
      gl.uniform1f(this.uIsPoint, 1.0);
      gl.bindBuffer(gl.ARRAY_BUFFER, this.pointBuffer);
      gl.enableVertexAttribArray(this.aPosition);
      gl.vertexAttribPointer(this.aPosition, 3, gl.FLOAT, false, stride, 0);
      gl.enableVertexAttribArray(this.aColor);
      gl.vertexAttribPointer(this.aColor, 4, gl.FLOAT, false, stride, 3 * 4);
      gl.drawArrays(gl.POINTS, 0, this.pointVertexCount);
    }
  }

  // --- Matrix Math Helpers ---
  mat4Perspective(fovy, aspect, near, far) {
    const f = 1.0 / Math.tan(fovy / 2);
    const nf = 1 / (near - far);
    return new Float32Array([
      f / aspect, 0, 0, 0,
      0, f, 0, 0,
      0, 0, (far + near) * nf, -1,
      0, 0, (2 * far * near) * nf, 0
    ]);
  }

  mat4LookAt(eye, center, up) {
    let z0 = eye[0] - center[0], z1 = eye[1] - center[1], z2 = eye[2] - center[2];
    let len = 1 / Math.hypot(z0, z1, z2);
    z0 *= len; z1 *= len; z2 *= len;

    let x0 = up[1] * z2 - up[2] * z1, x1 = up[2] * z0 - up[0] * z2, x2 = up[0] * z1 - up[1] * z0;
    len = Math.hypot(x0, x1, x2);
    if (!len) { x0 = 1; x1 = 0; x2 = 0; } else { len = 1 / len; x0 *= len; x1 *= len; x2 *= len; }

    let y0 = z1 * x2 - z2 * x1, y1 = z2 * x0 - z0 * x2, y2 = z0 * x1 - z1 * x0;

    return new Float32Array([
      x0, y0, z0, 0,
      x1, y1, z1, 0,
      x2, y2, z2, 0,
      -(x0 * eye[0] + x1 * eye[1] + x2 * eye[2]),
      -(y0 * eye[0] + y1 * eye[1] + y2 * eye[2]),
      -(z0 * eye[0] + z1 * eye[1] + z2 * eye[2]),
      1
    ]);
  }

  mat4Multiply(a, b) {
    const out = new Float32Array(16);
    for (let i = 0; i < 4; ++i) {
      for (let j = 0; j < 4; ++j) {
        out[j * 4 + i] =
          a[0 * 4 + i] * b[j * 4 + 0] +
          a[1 * 4 + i] * b[j * 4 + 1] +
          a[2 * 4 + i] * b[j * 4 + 2] +
          a[3 * 4 + i] * b[j * 4 + 3];
      }
    }
    return out;
  }
}

window.MetroWebGLViewer = MetroWebGLViewer;
