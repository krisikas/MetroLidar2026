/**
 * Инженерная панель контроля габарита и детекции препятствий.
 * Управление 3D сценой, потоком данных телеметрии и таблицей препятствий.
 */

document.addEventListener('DOMContentLoaded', () => {
  const viewer = new MetroWebGLViewer('gl-canvas');

  // Элементы управления камерой
  const btnCamFpv = document.getElementById('btn-cam-fpv');
  const btnCamOrbit = document.getElementById('btn-cam-orbit');
  const btnCamReset = document.getElementById('btn-cam-reset');

  // Слои отображения
  const chkRails = document.getElementById('chk-rails');
  const chkEnvelope = document.getElementById('chk-envelope');
  const chkPoints = document.getElementById('chk-points');

  // Верхняя полоса телеметрии
  const valDds = document.getElementById('val-dds');
  const valHz = document.getElementById('val-hz');
  const valCalcTime = document.getElementById('val-calc-time');
  const valPointsCount = document.getElementById('val-points-count');

  // Информационный блок состояния
  const hudStatusText = document.getElementById('hud-status-text');
  const dispTrainSpeed = document.getElementById('disp-train-speed');
  const dispBrakingDistance = document.getElementById('disp-braking-distance');
  const dispClosestObstacle = document.getElementById('disp-closest-obstacle');
  const dispTimeToCollision = document.getElementById('disp-time-to-collision');

  // Таблица препятствий
  const countObstacles = document.getElementById('count-obstacles');
  const tableBodyObstacles = document.getElementById('table-body-obstacles');

  // Переключение режимов камеры
  btnCamFpv.addEventListener('click', () => {
    viewer.setCameraMode('fpv');
    btnCamFpv.classList.add('active');
    btnCamOrbit.classList.remove('active');
  });

  btnCamOrbit.addEventListener('click', () => {
    viewer.setCameraMode('orbit');
    btnCamOrbit.classList.add('active');
    btnCamFpv.classList.remove('active');
  });

  btnCamReset.addEventListener('click', () => {
    viewer.resetCamera();
  });

  // Переключение видимости слоев
  chkRails.addEventListener('change', (e) => { viewer.showRails = e.target.checked; });
  chkEnvelope.addEventListener('change', (e) => { viewer.showEnvelope = e.target.checked; });
  chkPoints.addEventListener('change', (e) => { viewer.showPoints = e.target.checked; });

  const CATEGORIES = {
    0: 'Неопределено',
    1: 'Крупногабаритный объект',
    2: 'Предмет на путях',
    3: 'Подвешенный кабель'
  };

  function updateInterface(data) {
    if (!data) return;

    // 1. Обновление 3D сцены
    viewer.updateState(data);

    // 2. Статус связи и телеметрия
    const ddsActive = Boolean(data.dds_active);
    if (ddsActive) {
      valDds.textContent = 'АКТИВНА';
      valDds.className = 'cell-value online';
    } else {
      valDds.textContent = 'ОЖИДАНИЕ';
      valDds.className = 'cell-value';
    }

    if (data.topic_rates) {
      valHz.textContent = `${(data.topic_rates.lidar_hz || 0.0).toFixed(1)} Гц`;
    }

    if (data.telemetry) {
      valCalcTime.textContent = `${(data.telemetry.latency_ms || 0.0).toFixed(1)} мс`;
      valPointsCount.textContent = (data.telemetry.input_pts || 0).toLocaleString();
    }

    // 3. Параметры движения и торможения
    const speedMps = Number(data.train_speed || 0.0);
    const speedKmh = Number(data.train_speed_kmh || (speedMps * 3.6));

    if (speedMps < 0.20) {
      dispTrainSpeed.textContent = '0.0 км/ч (состав остановлен)';
    } else {
      dispTrainSpeed.textContent = `${speedKmh.toFixed(1)} км/ч (${speedMps.toFixed(2)} м/с)`;
    }

    const brakeDist = Number(data.braking_distance || 0.0);
    if (speedMps < 0.20 || brakeDist <= 0.0) {
      dispBrakingDistance.textContent = '0.0 м (состав остановлен)';
    } else {
      dispBrakingDistance.textContent = `${brakeDist.toFixed(1)} м`;
    }

    // 4. Анализ препятствий и статус пути
    const obstacles = data.obstacles || [];
    countObstacles.textContent = obstacles.length;

    let closest = null;
    if (obstacles.length > 0) {
      closest = obstacles.reduce((min, o) => (o.distance < min.distance ? o : min), obstacles[0]);
    }

    hudStatusText.className = 'hud-title';

    if (closest) {
      const dist = Number(closest.distance);
      const isCritical = dist <= brakeDist || closest.threat === 3 || (speedMps > 1.0 && dist < 40.0);

      if (isCritical) {
        hudStatusText.classList.add('danger');
        hudStatusText.textContent = `ОПАСНОСТЬ: ОБЪЕКТ НА ${dist.toFixed(1)} М`;
      } else {
        hudStatusText.classList.add('warning');
        hudStatusText.textContent = `ВНИМАНИЕ: ОБЪЕКТ НА ${dist.toFixed(1)} М`;
      }

      dispClosestObstacle.textContent = `${dist.toFixed(1)} м`;

      const ttc = Number(data.ttc);
      if (speedMps >= 0.50 && ttc > 0 && ttc < 300) {
        dispTimeToCollision.textContent = `${ttc.toFixed(1)} с`;
      } else {
        dispTimeToCollision.textContent = '— (состав неподвижен)';
      }
    } else {
      hudStatusText.textContent = 'ПУТЬ СВОБОДЕН';
      dispClosestObstacle.textContent = '—';
      dispTimeToCollision.textContent = '—';
    }

    // 5. Полноразмерная таблица препятствий
    if (obstacles.length === 0) {
      tableBodyObstacles.innerHTML = '<tr><td colspan="8" class="row-empty">Объектов в габарите пути не обнаружено</td></tr>';
    } else {
      let rows = '';
      for (const obs of obstacles) {
        let tag = '<span class="status-tag tag-info">ИНФО</span>';
        if (obs.threat === 3 || obs.distance <= brakeDist) {
          tag = '<span class="status-tag tag-danger">ОПАСНОСТЬ</span>';
        } else if (obs.threat === 2 || obs.threat === 1) {
          tag = '<span class="status-tag tag-warning">ВНИМАНИЕ</span>';
        }

        const cat = CATEGORIES[obs.category] || 'Объект';
        const posX = `${obs.x >= 0 ? '+' : ''}${Number(obs.x).toFixed(2)}`;
        const posZ = Number(obs.z).toFixed(2);
        const dims = `${Number(obs.size_x).toFixed(2)} × ${Number(obs.size_z).toFixed(2)}`;

        rows += `
          <tr>
            <td>#${obs.id}</td>
            <td>${cat}</td>
            <td>${Number(obs.distance).toFixed(1)} м</td>
            <td>${posX}</td>
            <td>${posZ}</td>
            <td>${dims}</td>
            <td>${obs.pts || 1}</td>
            <td>${tag}</td>
          </tr>
        `;
      }
      tableBodyObstacles.innerHTML = rows;
    }
  }

  // Подключение к потоку SSE
  let lastDataTime = 0;
  let es = null;

  function startStream() {
    if (es) {
      try { es.close(); } catch (e) {}
    }

    valDds.textContent = 'ПОДКЛЮЧЕНИЕ...';
    valDds.className = 'cell-value';

    try {
      es = new EventSource('/events');

      es.onopen = () => {
        valDds.textContent = 'АКТИВНА';
        valDds.className = 'cell-value online';
      };

      es.onmessage = (event) => {
        try {
          const payload = JSON.parse(event.data);
          lastDataTime = Date.now();
          updateInterface(payload);
        } catch (err) {
          console.error('SSE parse error:', err);
        }
      };

      es.onerror = () => {
        try { es.close(); } catch (e) {}
        es = null;
        setTimeout(startStream, 3000);
      };
    } catch (err) {
      console.warn('EventSource error:', err);
    }
  }

  async function pollState() {
    if (Date.now() - lastDataTime > 800) {
      try {
        const res = await fetch('/api/state');
        if (res.ok) {
          const payload = await res.json();
          lastDataTime = Date.now();
          updateInterface(payload);
        }
      } catch (err) {
        // Ожидание соединения
      }
    }
  }

  // Начальная загрузка
  fetch('/api/state')
    .then(r => r.json())
    .then(d => {
      lastDataTime = Date.now();
      updateInterface(d);
    })
    .catch(() => {});

  startStream();
  setInterval(pollState, 600);
});
