/**
 * Инженерная панель контроля габарита и детекции препятствий.
 * Управление 3D сценой, потоком данных телеметрии и таблицей препятствий.
 */

document.addEventListener('DOMContentLoaded', () => {
  const viewer = new MetroWebGLViewer('gl-canvas');

  // Элементы управления камерой
  const btnCamFpv = document.getElementById('btn-cam-fpv');
  const btnCamOrbit = document.getElementById('btn-cam-orbit');

  // Слои отображения
  const chkRails = document.getElementById('chk-rails');
  const chkEnvelope = document.getElementById('chk-envelope');
  const chkPoints = document.getElementById('chk-points');

  // Безопасные функции установки текстовых значений и классов
  function setSafeText(id, text, className) {
    const el = document.getElementById(id);
    if (el) {
      if (text !== undefined && text !== null) el.textContent = text;
      if (className !== undefined) el.className = className;
    }
  }

  function setSafeHtml(id, html) {
    const el = document.getElementById(id);
    if (el && html !== undefined && html !== null) {
      el.innerHTML = html;
    }
  }

  // Переключение режимов камеры
  if (btnCamFpv) {
    btnCamFpv.addEventListener('click', () => {
      viewer.setCameraMode('fpv');
      btnCamFpv.classList.add('active');
      if (btnCamOrbit) btnCamOrbit.classList.remove('active');
    });
  }

  if (btnCamOrbit) {
    btnCamOrbit.addEventListener('click', () => {
      viewer.setCameraMode('orbit');
      btnCamOrbit.classList.add('active');
      if (btnCamFpv) btnCamFpv.classList.remove('active');
    });
  }

  // Переключение видимости слоев
  if (chkRails) chkRails.addEventListener('change', (e) => { viewer.showRails = e.target.checked; });
  if (chkEnvelope) chkEnvelope.addEventListener('change', (e) => { viewer.showEnvelope = e.target.checked; });
  if (chkPoints) chkPoints.addEventListener('change', (e) => { viewer.showPoints = e.target.checked; });

  const CATEGORIES = {
    0: 'Неопределено',
    1: 'Крупногабаритный объект',
    2: 'Предмет на путях',
    3: 'Подвешенный кабель'
  };

  function updateInterface(data) {
    if (!data) return;

    // 1. Обновление 3D сцены
    try {
      viewer.updateState(data);
    } catch (e) {
      console.warn('Viewer update error:', e);
    }

    // 2. Статус связи и раздельные частоты системы
    try {
      const ddsActive = Boolean(data.dds_active);
      if (ddsActive) {
        setSafeText('val-dds', 'Активна', 'cell-value online');
      } else {
        setSafeText('val-dds', 'Ожидание', 'cell-value');
      }

      if (data.topic_rates) {
        setSafeText('val-hz-lidar', `${(data.topic_rates.lidar_hz || 0.0).toFixed(1)} Гц`);
        setSafeText('val-hz-path', `${(data.topic_rates.path_hz || 0.0).toFixed(1)} Гц`);
        setSafeText('val-hz-obs', `${(data.topic_rates.obstacles_hz || 0.0).toFixed(1)} Гц`);
      }

      if (data.telemetry) {
        setSafeText('val-calc-time', `${(data.telemetry.latency_ms || 0.0).toFixed(1)} мс`);
        setSafeText('val-points-count', (data.telemetry.input_pts || 0).toLocaleString());
      }
    } catch (e) {
      console.warn('Telemetry update error:', e);
    }

    // 3. Параметры движения и расчетная скорость по лидару
    let speedMps = 0.0;
    let brakeDist = 0.0;
    try {
      speedMps = Number(data.train_speed || 0.0);
      const speedKmh = Number(data.train_speed_kmh || (speedMps * 3.6));

      if (speedMps < 0.20) {
        setSafeText('disp-train-speed', '0.0 км/ч (состав остановлен)');
      } else {
        setSafeText('disp-train-speed', `${speedKmh.toFixed(1)} км/ч (${speedMps.toFixed(2)} м/с, расчет по лидару)`);
      }

      brakeDist = Number(data.braking_distance || 0.0);
      if (speedMps < 0.20 || brakeDist <= 0.0) {
        setSafeText('disp-braking-distance', '0.0 м (состав остановлен)');
      } else {
        setSafeText('disp-braking-distance', `${brakeDist.toFixed(1)} м`);
      }
    } catch (e) {
      console.warn('Speed telemetry error:', e);
    }

    // 4. Анализ препятствий и статус пути в HUD
    const obstacles = data.obstacles || [];
    try {
      setSafeText('count-obstacles', obstacles.length);

      let closest = null;
      if (obstacles.length > 0) {
        closest = obstacles.reduce((min, o) => (o.distance < min.distance ? o : min), obstacles[0]);
      }

      const hudStatusText = document.getElementById('hud-status-text');
      if (hudStatusText) {
        hudStatusText.className = 'hud-title';

        if (closest) {
          const dist = Number(closest.distance);
          const isCritical = dist <= brakeDist || closest.threat === 3 || (speedMps > 1.0 && dist < 40.0);

          if (isCritical) {
            hudStatusText.classList.add('danger');
            hudStatusText.textContent = `Опасность: объект на ${dist.toFixed(1)} м`;
          } else {
            hudStatusText.classList.add('warning');
            hudStatusText.textContent = `Внимание: объект на ${dist.toFixed(1)} м`;
          }

          setSafeText('disp-closest-obstacle', `${dist.toFixed(1)} м`);

          const ttc = Number(data.ttc);
          if (speedMps >= 0.50 && ttc > 0 && ttc < 300) {
            setSafeText('disp-time-to-collision', `${ttc.toFixed(1)} с`);
          } else {
            setSafeText('disp-time-to-collision', '— (состав неподвижен)');
          }
        } else {
          hudStatusText.textContent = 'Путь свободен';
          setSafeText('disp-closest-obstacle', '—');
          setSafeText('disp-time-to-collision', '—');
        }
      }
    } catch (e) {
      console.warn('HUD update error:', e);
    }

    // 5. Полноразмерная таблица подтвержденных препятствий
    try {
      if (obstacles.length === 0) {
        setSafeHtml('table-body-obstacles', '<tr><td colspan="8" class="row-empty">Объектов в габарите пути не обнаружено</td></tr>');
      } else {
        let rows = '';
        for (const obs of obstacles) {
          let tag = '<span class="status-tag tag-info">Инфо</span>';
          if (obs.threat === 3 || obs.distance <= brakeDist) {
            tag = '<span class="status-tag tag-danger">Опасность</span>';
          } else if (obs.threat === 2 || obs.threat === 1) {
            tag = '<span class="status-tag tag-warning">Внимание</span>';
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
        setSafeHtml('table-body-obstacles', rows);
      }
    } catch (e) {
      console.warn('Obstacle table update error:', e);
    }
  }

  // Подключение к потоку SSE
  let lastDataTime = 0;
  let es = null;

  function startStream() {
    if (es) {
      try { es.close(); } catch (e) {}
    }

    setSafeText('val-dds', 'Подключение...', 'cell-value');

    try {
      es = new EventSource('/events');

      es.onopen = () => {
        setSafeText('val-dds', 'Активна', 'cell-value online');
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
