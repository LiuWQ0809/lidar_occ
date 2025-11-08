(function () {
  const viewerContainer = document.getElementById('viewer');
  const statusEl = document.getElementById('status');
  const logEl = document.getElementById('log');
  const wsInput = document.getElementById('wsUrl');
  const pointTopicInput = document.getElementById('pointTopic');
  const gridTopicInput = document.getElementById('gridTopic');
  const connectBtn = document.getElementById('connectBtn');
  const reloadBtn = document.getElementById('reloadBtn');
  const togglePointCheckbox = document.getElementById('togglePointCloud');
  const toggleGridCheckbox = document.getElementById('toggleGrid');

  let ros = null;
  let tfClient = null;
  let viewer = null;
  let currentCloud = null;
  let currentGrid = null;

  function appendLog(message) {
    const time = new Date().toLocaleTimeString();
    const el = document.createElement('div');
    el.textContent = `[${time}] ${message}`;
    logEl.prepend(el);
    const entries = logEl.querySelectorAll('div');
    if (entries.length > 100) {
      logEl.removeChild(logEl.lastChild);
    }
  }

  function setStatus(text, ok = false) {
    statusEl.textContent = text;
    statusEl.classList.toggle('ok', ok);
  }

  function initViewer() {
    viewerContainer.innerHTML = '';
    const width = viewerContainer.clientWidth || viewerContainer.offsetWidth || 800;
    const height = viewerContainer.clientHeight || viewerContainer.offsetHeight || 600;

    viewer = new ROS3D.Viewer({
      divID: 'viewer',
      width,
      height,
      antialias: true,
      background: '#05070a',
      cameraPose: {x: 5, y: -5, z: 5},
    });

    viewer.cameraControls.rotateLeft(Math.PI / 4);
    viewer.cameraControls.rotateUp(Math.PI / 8);

    const grid = new ROS3D.Grid({
      num_cells: 40,
      cellSize: 1.0,
      color: '#2a3038',
    });
    viewer.addObject(grid);

    window.addEventListener('resize', () => {
      if (!viewer) {
        return;
      }
      const w = viewerContainer.clientWidth;
      const h = viewerContainer.clientHeight;
      viewer.resize(w, h);
    });
  }

  function clearSubscriptions() {
    if (currentCloud) {
      if (typeof currentCloud.unsubscribe === 'function') {
        currentCloud.unsubscribe();
      }
      if (viewer && currentCloud instanceof THREE.Object3D) {
        viewer.scene.remove(currentCloud);
      }
      currentCloud = null;
    }
    if (currentGrid) {
      if (typeof currentGrid.unsubscribe === 'function') {
        currentGrid.unsubscribe();
      }
      if (currentGrid.currentMarker && viewer) {
        viewer.scene.remove(currentGrid.currentMarker);
      }
      currentGrid = null;
    }
  }

  function disconnect() {
    clearSubscriptions();
    if (tfClient) {
      tfClient.dispose();
      tfClient = null;
    }
    if (ros) {
      ros.close();
      ros = null;
    }
    setStatus('未连接');
  }

  function connect() {
    const url = wsInput.value.trim() || `ws://${window.location.hostname}:9090`;
    appendLog(`尝试连接 ${url}`);

    disconnect();

    try {
      ros = new ROSLIB.Ros({url});
    } catch (error) {
      appendLog(`连接失败: ${error.message}`);
      setStatus('连接失败');
      return;
    }

    ros.on('connection', () => {
      setStatus('已连接', true);
      appendLog('WebSocket 连接成功');
      setupScene();
    });

    ros.on('error', (err) => {
      appendLog(`WebSocket 错误: ${err}`);
      setStatus('错误');
    });

    ros.on('close', () => {
      appendLog('WebSocket 已断开');
      setStatus('已断开');
    });
  }

  function setupScene() {
    clearSubscriptions();

    if (!ros) {
      return;
    }

    tfClient = new ROSLIB.TFClient({
      ros,
      fixedFrame: 'center_camera',
      angularThres: 0.01,
      transThres: 0.01,
      rate: 30.0,
    });

    const pointTopic = pointTopicInput.value.trim();
    if (pointTopic) {
      currentCloud = new ROS3D.PointCloud2({
        ros,
        tfClient,
        rootObject: viewer.scene,
        topic: pointTopic,
        size: 0.05,
        max_pts: 200000,
        material: {size: 0.05, color: 0x55aaff},
      });
    }

    const gridTopic = gridTopicInput.value.trim();
    if (gridTopic) {
      currentGrid = new ROS3D.OccupancyGridClient({
        ros,
        rootObject: viewer.scene,
        topic: gridTopic,
        continuous: true,
        compression: 'cbor',
        color: {r: 0.2, g: 0.8, b: 0.3, a: 0.7},
      });
    }

    toggleLayers();
    appendLog('可视化订阅已建立');
  }

  function toggleLayers() {
    if (currentCloud) {
      currentCloud.visible = togglePointCheckbox.checked;
    }
    if (currentGrid && currentGrid.currentMarker) {
      currentGrid.currentMarker.visible = toggleGridCheckbox.checked;
    }
  }

  connectBtn.addEventListener('click', connect);
  reloadBtn.addEventListener('click', () => {
    appendLog('重新应用 topic 配置');
    setupScene();
  });

  togglePointCheckbox.addEventListener('change', toggleLayers);
  toggleGridCheckbox.addEventListener('change', toggleLayers);

  initViewer();

  appendLog('填写 WebSocket 地址后点击“连接”以开始');
})();
