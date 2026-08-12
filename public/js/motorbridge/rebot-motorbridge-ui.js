(function () {
  const JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'];
  const JOINT_LABELS = ['J1 底座偏航', 'J2 肩部', 'J3 肘部', 'J4 腕部俯仰', 'J5 腕部偏航', 'J6 工具旋转'];
  const JOINT_LIMITS = [
    { min: -2.8, max: 2.8 },
    { min: 0, max: 3.14 },
    { min: 0, max: 3.14 },
    { min: -1.57, max: 1.57 },
    { min: -1.57, max: 1.57 },
    { min: -3.14, max: 3.14 }
  ];
  const GRIPPER_MAX_WIDTH = 0.0715;
  const GRIPPER_CLOSE_POSITION = 0.0;
  const GRIPPER_OPEN_POSITION = 5.0;
  const MOTOR_IDS = [1, 2, 3, 4, 5, 6, 7];
  const DEFAULT_URL = 'ws://127.0.0.1:9002';
  const STATE_FRESH_MS = 180;
  const ARM_TARGET_TOLERANCE = 0.05;
  const GRIPPER_TARGET_TOLERANCE = 0.15;
  const GRIPPER_ENDPOINT_TOLERANCE = 0.35;
  const TARGET_SETTLE_MS = 250;
  const TARGET_STATUS_INTERVAL_MS = 200;
  const FEEDBACK_TRANSITION_MS = 420;

  const els = {
    url: document.getElementById('mb-url'),
    token: document.getElementById('mb-token'),
    connect: document.getElementById('mb-connect'),
    disconnect: document.getElementById('mb-disconnect'),
    status: document.getElementById('mb-status'),
    message: document.getElementById('mb-message'),
    scan: document.getElementById('mb-scan'),
    enableAll: document.getElementById('mb-enable-all'),
    disableAll: document.getElementById('mb-disable-all'),
    safeHome: document.getElementById('mb-safe-home'),
    openGripper: document.getElementById('mb-open-gripper'),
    closeGripper: document.getElementById('mb-close-gripper'),
    vlim: document.getElementById('mb-vlim'),
    locKp: document.getElementById('mb-loc-kp'),
    mirror: document.getElementById('mb-mirror'),
    syncStatus: document.getElementById('mb-sync-status'),
    log: document.getElementById('mb-log'),
    motorList: document.getElementById('mb-motor-list'),
    jointControls: document.getElementById('mb-joint-controls'),
    gripperSlider: document.getElementById('mb-gripper-slider'),
    gripperWidth: document.getElementById('mb-gripper-width')
  };

  const MB_URL_STORAGE_KEY = 'rebotarm.mb.wsUrl';

  // Restore last-used MotorBridge WS address from localStorage (empty on first run).
  if (els.url) {
    const savedUrl = localStorage.getItem(MB_URL_STORAGE_KEY);
    if (savedUrl) {
      els.url.value = savedUrl;
    }
    // Persist whenever the user edits the field.
    els.url.addEventListener('change', () => {
      const v = els.url.value.trim();
      if (v) localStorage.setItem(MB_URL_STORAGE_KEY, v);
    });
    els.url.addEventListener('input', () => {
      const v = els.url.value.trim();
      if (v) localStorage.setItem(MB_URL_STORAGE_KEY, v);
    });
  }

  if (!window.MotorBridgeClient || !els.connect) return;

  const client = new window.MotorBridgeClient({
    url: els.url ? els.url.value : DEFAULT_URL,
    channel: 'can0',
    vendor: 'robstride',
    model: 'rs-00'
  });
  window.motorBridge = client;

  let scannedMotors = [];
  let enabledMotors = new Set();
  const pendingMoveTargets = new Map();
  let commandQueueRunning = false;
  const targetVerifications = new Map();
  const loggedScanIds = new Set();
  const stateFeedback = new Map();
  const paramFeedback = new Map();
  const hydratedFeedbackMotors = new Set();
  let commandFeedbackStreamActive = false;
  let targetSequence = 0;
  let controlTransactionLocked = false;

  function isMotorBridgeControlMode() {
    return !window.reBotControlMode || window.reBotControlMode.is('motorbridge');
  }

  client.addEventListener('status', (event) => {
    const detail = event.detail || {};
    setStatus(detail.state, detail.message);
    if (detail.message) writeLog(detail.message, detail.state === 'error' ? 'error' : detail.state === 'open' ? 'ok' : 'info');
    if (detail.state === 'closed') {
      pendingMoveTargets.clear();
      cancelAllTargetVerifications('连接断开');
      commandFeedbackStreamActive = false;
      resetMirrorFeedback();
    }
  });

  client.addEventListener('state', (event) => {
    const { motorId, state } = event.detail;
    handleStateFeedback(motorId, state);
  });

  client.addEventListener('params', (event) => {
    const { motorId, params } = event.detail;
    handleParamFeedback(motorId, params);
  });

  client.addEventListener('scanprogress', (event) => {
    const detail = event.detail || {};
    if (detail.phase === 'hit' && detail.hit) {
      const id = Number(detail.hit.device_id || detail.hit.probe);
      if (!loggedScanIds.has(id)) {
        loggedScanIds.add(id);
        writeLog('扫描发现电机 ID ' + id, 'ok');
      }
    }
  });

  client.addEventListener('scanretry', (event) => {
    const detail = event.detail || {};
    writeLog('补扫缺失电机: ' + (detail.ids || []).join(','), 'info');
  });

  client.addEventListener('controlprogress', (event) => {
    const detail = event.detail || {};
    if (detail.phase === 'retry') {
      writeLog(
        'J' + detail.motorId + ' ' + controlActionLabel(detail.action) +
          '未确认' + (detail.error ? '（' + detail.error + '）' : '') +
          '，正在重试 ' + (detail.attempt + 1) + '/3',
        'info'
      );
    } else if (detail.phase === 'confirm') {
      writeLog(
        'J' + detail.motorId + ' ' + controlActionLabel(detail.action) +
          '第 1 轮发送成功，等待整批第 2 轮确认',
        'info'
      );
    } else if (detail.phase === 'sent_twice') {
      writeLog(
        'J' + detail.motorId + ' ' + controlActionLabel(detail.action) +
          '第 2 轮发送成功，等待真实状态验证',
        'info'
      );
    } else if (detail.phase === 'ok') {
      const verified = detail.verifiedState && detail.verifiedState.state;
      const actualStatus = verified && (verified.status_name || verified.status);
      writeLog(
        'J' + detail.motorId + ' ' + controlActionLabel(detail.action) +
          '确认完成（命令成功发送至少 2 次，真实状态=' +
          String(actualStatus || (detail.action === 'enable' ? 'ENABLED' : 'DISABLED')) + '）',
        'ok'
      );
    } else if (detail.phase === 'failed') {
      writeLog(
        'J' + detail.motorId + ' ' + controlActionLabel(detail.action) +
          '失败: ' + (detail.error || '无响应'),
        'error'
      );
    }
  });

  els.connect.addEventListener('click', () => {
    const url = els.url ? els.url.value.trim() : DEFAULT_URL;
    const token = els.token ? els.token.value.trim() : '';
    if (url) localStorage.setItem(MB_URL_STORAGE_KEY, url);
    client.connect(url, token);
  });

  if (els.mirror) {
    els.mirror.addEventListener('change', () => {
      if (els.mirror.checked) {
        setSyncStatus(targetVerifications.size ? '等待实机到位确认' : '等待目标指令');
      } else {
        setSyncStatus('到位后同步已暂停');
      }
    });
  }

  waitForSimApi((sim) => {
    sim.onCommand((command) => forwardSimCommandToMotorBridge(command));
  });

  els.disconnect.addEventListener('click', async () => {
    setBulkControlsDisabled(true);
    await cancelQueuedMotionAndWait();
    if (!client.connected) {
      client.disconnect();
      enabledMotors.clear();
      renderMotorList();
      setBulkControlsDisabled(false);
      return;
    }

    els.disconnect.disabled = true;
    setMessage('正在安全断开：失能电机并关闭连接...');
    writeLog('安全断开开始：先确认回零，再失能 7 个电机', 'info');
    try {
      await safeHomeBeforeMotorBridgeDisable('安全断开');
      const result = await client.safeDisconnect(MOTOR_IDS);
      enabledMotors.clear();
      renderMotorList();
      (result.warnings || []).forEach((warning) => writeLog(warning, 'info'));
      setMessage('已安全断开');
      writeLog('安全断开完成：电机已失能，MotorBridge 已断开', 'ok');
    } catch (error) {
      setMessage('安全断开失败: ' + error.message);
      writeLog('安全断开失败，连接已保留: ' + error.message, 'error');
    } finally {
      els.disconnect.disabled = false;
      setBulkControlsDisabled(false);
    }
  });

  els.scan.addEventListener('click', async () => {
    if (!client.connected) { setMessage('请先连接 MotorBridge'); return; }
    loggedScanIds.clear();
    setBulkControlsDisabled(true);
    await cancelQueuedMotionAndWait();
    setMessage('正在扫描电机...');
    writeLog('扫描开始: joints 1..7', 'info');
    try {
      const result = await client.scan(1, 7);
      scannedMotors = getScannedMotorIds(result);
      renderMotorList();
      setMessage('扫描完成: 在线 ' + scannedMotors.length + '/7');
      writeLog('扫描完成: 在线 ' + scannedMotors.length + '/7', 'ok');
      if (result.data && Array.isArray(result.data.missing_ids) && result.data.missing_ids.length) {
        writeLog('扫描仍缺失 ID: ' + result.data.missing_ids.join(','), 'error');
      }
      if (scannedMotors.length === 0) {
        writeLog('扫描诊断: ' + summarizeScanResult(result), 'info');
      }
    } catch (error) {
      setMessage('扫描失败: ' + error.message);
      writeLog('扫描失败: ' + error.message, 'error');
    } finally {
      setBulkControlsDisabled(false);
    }
  });

  els.enableAll.addEventListener('click', async () => {
    if (!client.connected) { setMessage('请先连接'); return; }
    setBulkControlsDisabled(true);
    await cancelQueuedMotionAndWait();
    setMessage('正在使能所有电机...');
    writeLog('使能全部开始', 'info');
    try {
      const result = await client.enableAll(MOTOR_IDS);
      enabledMotors = new Set(result.succeeded);
      renderMotorList();
      setMessage('使能完成: 7/7');
      writeLog('使能完成: 7/7', 'ok');
      writeLog('单连接控制已就绪：仅在执行目标时读取当前电机反馈', 'ok');
    } catch (error) {
      const summary = error.summary;
      if (summary) {
        enabledMotors = new Set(summary.succeeded);
        renderMotorList();
        setMessage('使能部分完成: ' + summary.succeeded.length + '/7');
        writeLog('使能失败 ID: ' + summary.failed.map((item) => item.id).join(','), 'error');
      } else {
        setMessage('使能失败: ' + error.message);
        writeLog('使能失败: ' + error.message, 'error');
      }
    } finally {
      setBulkControlsDisabled(false);
    }
  });

  async function clearMotorFaultAndReEnable(motorId) {
    const id = Number(motorId);
    if (!client.connected) { setMessage('请先连接 MotorBridge'); return; }
    if (!scannedMotors.includes(id)) { setMessage('J' + id + ' 当前不在线，请先扫描'); return; }

    setBulkControlsDisabled(true);
    await cancelQueuedMotionAndWait();
    enabledMotors.delete(id);
    renderMotorList();
    setMessage('正在清除 J' + id + ' 故障...');
    writeLog('J' + id + ' 清故障开始', 'info');
    try {
      await client.clearError(id);
      writeLog('J' + id + ' clear_error 成功，正在重新使能并验证真实状态', 'ok');
      await delay(180);
      const result = await client.enableAll([id]);
      if (!result.succeeded.includes(id)) {
        throw new Error('重新使能未通过真实状态验证');
      }
      enabledMotors.add(id);
      renderMotorList();
      setMessage('J' + id + ' 故障已清除并重新使能');
      writeLog('J' + id + ' 故障恢复完成：真实状态=ENABLED', 'ok');
    } catch (error) {
      enabledMotors.delete(id);
      renderMotorList();
      const detail = error.summary && error.summary.failed && error.summary.failed[0]
        ? error.summary.failed[0].error
        : error.message;
      setMessage('J' + id + ' 故障恢复失败: ' + detail);
      writeLog('J' + id + ' 故障恢复失败: ' + detail, 'error');
    } finally {
      setBulkControlsDisabled(false);
    }
  }

  els.disableAll.addEventListener('click', async () => {
    if (!client.connected) { setMessage('请先连接'); return; }
    setBulkControlsDisabled(true);
    await cancelQueuedMotionAndWait();
    setMessage('正在失能所有电机...');
    writeLog('失能全部开始：先逐轴回零并确认到位', 'info');
    try {
      await safeHomeBeforeMotorBridgeDisable('全部失能');
      const result = await client.disableAll(MOTOR_IDS);
      result.succeeded.forEach((id) => enabledMotors.delete(id));
      renderMotorList();
      setMessage('失能完成: 7/7');
      writeLog('失能完成: 7/7（已发送 MotorBridge disable）', 'ok');
    } catch (error) {
      const summary = error.summary;
      if (summary) {
        summary.succeeded.forEach((id) => enabledMotors.delete(id));
        renderMotorList();
        setMessage('失能部分完成: ' + summary.succeeded.length + '/7');
        writeLog('失能失败 ID: ' + summary.failed.map((item) => item.id).join(','), 'error');
      } else {
        setMessage('失能失败: ' + error.message);
        writeLog('失能失败: ' + error.message, 'error');
      }
    } finally {
      setBulkControlsDisabled(false);
    }
  });

  els.safeHome.addEventListener('click', async () => {
    if (!client.connected) { setMessage('请先连接'); return; }
    if (enabledMotors.size === 0) { setMessage('请先使能电机'); return; }
    pendingMoveTargets.clear();
    cancelAllTargetVerifications('安全回零替换当前目标');
    const homeIds = MOTOR_IDS.filter((id) => enabledMotors.has(id));
    homeIds.forEach((id) => queueMotorMove(id, 0, '安全回零', { vlim: 0.5, locKp: 30 }));
    setMessage(homeIds.length + ' 轴回零目标已更新，正在按顺序发送');
    writeLog('安全回零：J' + homeIds.join('/J') + ' 已进入全局串行队列', 'info');
  });

  els.openGripper.addEventListener('click', () => {
    if (!client.connected || !enabledMotors.has(7)) { setMessage('请先连接并使能夹爪'); return; }
    queueMotorMove(7, GRIPPER_OPEN_POSITION, '夹爪打开');
    setMessage('夹爪打开目标已加入 J7 队列');
  });

  els.closeGripper.addEventListener('click', () => {
    if (!client.connected || !enabledMotors.has(7)) { setMessage('请先连接并使能夹爪'); return; }
    queueMotorMove(7, GRIPPER_CLOSE_POSITION, '夹爪闭合');
    setMessage('夹爪闭合目标已加入 J7 队列');
  });

  if (els.gripperSlider) {
    els.gripperSlider.addEventListener('input', () => {
      const width = parseFloat(els.gripperSlider.value) / 1000;
      if (els.gripperWidth) els.gripperWidth.textContent = (width * 1000).toFixed(1) + ' mm';
    });
    els.gripperSlider.addEventListener('change', () => {
      if (!client.connected || !enabledMotors.has(7)) return;
      const width = parseFloat(els.gripperSlider.value) / 1000;
      queueMotorMove(7, gripperWidthToMotorPosition(width), '夹爪宽度滑块');
    });
  }

  if (window.reBotControlMode) {
    window.reBotControlMode.onChange(({ mode }) => {
      if (mode === 'motorbridge') {
        writeLog('MotorBridge 模式已启用：目标到位后再提交 3D', 'ok');
        setSyncStatus(targetVerifications.size ? '等待实机到位确认' : '等待目标指令');
        return;
      }
      pendingMoveTargets.clear();
      cancelAllTargetVerifications('切换到 ROS2');
      writeLog('已切换到 ROS2，MotorBridge 目标确认和网页控制暂停', 'warn');
    });
  }

  window.setInterval(() => {
    checkTargetVerificationTimeouts();
  }, 250);

  function renderMotorList() {
    if (!els.motorList) return;
    els.motorList.innerHTML = '';
    MOTOR_IDS.forEach((id, index) => {
      const jointName = index < 6 ? JOINT_NAMES[index] : 'gripper';
      const label = index < 6 ? JOINT_LABELS[index] : 'J7 夹爪';
      const limits = index < 6
        ? JOINT_LIMITS[index]
        : { min: GRIPPER_CLOSE_POSITION, max: GRIPPER_OPEN_POSITION };
      const online = scannedMotors.includes(id);
      const enabled = enabledMotors.has(id);
      const state = client.getMotorState(id);

      const card = document.createElement('div');
      card.className = 'mb-motor-card' + (enabled ? ' enabled' : (online ? ' online' : ''));
      card.id = 'mb-motor-' + id;

      const posText = state && typeof state.pos === 'number' ? (state.pos.toFixed(4)) : '--';
      const velText = state && typeof state.vel === 'number' ? (state.vel.toFixed(4)) : '--';
      const torqText = state && typeof state.torq === 'number' ? (state.torq.toFixed(4)) : '--';
      const statusText = state && state.status_name ? state.status_name : (enabled ? 'ENABLED' : (online ? 'ONLINE' : 'OFFLINE'));

      card.innerHTML = `
        <div class="mb-motor-header">
          <span class="mb-motor-name">${label}</span>
          <span class="mb-motor-id">CAN_ID 0x${id.toString(16)}</span>
          <span class="mb-motor-status ${statusText === 'ENABLED' ? 'ok' : ''}">${statusText}</span>
          <button type="button" class="mb-clear-error" data-motor-id="${id}"
            ${!online || controlTransactionLocked ? 'disabled' : ''}>清故障</button>
        </div>
        <div class="mb-motor-data">
          <span>位置 ${posText}</span>
          <span>速度 ${velText}</span>
          <span>力矩 ${torqText}</span>
        </div>
        <div class="mb-motor-slider">
          <input type="range" class="mb-joint-range" data-motor-id="${id}" data-joint-name="${jointName}"
            min="${limits.min}" max="${limits.max}" step="0.01" value="0"
            ${!enabled || controlTransactionLocked ? 'disabled' : ''}>
          <span class="mb-joint-value">0.00</span>
        </div>
      `;

      const range = card.querySelector('.mb-joint-range');
      const valueSpan = card.querySelector('.mb-joint-value');
      const clearErrorButton = card.querySelector('.mb-clear-error');
      if (clearErrorButton) {
        clearErrorButton.addEventListener('click', () => clearMotorFaultAndReEnable(id));
      }
      if (range && enabled) {
        range.addEventListener('input', () => {
          const pos = parseFloat(range.value);
          valueSpan.textContent = pos.toFixed(2);
        });
        range.addEventListener('change', () => {
          const pos = parseFloat(range.value);
          valueSpan.textContent = pos.toFixed(2);
          queueMotorMove(id, pos, id === 7 ? 'J7 电机滑块' : '关节滑块');
        });
      }

      els.motorList.appendChild(card);
    });
  }

  function updateMotorCard(motorId, state) {
    const card = document.getElementById('mb-motor-' + motorId);
    if (!card) return;
    const dataDiv = card.querySelector('.mb-motor-data');
    if (dataDiv && state) {
      const spans = dataDiv.querySelectorAll('span');
      if (spans.length >= 3) {
        spans[0].textContent = '位置 ' + (typeof state.pos === 'number' ? state.pos.toFixed(4) : '--');
        spans[1].textContent = '速度 ' + (typeof state.vel === 'number' ? state.vel.toFixed(4) : '--');
        spans[2].textContent = '力矩 ' + (typeof state.torq === 'number' ? state.torq.toFixed(4) : '--');
      }
    }
    const statusSpan = card.querySelector('.mb-motor-status');
    if (statusSpan && state.status_name) {
      statusSpan.textContent = state.status_name;
      statusSpan.className = 'mb-motor-status ' + (state.status_name === 'ENABLED' ? 'ok' : '');
    }
  }

  function updateMotorParams(motorId, values) {
    const card = document.getElementById('mb-motor-' + motorId);
    if (!card) return;
    const dataDiv = card.querySelector('.mb-motor-data');
    if (dataDiv && values) {
      const spans = dataDiv.querySelectorAll('span');
      if (spans.length >= 3) {
        // J1-J6 position is owned by the realtime state stream. Only the
        // gripper uses the polled mechPos register as its position source.
        if (motorId === 7 && typeof values.mechPos === 'number') {
          spans[0].textContent = '位置 ' + values.mechPos.toFixed(4);
        }
        if (typeof values.mechVel === 'number') spans[1].textContent = '速度 ' + values.mechVel.toFixed(4);
        if (typeof values.torque_fdb === 'number') spans[2].textContent = '力矩 ' + values.torque_fdb.toFixed(4);
      }
    }
  }

  function cacheStateFeedback(motorId, state) {
    const id = Number(motorId);
    if (!MOTOR_IDS.includes(id) || !state || typeof state.pos !== 'number') return;
    stateFeedback.set(id, {
      value: state.pos,
      velocity: typeof state.vel === 'number' ? state.vel : 0,
      receivedAt: performance.now()
    });
  }

  function handleStateFeedback(motorId, state) {
    updateMotorCard(motorId, state);
    cacheStateFeedback(motorId, state);
    if (state && typeof state.pos === 'number') {
      hydrateMotorPositionOnce(Number(motorId), state.pos);
      processTargetFeedback(
        Number(motorId),
        state.pos,
        typeof state.vel === 'number' ? state.vel : 0,
        'state'
      );
    }
  }

  function handleParamFeedback(motorId, params) {
    if (!params || !params.values) return;
    updateMotorParams(motorId, params.values);
    cacheParamFeedback(motorId, params.values);
    const id = Number(motorId);
    const state = stateFeedback.get(id);
    const hasFreshState = state && performance.now() - state.receivedAt <= STATE_FRESH_MS;
    if (!hasFreshState && typeof params.values.mechPos === 'number') {
      hydrateMotorPositionOnce(id, params.values.mechPos);
      processTargetFeedback(
        id,
        params.values.mechPos,
        typeof params.values.mechVel === 'number' ? params.values.mechVel : 0,
        'params'
      );
    }
  }

  function delay(ms) {
    return new Promise((resolve) => window.setTimeout(resolve, ms));
  }

  async function safeHomeBeforeMotorBridgeDisable(reason) {
    const homeIds = MOTOR_IDS.filter((id) => enabledMotors.has(id));
    if (!homeIds.length) {
      writeLog(reason + '：没有记录为已使能的电机，跳过回零', 'info');
      return;
    }
    for (const motorId of homeIds) {
      setMessage(reason + '：J' + motorId + ' 正在回零并确认');
      const result = await sendMotorTarget(motorId, 0, 0.5, 30);
      if (!result || !result.arrived) {
        throw new Error(
          'J' + motorId + ' 未确认回到零位，已中止失能，电机保持使能'
        );
      }
    }
    writeLog(reason + '：J' + homeIds.join('/J') + ' 已确认在零位', 'ok');
  }

  function cacheParamFeedback(motorId, values) {
    const id = Number(motorId);
    if (!MOTOR_IDS.includes(id) || !values || typeof values.mechPos !== 'number') return;
    paramFeedback.set(id, {
      value: values.mechPos,
      velocity: typeof values.mechVel === 'number' ? values.mechVel : 0,
      receivedAt: performance.now()
    });
  }

  function resetMirrorFeedback() {
    stateFeedback.clear();
    paramFeedback.clear();
    hydratedFeedbackMotors.clear();
    cancelAllTargetVerifications('反馈重置');
    setSyncStatus('等待目标指令');
  }

  function setSyncStatus(message) {
    if (els.syncStatus) els.syncStatus.textContent = message;
  }

  function getVlim() {
    return els.vlim ? clamp(parseFloat(els.vlim.value) || 0.25, 0.05, 0.7) : 0.25;
  }

  function getLocKp() {
    return els.locKp ? clamp(parseFloat(els.locKp.value) || 30, 1, 100) : 30;
  }

  function controlActionLabel(action) {
    return action === 'enable' ? '使能' : '失能';
  }

  function setBulkControlsDisabled(disabled) {
    controlTransactionLocked = Boolean(disabled);
    [
      els.connect,
      els.disconnect,
      els.scan,
      els.enableAll,
      els.disableAll,
      els.safeHome,
      els.openGripper,
      els.closeGripper
    ].forEach((button) => {
      if (button) button.disabled = controlTransactionLocked;
    });
    if (els.gripperSlider) {
      els.gripperSlider.disabled = controlTransactionLocked || !enabledMotors.has(7);
    }
    document.querySelectorAll('.mb-joint-range').forEach((range) => {
      const motorId = Number(range.dataset.motorId);
      range.disabled = controlTransactionLocked || !enabledMotors.has(motorId);
    });
    document.querySelectorAll('.mb-clear-error').forEach((button) => {
      const motorId = Number(button.dataset.motorId);
      button.disabled = controlTransactionLocked || !scannedMotors.includes(motorId);
    });
    document.querySelectorAll('#joint-controls input, #preset-buttons button').forEach((control) => {
      control.disabled = controlTransactionLocked;
    });
    ['open-gripper', 'close-gripper'].forEach((id) => {
      const control = document.getElementById(id);
      if (control) control.disabled = controlTransactionLocked;
    });
  }

  function gripperWidthToMotorPosition(width) {
    const ratio = clamp(Number(width) / GRIPPER_MAX_WIDTH, 0, 1);
    return GRIPPER_CLOSE_POSITION +
      ratio * (GRIPPER_OPEN_POSITION - GRIPPER_CLOSE_POSITION);
  }

  function gripperMotorPositionToWidth(position) {
    const span = GRIPPER_OPEN_POSITION - GRIPPER_CLOSE_POSITION;
    const ratio = span === 0 ? 0 : (Number(position) - GRIPPER_CLOSE_POSITION) / span;
    return clamp(ratio, 0, 1) * GRIPPER_MAX_WIDTH;
  }

  function updateGripperControl(width) {
    const safeWidth = clamp(Number(width) || 0, 0, GRIPPER_MAX_WIDTH);
    if (els.gripperSlider) els.gripperSlider.value = String(safeWidth * 1000);
    if (els.gripperWidth) els.gripperWidth.textContent = (safeWidth * 1000).toFixed(1) + ' mm';
  }

  function updateMotorPositionControls(motorId, position) {
    const id = Number(motorId);
    const safePosition = Number(position);
    if (!Number.isFinite(safePosition)) return;
    const card = document.getElementById('mb-motor-' + id);
    const range = card && card.querySelector('.mb-joint-range');
    const value = card && card.querySelector('.mb-joint-value');
    if (range) range.value = String(safePosition);
    if (value) value.textContent = safePosition.toFixed(2);
    if (id === 7) {
      updateGripperControl(gripperMotorPositionToWidth(safePosition));
    }
  }

  function queueMotorMove(motorId, position, reason, options) {
    if (controlTransactionLocked || !isMotorBridgeControlMode() || !client.connected) return false;
    const target = Number(position);
    if (!Number.isFinite(target)) return false;
    const id = Number(motorId);
    if (!enabledMotors.has(id)) return false;
    const replaced = pendingMoveTargets.has(id) || targetVerifications.has(id);
    pendingMoveTargets.set(id, {
      position: target,
      reason: reason || ('J' + id + ' 目标'),
      vlim: options && Number.isFinite(options.vlim) ? options.vlim : getVlim(),
      locKp: options && Number.isFinite(options.locKp) ? options.locKp : getLocKp()
    });
    cancelTargetVerification(id, '目标已更新');
    updateMotorPositionControls(id, target);
    setSyncStatus('J' + id + ' 目标 ' + target.toFixed(3) + ' rad · 等待发送');
    writeLog(
      (reason || ('J' + id)) + (replaced ? ' 更新队列' : ' 入队') +
        ': J' + id + '=' + target.toFixed(4) + ' rad',
      'info'
    );
    flushMotorMoves();
    return true;
  }

  function forwardSimCommandToMotorBridge(command) {
    if (!isMotorBridgeControlMode() || !command || controlTransactionLocked) return;
    if (command.type === 'joint-batch') {
      const joints = command.joints && typeof command.joints === 'object' ? command.joints : {};
      const queued = [];
      JOINT_NAMES.forEach((name, index) => {
        const value = Number(joints[name]);
        if (Number.isFinite(value) && queueMotorMove(index + 1, value, command.label || '3D 批量目标')) {
          queued.push(index + 1);
        }
      });
      const gripperWidth = Number(joints.gripper);
      if (Number.isFinite(gripperWidth) && queueMotorMove(
        7,
        gripperWidthToMotorPosition(gripperWidth),
        command.label || '3D 夹爪目标'
      )) {
        queued.push(7);
      }
      if (queued.length) {
        writeLog(
          (command.label || command.source || '3D 批量目标') +
            ' -> MotorBridge 串行队列 J' + queued.join('/J'),
          'ok'
        );
      }
      return;
    }

    if (command.type !== 'joint' || command.phase === 'input') return;
    if (command.name === 'gripper') {
      queueMotorMove(7, gripperWidthToMotorPosition(command.value), '3D 夹爪按钮/滑块');
      return;
    }
    const jointIndex = JOINT_NAMES.indexOf(command.name);
    if (jointIndex >= 0) {
      queueMotorMove(jointIndex + 1, command.value, '3D 关节滑块');
    }
  }

  async function flushMotorMoves() {
    if (commandQueueRunning) return;
    commandQueueRunning = true;
    try {
      while (pendingMoveTargets.size) {
        if (!isMotorBridgeControlMode() || !client.connected) {
          pendingMoveTargets.clear();
          break;
        }
        const next = pendingMoveTargets.entries().next().value;
        if (!next) break;
        const motorId = next[0];
        const command = next[1];
        pendingMoveTargets.delete(motorId);
        try {
          await sendMotorTarget(
            motorId,
            command.position,
            command.vlim,
            command.locKp
          );
        } catch (error) {
          if (!pendingMoveTargets.has(motorId)) {
            restoreMotorPositionControls(motorId);
          }
          setMessage('J' + motorId + ' 控制失败: ' + error.message);
          writeLog(
            command.reason + ' 发送失败: J' + motorId + ' · ' + error.message,
            'error'
          );
        }
      }
    } finally {
      commandQueueRunning = false;
      if (pendingMoveTargets.size && isMotorBridgeControlMode() && client.connected) {
        flushMotorMoves();
      }
    }
  }

  async function cancelQueuedMotionAndWait() {
    pendingMoveTargets.clear();
    cancelAllTargetVerifications('控制事务取消');
    while (commandQueueRunning) {
      await delay(25);
    }
  }

  function restoreMotorPositionControls(motorId) {
    const id = Number(motorId);
    const feedback = getLatestFeedback(id);
    if (feedback && Number.isFinite(feedback.value)) {
      syncFeedbackPosition(id, feedback.value);
    }
  }

  function hydrateMotorPositionOnce(motorId, position) {
    const id = Number(motorId);
    if (
      hydratedFeedbackMotors.has(id) ||
      !enabledMotors.has(id) ||
      !isMotorBridgeControlMode() ||
      pendingMoveTargets.has(id) ||
      targetVerifications.has(id) ||
      !Number.isFinite(position)
    ) return;
    hydratedFeedbackMotors.add(id);
    syncFeedbackPosition(id, position);
  }

  function getLatestFeedback(motorId) {
    const state = stateFeedback.get(motorId);
    const params = paramFeedback.get(motorId);
    if (!state) return params || null;
    if (!params) return state;
    return state.receivedAt >= params.receivedAt ? state : params;
  }

  async function sendMotorTarget(motorId, target, vlim, locKp) {
    const verification = beginTargetVerification(motorId, target, vlim);
    try {
      await client.movePosVel(motorId, target, vlim, locKp);
      verification.commandAccepted = true;
      if (verification.settled) return verification.completion;
      if (targetVerifications.get(motorId) === verification) {
        setSyncStatus('J' + motorId + ' 指令已发送 · 等待实机到位');
        writeLog(
          'J' + motorId + ' 指令已发送: target=' + Number(target).toFixed(4) + ' rad',
          'info'
        );
      }
      await client.startParamStream(100);
      commandFeedbackStreamActive = true;
      return await verification.completion;
    } catch (error) {
      if (targetVerifications.get(motorId) === verification) {
        targetVerifications.delete(motorId);
      }
      finishTargetVerification(verification, { arrived: false, error: error.message });
      throw error;
    } finally {
      if (commandFeedbackStreamActive) {
        commandFeedbackStreamActive = false;
        if (client.connected) {
          try {
            await client.stopParamStream();
          } catch (error) {
            writeLog('J' + motorId + ' 停止反馈读取失败: ' + error.message, 'error');
          }
        }
      }
    }
  }

  function beginTargetVerification(motorId, target, vlim) {
    const id = Number(motorId);
    const now = performance.now();
    const feedback = getLatestFeedback(id);
    const distance = feedback && Number.isFinite(feedback.value)
      ? Math.abs(target - feedback.value)
      : (id === 7 ? GRIPPER_OPEN_POSITION - GRIPPER_CLOSE_POSITION : 6.28);
    const expectedMs = distance / Math.max(Number(vlim) || 0.05, 0.05) * 2000;
    const isGripperEndpoint = id === 7 && (
      Math.abs(target - GRIPPER_OPEN_POSITION) < 0.001 ||
      Math.abs(target - GRIPPER_CLOSE_POSITION) < 0.001
    );
    let resolveCompletion;
    const completion = new Promise((resolve) => {
      resolveCompletion = resolve;
    });
    const verification = {
      sequence: ++targetSequence,
      motorId: id,
      target: Number(target),
      tolerance: isGripperEndpoint
        ? GRIPPER_ENDPOINT_TOLERANCE
        : id === 7
        ? GRIPPER_TARGET_TOLERANCE
        : ARM_TARGET_TOLERANCE,
      createdAt: now,
      deadlineAt: now + clamp(6000 + expectedMs, 8000, 60000),
      insideSince: 0,
      stableValues: [],
      lastStatusAt: 0,
      lastPosition: null,
      lastError: null,
      commandAccepted: false,
      completion,
      resolveCompletion,
      settled: false
    };
    targetVerifications.set(id, verification);
    setSyncStatus('J' + id + ' 目标 ' + Number(target).toFixed(3) + ' rad · 正在下发');
    return verification;
  }

  function finishTargetVerification(verification, result) {
    if (!verification || verification.settled) return;
    verification.settled = true;
    verification.resolveCompletion(result || { arrived: false });
  }

  function cancelTargetVerification(motorId, reason) {
    const id = Number(motorId);
    const verification = targetVerifications.get(id);
    if (!verification) return;
    targetVerifications.delete(id);
    finishTargetVerification(verification, { arrived: false, cancelled: true, reason });
  }

  function cancelAllTargetVerifications(reason) {
    Array.from(targetVerifications.keys()).forEach((motorId) => {
      cancelTargetVerification(motorId, reason);
    });
  }

  function processTargetFeedback(motorId, position, velocity, source) {
    if (!isMotorBridgeControlMode()) return;
    const verification = targetVerifications.get(motorId);
    if (!verification || !verification.commandAccepted || !Number.isFinite(position)) return;

    const now = performance.now();
    const error = Math.abs(position - verification.target);
    verification.lastPosition = position;
    verification.lastError = error;

    const entryTolerance = verification.tolerance;
    const holdTolerance = verification.insideSince ? entryTolerance * 1.5 : entryTolerance;
    const insideWindow = error <= holdTolerance;

    if (now - verification.lastStatusAt >= TARGET_STATUS_INTERVAL_MS) {
      verification.lastStatusAt = now;
      const dwell = verification.insideSince
        ? Math.min(Math.round(now - verification.insideSince), TARGET_SETTLE_MS)
        : 0;
      setSyncStatus(
        'J' + motorId + ' 验证中 · 目标 ' + verification.target.toFixed(3) +
          ' · 实际 ' + position.toFixed(3) +
          ' · 误差 ' + error.toFixed(3) + '/' + entryTolerance.toFixed(3) +
          ' · 稳定 ' + dwell + '/' + TARGET_SETTLE_MS + ' ms'
      );
    }

    if (!insideWindow) {
      verification.insideSince = 0;
      verification.stableValues.length = 0;
      return;
    }

    if (!verification.insideSince) verification.insideSince = now;
    verification.stableValues.push(position);
    if (verification.stableValues.length > 12) verification.stableValues.shift();
    if (
      now - verification.insideSince < TARGET_SETTLE_MS ||
      verification.stableValues.length < 3
    ) return;

    const finalPosition = verification.stableValues.reduce((sum, value) => sum + value, 0) /
      verification.stableValues.length;
    targetVerifications.delete(motorId);
    finishTargetVerification(verification, { arrived: true, position: finalPosition });
    const syncedTo3D = syncFeedbackPosition(motorId, finalPosition);

    const finalError = Math.abs(finalPosition - verification.target);
    const sourceLabel = source === 'state' ? '状态流' : '参数反馈';
    setMessage('J' + motorId + ' 已到位，误差 ' + finalError.toFixed(4) + ' rad');
    writeLog(
      'J' + motorId + ' 到位确认: target=' + verification.target.toFixed(4) +
        ', actual=' + finalPosition.toFixed(4) +
        ', error=' + finalError.toFixed(4) + ' rad（' + sourceLabel + '）',
      'ok'
    );
    setSyncStatus(
      !syncedTo3D
        ? 'J' + motorId + ' 已到位 · 3D 同步已暂停'
        : targetVerifications.size
        ? 'J' + motorId + ' 已同步 · 仍有 ' + targetVerifications.size + ' 轴待确认'
        : 'J' + motorId + ' 已到位并同步 3D'
    );
  }

  function syncFeedbackPosition(motorId, position) {
    hydratedFeedbackMotors.add(Number(motorId));
    updateMotorPositionControls(motorId, position);
    if (
      !els.mirror ||
      !els.mirror.checked ||
      !window.reBotSim ||
      typeof window.reBotSim.setAngles !== 'function'
    ) return false;
    const snapshot = {};
    if (motorId === 7) {
      snapshot.gripper = gripperMotorPositionToWidth(position);
    } else {
      snapshot['joint' + motorId] = position;
    }
    window.reBotSim.setAngles(snapshot, {
      source: 'motorbridge',
      emit: false,
      animate: true,
      duration: FEEDBACK_TRANSITION_MS
    });
    return true;
  }

  function checkTargetVerificationTimeouts() {
    if (!targetVerifications.size) return;
    const now = performance.now();
    targetVerifications.forEach((verification, motorId) => {
      if (now <= verification.deadlineAt) return;
      targetVerifications.delete(motorId);
      finishTargetVerification(verification, {
        arrived: false,
        timeout: true,
        position: verification.lastPosition
      });
      if (Number.isFinite(verification.lastPosition)) {
        syncFeedbackPosition(motorId, verification.lastPosition);
      } else {
        restoreMotorPositionControls(motorId);
      }
      writeLog(
        'J' + motorId + ' 到位确认超时: target=' + verification.target.toFixed(4) +
          (Number.isFinite(verification.lastPosition)
            ? ', actual=' + verification.lastPosition.toFixed(4) +
              ', error=' + verification.lastError.toFixed(4) + ' rad'
            : ', 尚未收到有效位置反馈'),
        'error'
      );
      setSyncStatus('J' + motorId + ' 未在时限内到位');
    });
  }

  function setStatus(state, message) {
    if (!els.status) return;
    els.status.className = 'mini-pill';
    if (state === 'open') {
      els.status.classList.add('online');
      els.status.textContent = '在线';
    } else if (state === 'connecting') {
      els.status.classList.add('warn');
      els.status.textContent = '连接中';
    } else if (state === 'error') {
      els.status.classList.add('error');
      els.status.textContent = '错误';
    } else {
      els.status.textContent = '离线';
    }
    setMessage(message);
  }

  function setMessage(message) {
    if (els.message) els.message.textContent = message || '';
  }

  function waitForSimApi(callback) {
    if (window.reBotSim && typeof window.reBotSim.onCommand === 'function') {
      callback(window.reBotSim);
      return;
    }
    window.setTimeout(() => waitForSimApi(callback), 50);
  }

  function writeLog(message, level) {
    if (!els.log || !message) return;
    const line = document.createElement('div');
    line.className = 'ros-log-line ' + (level || 'info');
    const now = new Date();
    line.innerHTML = '<time>' + now.toLocaleTimeString() + '</time><span></span>';
    line.querySelector('span').textContent = String(message);
    els.log.prepend(line);
    while (els.log.children.length > 80) els.log.lastElementChild.remove();
  }

  function clamp(value, min, max) {
    return Math.max(min, Math.min(max, value));
  }

  function getScannedMotorIds(result) {
    const data = result && result.data;
    const candidates = result && (
      result.motors ||
      result.results ||
      (data && data.hits) ||
      data
    );
    const items = Array.isArray(candidates) ? candidates : [];
    const ids = items.map((item) => {
      if (typeof item === 'number') return item;
      if (!item || typeof item !== 'object') return null;
      return item.device_id || item.motor_id || item.probe || item.esc_id || item.id || null;
    });
    return Array.from(new Set(ids.map(Number).filter((id) => MOTOR_IDS.includes(id)))).sort((a, b) => a - b);
  }

  function summarizeScanResult(result) {
    if (!result) return '网关未返回结果';
    const data = result.data || {};
    const hits = Array.isArray(data.hits) ? data.hits.length : 0;
    return 'probes=' + (result.probes || 0) +
      ', hits=' + hits +
      ', gateway_count=' + (data.count === undefined ? '?' : data.count) +
      ', transport=' + (data.transport || '?') +
      ', feedback_ids=' + JSON.stringify(data.feedback_ids || []) +
      ', ok=' + String(result.ok);
  }

  renderMotorList();
})();
