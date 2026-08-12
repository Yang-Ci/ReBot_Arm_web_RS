(function () {
  class MotorBridgeClient extends EventTarget {
    constructor(options) {
      super();
      this.url = options && options.url ? options.url : 'ws://127.0.0.1:9002';
      this.token = options && options.token ? options.token : '';
      this.channel = options && options.channel ? options.channel : 'can0';
      this.vendor = options && options.vendor ? options.vendor : 'robstride';
      this.model = options && options.model ? options.model : 'rs-00';
      this.socket = null;
      this.connected = false;
      this.autoReconnect = false;
      this.reconnectDelay = 2000;
      this._reqId = 1;
      this._pending = new Map();
      this._motorStates = new Map();
      this._scanResults = [];
      this._connectSeq = 0;
      this._targetMotorId = null;
      this._stateStreamOn = false;
      this._activeReportMotorIds = [];
      this._paramStreamOn = false;
    }

    connect(url, token) {
      if (url) this.url = url;
      if (token !== undefined) this.token = token;
      this._manualClose = false;
      if (this.socket && this.socket.readyState === WebSocket.OPEN) {
        this._emitStatus('open', '已连接');
        return;
      }
      if (this.socket && this.socket.readyState === WebSocket.CONNECTING) return;

      const seq = ++this._connectSeq;
      let fullUrl = this.url;
      if (this.token) {
        const separator = this.url.includes('?') ? '&' : '?';
        fullUrl += separator + 'motorbridge_ws_token=' + encodeURIComponent(this.token);
      }
      this._emitStatus('connecting', '正在连接 ' + this.url);
      this.socket = new WebSocket(fullUrl);
      const socket = this.socket;

      socket.addEventListener('open', () => {
        if (seq !== this._connectSeq || socket !== this.socket) return;
        this.connected = true;
        this._emitStatus('open', 'MotorBridge 已连接');
      });

      socket.addEventListener('message', (event) => {
        if (seq === this._connectSeq && socket === this.socket) this._handleMessage(event);
      });

      socket.addEventListener('error', () => {
        if (seq === this._connectSeq && socket === this.socket)
          this._emitStatus('error', 'MotorBridge WebSocket 出错');
      });

      socket.addEventListener('close', () => {
        if (seq !== this._connectSeq || socket !== this.socket) return;
        this.connected = false;
        this._rejectAllPending('连接已断开');
        this._emitStatus('closed', 'MotorBridge 已断开');
        if (!this._manualClose && this.autoReconnect) {
          window.setTimeout(() => this.connect(), this.reconnectDelay);
        }
      });
    }

    disconnect() {
      this._manualClose = true;
      this.autoReconnect = false;
      const socket = this.socket;
      ++this._connectSeq;
      this.socket = null;
      this.connected = false;
      this._targetMotorId = null;
      this._stateStreamOn = false;
      this._activeReportMotorIds = [];
      this._paramStreamOn = false;
      this._rejectAllPending('连接已断开');
      if (socket && socket.readyState < WebSocket.CLOSING) socket.close();
      this._emitStatus('closed', 'MotorBridge 已断开');
    }

    async safeDisconnect(motorIds) {
      if (!this.connected) {
        this.disconnect();
        return { warnings: [] };
      }

      const ids = motorIds || [1, 2, 3, 4, 5, 6, 7];
      const warnings = [];
      if (this._paramStreamOn) {
        try {
          await this.stopParamStream();
        } catch (error) {
          warnings.push('停止参数流失败: ' + error.message);
        }
      }
      if (this._stateStreamOn) {
        try {
          await this.stopStateStream();
        } catch (error) {
          warnings.push('停止状态流失败: ' + error.message);
        }
      }

      await this.disableAll(ids);
      try {
        await this.closeBus();
      } catch (error) {
        warnings.push('关闭 CAN 会话失败: ' + error.message);
      }

      this.disconnect();
      return { warnings };
    }

    async scan(startId, endId) {
      const start = startId || 1;
      const end = endId || 7;
      if (this._paramStreamOn) {
        try {
          await this.stopParamStream();
        } catch (error) {
          // Continue with the scan; closeBus below will still stop the stream.
        }
      }
      if (this._stateStreamOn) {
        try {
          await this.stopStateStream();
        } catch (error) {
          // Continue with the scan; closeBus below will reset the session.
        }
      }
      // A browser refresh can close old telemetry sockets before they disable
      // the motor-side active report setting. Clear any leftover reporters so
      // their unsolicited status frames do not collide with scan responses.
      for (let id = start; id <= end; id += 1) {
        try {
          await this.setTarget(id);
          await this._send({
            op: 'set_active_report',
            req_id: this._nextId(),
            enabled: false
          });
        } catch (error) {
          // Offline IDs are expected during a scan; continue probing all IDs.
        }
      }
      try {
        await this.closeBus();
      } catch (error) {
        // A standby gateway may not have an open bus yet.
      }
      this._stateStreamOn = false;
      this._activeReportMotorIds = [];
      this._paramStreamOn = false;

      const ids = [];
      for (let id = start; id <= end; id += 1) ids.push(id);
      const hitsById = new Map();
      const attempts = {};
      let lastResult = null;

      for (let round = 1; round <= 2; round += 1) {
        const pendingIds = ids.filter((id) => !hitsById.has(id));
        if (pendingIds.length === 0) break;
        if (round > 1) {
          this.dispatchEvent(new CustomEvent('scanretry', {
            detail: { round, ids: pendingIds.slice() }
          }));
          await this._delay(120);
        }

        for (const id of pendingIds) {
          attempts[id] = (attempts[id] || 0) + 1;
          try {
            const result = await this._send({
              op: 'scan',
              req_id: this._nextId(),
              vendor: this.vendor,
              transport: 'auto',
              channel: this.channel,
              model: this.model,
              start_id: id,
              end_id: id,
              timeout_ms: round === 1 ? 120 : 180,
              feedback_ids: [253, 255, 254, 0, 170]
            });
            lastResult = result;
            const hits = result && result.data && Array.isArray(result.data.hits)
              ? result.data.hits
              : [];
            for (const hit of hits) {
              const hitId = Number(
                hit && (hit.device_id || hit.motor_id || hit.probe || hit.esc_id || hit.id)
              );
              if (hitId >= start && hitId <= end) hitsById.set(hitId, hit);
            }
          } catch (error) {
            // A missed probe is retried in the next round.
          }
          await this._delay(45);
        }
      }

      const hits = Array.from(hitsById.values()).sort((a, b) => {
        const aId = Number(a.device_id || a.motor_id || a.probe || a.esc_id || a.id);
        const bId = Number(b.device_id || b.motor_id || b.probe || b.esc_id || b.id);
        return aId - bId;
      });
      const result = lastResult || { ok: true, op: 'scan', data: {} };
      result.ok = true;
      result.data = Object.assign({}, result.data || {}, {
        count: hits.length,
        hits,
        attempts,
        missing_ids: ids.filter((id) => !hitsById.has(id))
      });
      this._scanResults = hits;
      result.probes = end - start + 1;
      return result;
    }

    enableMotor(motorId) {
      return this.setTarget(motorId).then(() => {
        return this._send({
          op: 'enable',
          req_id: this._nextId(),
          vendor: this.vendor,
          motor_id: motorId,
          feedback_id: 253,
          timeout_ms: 1000
        });
      });
    }

    disableMotor(motorId) {
      return this.setTarget(motorId).then(() => {
        return this._send({
          op: 'disable',
          req_id: this._nextId(),
          vendor: this.vendor,
          motor_id: motorId,
          feedback_id: 253,
          timeout_ms: 1000
        });
      });
    }

    clearError(motorId) {
      return this.setTarget(motorId).then(() => {
        return this._send({
          op: 'clear_error',
          req_id: this._nextId(),
          vendor: this.vendor,
          motor_id: motorId,
          feedback_id: 253,
          timeout_ms: 1000
        });
      });
    }

    async setTarget(motorId, feedbackId) {
      const result = await this._send({
        op: 'set_target',
        req_id: this._nextId(),
        vendor: this.vendor,
        channel: this.channel,
        model: this.model,
        motor_id: motorId,
        feedback_id: feedbackId || 253
      });
      this._targetMotorId = Number(motorId);
      return result;
    }

    movePosVel(motorId, pos, vlim, locKp) {
      return this._send({
        op: 'set_target',
        req_id: this._nextId(),
        vendor: this.vendor,
        channel: this.channel,
        model: this.model,
        motor_id: motorId,
        feedback_id: 253
      }).then(() => {
        return this._send({
          op: 'pos_vel',
          req_id: this._nextId(),
          vendor: this.vendor,
          continuous: false,
          pos: pos,
          vlim: vlim || 1.0,
          ensure_timeout_ms: 2000,
          loc_kp: locKp || 30.0
        });
      });
    }

    async enableAll(motorIds) {
      const ids = motorIds || [1, 2, 3, 4, 5, 6, 7];
      return this._controlAll('enable', ids);
    }

    async disableAll(motorIds) {
      const ids = motorIds || [1, 2, 3, 4, 5, 6, 7];
      return this._controlAll('disable', ids);
    }

    async startStateStream(motorId) {
      const requestedId = Array.isArray(motorId) ? motorId[0] : motorId;
      const id = Number(requestedId || this._targetMotorId);
      if (!Number.isInteger(id) || id <= 0) {
        throw new Error('状态流需要指定 motor_id');
      }
      if (this._stateStreamOn && this._activeReportMotorIds[0] === id) {
        return { succeeded: [id], failed: [] };
      }

      await this.setTarget(id);
      await this._send({
        op: 'set_active_report',
        req_id: this._nextId(),
        enabled: true
      });

      await this._send({
        op: 'state_stream',
        req_id: this._nextId(),
        enabled: true
      });
      this._stateStreamOn = true;
      this._activeReportMotorIds = [id];
      return { succeeded: [id], failed: [] };
    }

    async stopStateStream() {
      await this._send({
        op: 'state_stream',
        req_id: this._nextId(),
        enabled: false
      });

      const failed = [];
      for (const id of this._activeReportMotorIds) {
        try {
          await this.setTarget(id);
          await this._send({
            op: 'set_active_report',
            req_id: this._nextId(),
            enabled: false
          });
        } catch (error) {
          failed.push({ id, error: error.message });
        }
      }
      this._stateStreamOn = false;
      this._activeReportMotorIds = [];
      return { failed };
    }

    async verifyMotorEnabledState(motorId, expectedEnabled, timeoutMs) {
      const id = Number(motorId);
      const waitMs = Math.max(Number(timeoutMs) || 1500, 300);
      let timer = null;
      let finishState;
      const statePromise = new Promise((resolve) => {
        finishState = resolve;
      });
      const onState = (event) => {
        const detail = event.detail || {};
        if (Number(detail.motorId) !== id || !detail.state) return;
        finishState(detail.state);
      };

      this.addEventListener('state', onState);
      timer = window.setTimeout(() => finishState(null), waitMs);
      try {
        await this.startStateStream(id);
        const state = await statePromise;
        if (!state) throw new Error('未收到真实状态帧');
        const actualEnabled = this._parseEnabledState(state);
        if (actualEnabled === null) {
          throw new Error(
            '状态无法识别: ' +
              String(state.status_name || state.status || state.status_code || 'unknown')
          );
        }
        if (actualEnabled !== Boolean(expectedEnabled)) {
          throw new Error(
            '真实状态为 ' + (actualEnabled ? 'ENABLED' : 'DISABLED') +
              '，期望 ' + (expectedEnabled ? 'ENABLED' : 'DISABLED')
          );
        }
        return { motorId: id, enabled: actualEnabled, state };
      } finally {
        if (timer) window.clearTimeout(timer);
        this.removeEventListener('state', onState);
        try {
          await this._send({
            op: 'state_stream',
            req_id: this._nextId(),
            enabled: false
          });
        } catch (error) {
          // Continue with motor-side active-report cleanup.
        }
        try {
          await this.setTarget(id);
          await this._send({
            op: 'set_active_report',
            req_id: this._nextId(),
            enabled: false
          });
        } catch (error) {
          // The next scan also clears any leftover active-report setting.
        }
        this._stateStreamOn = false;
        this._activeReportMotorIds = [];
      }
    }

    _parseEnabledState(state) {
      if (!state || typeof state !== 'object') return null;
      if (typeof state.enabled === 'boolean') return state.enabled;
      const name = String(state.status_name || state.status || '')
        .trim()
        .toUpperCase();
      if (['ENABLED', 'RUN', 'RUNNING', 'MOTOR'].includes(name)) return true;
      if (['DISABLED', 'RESET', 'STOP', 'STOPPED', 'IDLE'].includes(name)) return false;
      if (name.includes('DISABLE') || name.includes('RESET') || name.includes('STOP')) return false;
      if (name.includes('ENABLE') || name.startsWith('MOTOR')) return true;
      return null;
    }

    async startParamStream(intervalMs) {
      if (this._paramStreamOn) return Promise.resolve();
      await this._send({
        op: 'param_stream',
        req_id: this._nextId(),
        enabled: true,
        profile: 'realtime',
        interval_ms: intervalMs || 100,
        timeout_ms: 80
      });
      this._paramStreamOn = true;
    }

    async stopParamStream() {
      await this._send({
        op: 'param_stream',
        req_id: this._nextId(),
        enabled: false
      });
      this._paramStreamOn = false;
    }

    async closeBus() {
      const result = await this._send({
        op: 'close_bus',
        req_id: this._nextId()
      });
      this._stateStreamOn = false;
      this._activeReportMotorIds = [];
      this._paramStreamOn = false;
      return result;
    }

    getMotorState(motorId) {
      return this._motorStates.get(motorId) || null;
    }

    getAllStates() {
      const result = {};
      this._motorStates.forEach((state, id) => {
        result[id] = state;
      });
      return result;
    }

    _handleMessage(event) {
      let data;
      try {
        data = JSON.parse(event.data);
      } catch (error) {
        return;
      }

      if (data.type === 'scan_progress') {
        this.dispatchEvent(new CustomEvent('scanprogress', { detail: data.data || {} }));
      }

      const isFinalResponse = typeof data.ok === 'boolean' ||
        (Boolean(data.error) && data.type !== 'scan_progress');
      if (isFinalResponse && data.req_id !== undefined && this._pending.has(data.req_id)) {
        const pending = this._pending.get(data.req_id);
        this._pending.delete(data.req_id);
        if (data.ok === false || data.error) {
          pending.reject(new Error(data.error));
        } else {
          pending.resolve(data);
        }
      }

      if (data.type === 'state' && data.data) {
        const motorId = data.data.device_id || data.data.motor_id || this._targetMotorId;
        if (motorId && data.data.has_value !== false) {
          this._motorStates.set(motorId, data.data);
          this.dispatchEvent(new CustomEvent('state', {
            detail: { motorId, state: data.data }
          }));
        }
      }

      if (data.type === 'robstride_params' && data.data) {
        const motorId = data.data.motor_id;
        if (motorId) {
          const existing = this._motorStates.get(motorId) || {};
          const merged = Object.assign({}, existing, {
            mechPos: data.data.values ? data.data.values.mechPos : undefined,
            mechVel: data.data.values ? data.data.values.mechVel : undefined,
            torque_fdb: data.data.values ? data.data.values.torque_fdb : undefined,
            VBUS: data.data.values ? data.data.values.VBUS : undefined,
            run_mode: data.data.values ? data.data.values.run_mode : undefined
          });
          this._motorStates.set(motorId, merged);
          this.dispatchEvent(new CustomEvent('params', {
            detail: { motorId, params: data.data }
          }));
        }
      }

      if (data.type === 'scan_result' || data.op === 'scan_result') {
        this._scanResults = data.motors || data.results || (data.data && data.data.hits) || [];
        this.dispatchEvent(new CustomEvent('scan', { detail: data }));
      }
    }

    _send(payload) {
      return new Promise((resolve, reject) => {
        if (!this.connected) {
          reject(new Error('MotorBridge 未连接'));
          return;
        }
        const reqId = payload.req_id || this._nextId();
        payload.req_id = reqId;
        this._pending.set(reqId, { resolve, reject });
        this.socket.send(JSON.stringify(payload));

        window.setTimeout(() => {
          if (this._pending.has(reqId)) {
            this._pending.delete(reqId);
            reject(new Error('请求超时'));
          }
        }, 5000);
      });
    }

    _nextId() {
      return ++this._reqId;
    }

    async _controlAll(action, motorIds) {
      const ids = Array.from(motorIds || []);
      const succeeded = [];
      const failed = [];
      const attempts = {};
      const results = {};
      const verifiedStates = {};
      const successCounts = {};
      const lastErrors = {};
      const operation = action === 'enable'
        ? (id) => this.enableMotor(id)
        : (id) => this.disableMotor(id);

      const sendAttempt = async (id) => {
        const attempt = (attempts[id] || 0) + 1;
        attempts[id] = attempt;
        this.dispatchEvent(new CustomEvent('controlprogress', {
          detail: { action, motorId: id, attempt, phase: 'start' }
        }));
        try {
          const response = await operation(id);
          if (!results[id]) results[id] = [];
          results[id].push(response);
          successCounts[id] = (successCounts[id] || 0) + 1;
          lastErrors[id] = null;
          this.dispatchEvent(new CustomEvent('controlprogress', {
            detail: {
              action,
              motorId: id,
              attempt,
              phase: successCounts[id] < 2 ? 'confirm' : 'sent_twice'
            }
          }));
          return true;
        } catch (error) {
          lastErrors[id] = error;
          this.dispatchEvent(new CustomEvent('controlprogress', {
            detail: {
              action,
              motorId: id,
              attempt,
              phase: attempt < 3 ? 'retry' : 'failed',
              error: error.message
            }
          }));
          return false;
        }
      };

      // Batch phase 1 and 2: every motor gets one command before the next pass.
      // Do not interleave active-report verification while later IDs are still
      // waiting for their enable/disable command.
      for (let pass = 1; pass <= 2; pass += 1) {
        for (const id of ids) {
          if ((successCounts[id] || 0) < 2) {
            await sendAttempt(id);
            await this._delay(90);
          }
        }
      }

      for (const id of ids) {
        let verifiedState = null;
        if ((successCounts[id] || 0) >= 2) {
          try {
            verifiedState = await this.verifyMotorEnabledState(id, action === 'enable', 1500);
          } catch (error) {
            lastErrors[id] = error;
          }
        }

        // The third and final attempt is used only when two successful sends
        // were not reached, or when the first real-state verification failed.
        if (!verifiedState && (attempts[id] || 0) < 3) {
          this.dispatchEvent(new CustomEvent('controlprogress', {
            detail: {
              action,
              motorId: id,
              attempt: attempts[id] || 0,
              phase: 'retry',
              error: lastErrors[id] ? lastErrors[id].message : '成功发送不足 2 次'
            }
          }));
          await this._delay(180);
          await sendAttempt(id);
          if ((successCounts[id] || 0) >= 2) {
            try {
              verifiedState = await this.verifyMotorEnabledState(id, action === 'enable', 1500);
            } catch (error) {
              lastErrors[id] = error;
            }
          }
        }

        if (verifiedState) {
          verifiedStates[id] = verifiedState;
          succeeded.push(id);
          this.dispatchEvent(new CustomEvent('controlprogress', {
            detail: {
              action,
              motorId: id,
              attempt: attempts[id],
              phase: 'ok',
              verifiedState
            }
          }));
        } else {
          const errorMessage = lastErrors[id]
            ? lastErrors[id].message
            : '控制命令成功发送不足 2 次或真实状态未确认';
          failed.push({ id, error: errorMessage });
          this.dispatchEvent(new CustomEvent('controlprogress', {
            detail: {
              action,
              motorId: id,
              attempt: attempts[id] || 0,
              phase: 'failed',
              error: errorMessage
            }
          }));
        }
        await this._delay(90);
      }

      const summary = { action, succeeded, failed, attempts, results, verifiedStates };
      if (failed.length > 0) {
        const error = new Error(
          action + ' 部分失败，电机 ID: ' + failed.map((item) => item.id).join(',')
        );
        error.summary = summary;
        throw error;
      }
      return summary;
    }

    _delay(ms) {
      return new Promise((resolve) => window.setTimeout(resolve, ms));
    }

    _rejectAllPending(message) {
      this._pending.forEach((pending) => pending.reject(new Error(message)));
      this._pending.clear();
    }

    _emitStatus(state, message) {
      this.dispatchEvent(new CustomEvent('status', {
        detail: { state, message }
      }));
    }
  }

  window.MotorBridgeClient = MotorBridgeClient;
})();
