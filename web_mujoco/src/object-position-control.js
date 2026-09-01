export const OBJECT_MOVE_STEP = 0.015;

const TABLE_BOUNDS = {
  minX: 0.13,
  maxX: 0.63,
  minY: -0.45,
  maxY: 0.45
};

const EDGE_CLEARANCE = 0.008;
const OBJECT_CLEARANCE = 0.004;

export const MOVABLE_OBJECTS = [
  {
    id: 'red',
    body: 'red_cube',
    halfX: 0.0225,
    halfY: 0.0225,
    restZ: 0.1225
  },
  {
    id: 'blue',
    body: 'blue_block',
    halfX: 0.026,
    halfY: 0.016,
    restZ: 0.119
  },
  {
    id: 'yellow',
    body: 'yellow_cylinder',
    halfX: 0.020,
    halfY: 0.020,
    restZ: 0.126
  }
];

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

export function createObjectPositionController({ mujoco, model, data, onChange }) {
  const bodyType = mujoco.mjtObj.mjOBJ_BODY.value;
  const objects = MOVABLE_OBJECTS.map((spec) => {
    const bodyId = mujoco.mj_name2id(model, bodyType, spec.body);
    if (bodyId < 0) throw new Error(`物体位置控制缺少 body：${spec.body}`);
    const jointAddress = model.body_jntadr[bodyId];
    return {
      ...spec,
      bodyId,
      qposAddress: model.jnt_qposadr[jointAddress],
      dofAddress: model.jnt_dofadr[jointAddress]
    };
  });

  let enabled = false;
  let selectedId = objects[0].id;
  let lastResult = null;

  function selected() {
    return objects.find((object) => object.id === selectedId) || objects[0];
  }

  function positionOf(object) {
    return {
      x: data.xpos[object.bodyId * 3],
      y: data.xpos[object.bodyId * 3 + 1],
      z: data.xpos[object.bodyId * 3 + 2]
    };
  }

  function snapshot() {
    return {
      enabled,
      selectedId,
      position: positionOf(selected()),
      lastResult
    };
  }

  function notify() {
    onChange?.(snapshot());
  }

  function overlapsAnother(selectedObject, x, y) {
    return objects.some((other) => {
      if (other.id === selectedObject.id) return false;
      const position = positionOf(other);
      return (
        Math.abs(x - position.x) < selectedObject.halfX + other.halfX + OBJECT_CLEARANCE &&
        Math.abs(y - position.y) < selectedObject.halfY + other.halfY + OBJECT_CLEARANCE
      );
    });
  }

  function placeOnTable(object, x, y) {
    const qpos = object.qposAddress;
    data.qpos[qpos] = x;
    data.qpos[qpos + 1] = y;
    data.qpos[qpos + 2] = object.restZ;
    data.qpos[qpos + 3] = 1;
    data.qpos[qpos + 4] = 0;
    data.qpos[qpos + 5] = 0;
    data.qpos[qpos + 6] = 0;
    for (let index = 0; index < 6; index += 1) {
      data.qvel[object.dofAddress + index] = 0;
    }
    mujoco.mj_forward(model, data);
  }

  function setEnabled(next) {
    enabled = Boolean(next);
    lastResult = null;
    notify();
    return enabled;
  }

  function setSelected(next) {
    if (!objects.some((object) => object.id === next)) return false;
    selectedId = next;
    lastResult = null;
    notify();
    return true;
  }

  function move(dx, dy) {
    if (!enabled || !Number.isFinite(dx) || !Number.isFinite(dy)) {
      lastResult = { ok: false, reason: 'disabled' };
      notify();
      return lastResult;
    }

    const object = selected();
    const current = positionOf(object);
    const minX = TABLE_BOUNDS.minX + object.halfX + EDGE_CLEARANCE;
    const maxX = TABLE_BOUNDS.maxX - object.halfX - EDGE_CLEARANCE;
    const minY = TABLE_BOUNDS.minY + object.halfY + EDGE_CLEARANCE;
    const maxY = TABLE_BOUNDS.maxY - object.halfY - EDGE_CLEARANCE;
    const x = clamp(current.x + dx, minX, maxX);
    const y = clamp(current.y + dy, minY, maxY);

    if (Math.hypot(x - current.x, y - current.y) < 1e-8) {
      lastResult = { ok: false, reason: 'boundary', position: current };
      notify();
      return lastResult;
    }
    if (overlapsAnother(object, x, y)) {
      lastResult = { ok: false, reason: 'overlap', position: current };
      notify();
      return lastResult;
    }

    placeOnTable(object, x, y);
    lastResult = {
      ok: true,
      clamped: Math.abs(x - (current.x + dx)) > 1e-8 || Math.abs(y - (current.y + dy)) > 1e-8,
      position: positionOf(object)
    };
    notify();
    return lastResult;
  }

  function reset() {
    enabled = false;
    lastResult = null;
    notify();
  }

  return {
    objects: objects.map(({ bodyId, qposAddress, dofAddress, ...object }) => ({ ...object })),
    setEnabled,
    setSelected,
    move,
    reset,
    state: snapshot,
    isEnabled() {
      return enabled;
    }
  };
}
