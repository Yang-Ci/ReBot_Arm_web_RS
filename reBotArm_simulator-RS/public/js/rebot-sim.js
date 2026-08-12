(function () {
  const DEG = Math.PI / 180;
  const t = window.rebotI18n ? window.rebotI18n.t : (k) => k;
  const RAD = 180 / Math.PI;
  const NOMINAL_REACH = 0.56;
  const GRIPPER_COMMAND_MAX = 0.0715;
  const GRIPPER_VISUAL_MAX = 0.05;
  const GRIPPER_FINGER_TRAVEL_M = 0.05;
  const GRIPPER_ANIMATION_MS = 520;
 const URDF_HAS_BUILTIN_GRIPPER = true;
 const GRIPPER_MESH_VERSION = 'rs-v1';
  const FAKE_GRASP_LOCAL_OFFSET = new THREE.Vector3(-0.05, 0, -0.02);
  const TABLE_CENTER_X = 0.35;
  const TABLE_WIDTH = 0.36;
  const TABLE_DEPTH = 0.36;
  const TABLE_SURFACE_Y = 0.10;
  const TABLE_THICKNESS = 0.03;

  // The URDF supplies the colour. These Web-specific PBR values compensate for
  // Three.js running without an environment map; copying MuJoCo's high metal
  // value directly makes anodized silver render nearly black in the browser.
  const URDF_FINISH_PARAMS = Object.freeze({
    rs_base_black: { roughness: 0.20, metalness: 0.82 },
    rs_motor_black: { roughness: 0.48, metalness: 0.48 },
   rs_pla_black: { roughness: 0.58, metalness: 0.08 },
    rs_wordmark_black: { roughness: 0.76, metalness: 0.00 },
   rs_anodized_silver: { roughness: 0.24, metalness: 0.58 },
    rs_seeed_green: { roughness: 0.34, metalness: 0.18 }
  });

  const jointDefs = [
    { name: 'joint1', label: 'joint.j1', min: -2.8, max: 2.8, home: 0 },
    { name: 'joint2', label: 'joint.j2', min: 0, max: 3.14, home: 0 },
    { name: 'joint3', label: 'joint.j3', min: 0, max: 3.14, home: 0 },
    { name: 'joint4', label: 'joint.j4', min: -1.57, max: 1.57, home: 0 },
    { name: 'joint5', label: 'joint.j5', min: -1.57, max: 1.57, home: 0 },
    { name: 'joint6', label: 'joint.j6', min: -3.14, max: 3.14, home: 0 },
    { name: 'gripper', label: 'joint.gripper', min: 0, max: GRIPPER_COMMAND_MAX, home: 0, unit: 'm' }
  ];

  const presets = {
    ready: { label: 'preset.ready', angles: [0, 0, 0, 0, 0, 0, 0] },
    forward: { label: 'preset.forward', angles: [0, 25, 35, 28, 0, 0, 90] },
    left: { label: 'preset.left', angles: [42, 25, 45, 32, 18, 0, 90] },
    right: { label: 'preset.right', angles: [-42, 25, 45, 32, -18, 0, 20] },
    inspect: { label: 'preset.inspect', angles: [18, 36, 26, -16, 45, 90, 45] },
    fold: { label: 'preset.fold', angles: [0, 88, 118, 78, 0, 0, 0] }
  };

  let scene;  let camera;
  let sceneResizeObserver;
  let renderer;
  let controls;
  let robot;
  let robotFrame;
  let ghostRobot;
  let ghostDisplayActive = false;
  let hardwareFeedbackDriven = false;
  let gripperGroup;
  let ghostGripperGroup;
  let mujocoSceneGroup;
  let envelopeGroup;
  let workspacePlanarReach = NOMINAL_REACH;
  let workspaceVerticalReach = NOMINAL_REACH;
  let targetGhost;
  let tcpMarker;
  let dragErrorLine;
  let animation = null;
  let currentAngles = {};
  let targetAngles = {};
  let moveStartAngles = {};
  let moveStart = 0;
  let moveDuration = 900;
  let gripperMotion = null;
  let dragMode = false;
  let draggingTcp = false;
  let dragPlane = null;
  let dragTarget = new THREE.Vector3();
  let dragLastTime = 0;
  let dragPointerId = null;
  let dragTargetClamped = false;
  let dragSettling = false;
  let dragSettleStart = 0;
  let dragSettleLastTime = 0;
  const DRAG_SETTLE_TIMEOUT_MS = 1400;
  const DRAG_SETTLE_TARGET_ERROR = 0.002;
  let teachingRecording = false;
  let teachingStart = 0;
  let teachingLastSample = 0;
  let teachingWaypoints = [];
  let teachingPlayback = null;
  const TEACH_SAMPLE_INTERVAL_MS = 90;
  const TEACH_MIN_TCP_STEP = 0.004;
  const TEACH_REPLAY_SAMPLE_HZ = 30;
  const TEACH_REPLAY_MAX_POINTS = 1500;
  const TEACH_REPLAY_ARM_SPEED_RAD_S = 0.6;
 const commandListeners = new Set();
 const axisLabelSprites = [];
  let carriedObject = null;
  const taskObjects = new Map();

  const els = {
    host: document.getElementById('scene-host'),
    loading: document.getElementById('loading-mask'),
    loadingText: document.getElementById('loading-text'),
    status: document.getElementById('load-status'),
    tcp: document.getElementById('tcp-position'),
    reach: document.getElementById('reach-state'),
    dragMarker: document.getElementById('drag-marker'),
    dragHud: document.getElementById('drag-hud'),
    dragStatus: document.getElementById('drag-status'),
    teachRecord: document.getElementById('teach-record'),
    teachReplay: document.getElementById('teach-replay'),
    teachExport: document.getElementById('teach-export'),
    teachClear: document.getElementById('teach-clear'),
    teachStatus: document.getElementById('teach-status'),
    teachExportText: document.getElementById('teach-export-text'),
    joints: document.getElementById('joint-controls'),
    presets: document.getElementById('preset-buttons'),
    planTrajectory: document.getElementById('plan-trajectory'),
    toggleDrag: document.getElementById('toggle-drag')
  };

  jointDefs.forEach((joint) => {
    currentAngles[joint.name] = joint.home;
    targetAngles[joint.name] = joint.home;
  });

  init();

  function init() {
    buildControls();
    setupScene();
    setupEvents();
    updateTeachingStatus();
    loadRobot();
    animate();
  }

  function buildControls() {
    Object.entries(presets).forEach(([key, preset]) => {
      const button = document.createElement('button');
      button.type = 'button';
      button.textContent = t(preset.label);
      button.addEventListener('click', () => applyPreset(key, false, { source: 'preset' }));
      els.presets.appendChild(button);
    });

    jointDefs.forEach((joint) => {
      const wrap = document.createElement('div');
      wrap.className = 'joint-control';

      const head = document.createElement('div');
      head.className = 'joint-head';
      head.innerHTML = `<strong>${t(joint.label)}</strong><span class="joint-value" id="${joint.name}-value">0.0 °</span>`;

      const range = document.createElement('input');
      range.type = 'range';
      range.id = joint.name;
      if (joint.unit === 'm') {
        range.min = (joint.min * 1000).toFixed(0);
        range.max = (joint.max * 1000).toFixed(0);
        range.step = '1';
        range.value = (joint.home * 1000).toFixed(0);
      } else {
        range.min = (joint.min * RAD).toFixed(1);
        range.max = (joint.max * RAD).toFixed(1);
        range.step = '1';
        range.value = (joint.home * RAD).toFixed(1);
      }
      range.addEventListener('input', () => {
        stopPath();
        setGhostDisplay(false);
        const value = joint.unit === 'm' ? Number(range.value) / 1000 : Number(range.value) * DEG;
        if (hardwareFeedbackDriven) {
          emitCommand({
            type: 'joint',
            name: joint.name,
            value,
            source: 'slider',
            phase: 'input',
            stamp: performance.now()
          });
          return;
        }
        setJoint(joint.name, value, true, { source: 'slider', phase: 'input' });
        syncGhostToRobot();
      });
      range.addEventListener('change', () => {
        setGhostDisplay(false);
        const value = joint.unit === 'm' ? Number(range.value) / 1000 : Number(range.value) * DEG;
        if (hardwareFeedbackDriven) {
          emitCommand({
            type: 'joint',
            name: joint.name,
            value,
            source: 'slider',
            phase: 'commit',
            stamp: performance.now()
          });
          return;
        }
        setJoint(joint.name, value, true, { source: 'slider', phase: 'commit' });
        syncGhostToRobot();
      });

      wrap.appendChild(head);
      wrap.appendChild(range);
      els.joints.appendChild(wrap);
      updateJointLabel(joint.name);
    });
  }

  function setupScene() {
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x111211);

    camera = new THREE.PerspectiveCamera(48, getAspect(), 0.01, 20);
    resetCamera();

    renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.setSize(els.host.clientWidth, els.host.clientHeight);
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    // This project ships Three.js r128, which uses outputEncoding. Keep the
    // newer property as a forward-compatible path when the library is updated.
    if ('outputColorSpace' in renderer && THREE.SRGBColorSpace) {
      renderer.outputColorSpace = THREE.SRGBColorSpace;
    } else if (THREE.sRGBEncoding !== undefined) {
      renderer.outputEncoding = THREE.sRGBEncoding;
    }
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 0.98;
    els.host.appendChild(renderer.domElement);

    controls = createOrbit(camera, renderer.domElement, new THREE.Vector3(0.18, 0.2, 0));

    robotFrame = new THREE.Group();
    robotFrame.rotation.x = -Math.PI / 2;
    scene.add(robotFrame);

    setupLights();
    createWorkbench();
    createDirectionAxes();
    envelopeGroup = createEnvelope();
    scene.add(envelopeGroup);

    tcpMarker = new THREE.Mesh(
      new THREE.SphereGeometry(0.012, 24, 16),
      new THREE.MeshStandardMaterial({ color: 0x33d6b0, emissive: 0x0a4d3d, emissiveIntensity: 0.9 })
    );
    tcpMarker.visible = false;
    scene.add(tcpMarker);

    targetGhost = new THREE.Mesh(
      new THREE.SphereGeometry(0.018, 28, 18),
      new THREE.MeshBasicMaterial({ color: 0xf2a541, transparent: true, opacity: 0.85 })
    );
    targetGhost.visible = false;
    scene.add(targetGhost);

    dragErrorLine = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]),
      new THREE.LineBasicMaterial({ color: 0xff6b5f, transparent: true, opacity: 0.82 })
    );
    dragErrorLine.visible = false;
    scene.add(dragErrorLine);
  }

  function setupLights() {
    scene.add(new THREE.HemisphereLight(0xfff8e8, 0x141613, 0.72));

    const key = new THREE.DirectionalLight(0xfff3dc, 1.35);
    key.position.set(1.4, 2.2, 1.2);
    key.castShadow = true;
    key.shadow.mapSize.set(2048, 2048);
    key.shadow.camera.near = 0.1;
    key.shadow.camera.far = 6;
    key.shadow.camera.left = -1.4;
    key.shadow.camera.right = 1.4;
    key.shadow.camera.top = 1.4;
    key.shadow.camera.bottom = -1.4;
    scene.add(key);

    const side = new THREE.DirectionalLight(0xdde8e2, 0.25);
    side.position.set(-1, 0.6, -1.2);
    scene.add(side);

    const rim = new THREE.DirectionalLight(0xd8fff0, 0.38);
    rim.position.set(-0.8, 1.5, 1.8);
    scene.add(rim);
  }

  function createWorkbench() {
    const grid = new THREE.GridHelper(2.4, 48, 0x4d716a, 0x2c3a35);
    grid.position.y = 0;
    scene.add(grid);

    mujocoSceneGroup = new THREE.Group();
    mujocoSceneGroup.name = 'mujoco-task-scene';

    const table = new THREE.Mesh(
      new THREE.BoxGeometry(TABLE_WIDTH, TABLE_THICKNESS, TABLE_DEPTH),
      new THREE.MeshStandardMaterial({
        color: 0x575e5c,
        map: createTableTexture(),
        roughness: 0.66,
        metalness: 0.08
      })
    );
    table.name = 'mujoco-task-table';
    table.position.set(TABLE_CENTER_X, TABLE_SURFACE_Y - TABLE_THICKNESS / 2, 0);
    table.castShadow = true;
    table.receiveShadow = true;
    table.userData.collisionKind = 'table';
    mujocoSceneGroup.add(table);

    const tableOutline = new THREE.LineSegments(
      new THREE.EdgesGeometry(table.geometry, 28),
      new THREE.LineBasicMaterial({ color: 0xb9c3c0, transparent: true, opacity: 0.34 })
    );
    tableOutline.position.copy(table.position);
    mujocoSceneGroup.add(tableOutline);
    mujocoSceneGroup.add(createTaskSpace());

    const sceneToggle = document.getElementById('toggle-mujoco-scene');
    mujocoSceneGroup.visible = !sceneToggle || sceneToggle.checked;
    scene.add(mujocoSceneGroup);
  }

  function createTableTexture() {
    const canvas = document.createElement('canvas');
    canvas.width = 192;
    canvas.height = 192;
    const ctx = canvas.getContext('2d');
    const image = ctx.createImageData(canvas.width, canvas.height);
    let seed = 601;
    for (let y = 0; y < canvas.height; y += 1) {
      for (let x = 0; x < canvas.width; x += 1) {
        seed = (seed * 1664525 + 1013904223) >>> 0;
        const grain = ((seed >>> 24) / 255 - 0.5) * 10;
        const streak = Math.sin(y * 0.19) * 1.8;
        const index = (y * canvas.width + x) * 4;
        image.data[index] = clamp(88 + grain + streak, 0, 255);
        image.data[index + 1] = clamp(96 + grain + streak, 0, 255);
        image.data[index + 2] = clamp(93 + grain + streak, 0, 255);
        image.data[index + 3] = 255;
      }
    }
    ctx.putImageData(image, 0, 0);
    const texture = new THREE.CanvasTexture(canvas);
    texture.wrapS = THREE.RepeatWrapping;
    texture.wrapT = THREE.RepeatWrapping;
    texture.repeat.set(2.8, 2.8);
    texture.anisotropy = Math.min(8, renderer.capabilities.getMaxAnisotropy());
    if ('colorSpace' in texture && THREE.SRGBColorSpace) texture.colorSpace = THREE.SRGBColorSpace;
    else if (THREE.sRGBEncoding !== undefined) texture.encoding = THREE.sRGBEncoding;
    return texture;
  }

  function createDirectionAxes() {
    const origin = new THREE.Vector3(0, 0.006, 0);
    addArrow(origin, new THREE.Vector3(1, 0, 0), 0xef5a4d, t('app.axisX'));
    addArrow(origin, new THREE.Vector3(0, 0, -1), 0x77c96b, t('app.axisY'));
    addArrow(origin, new THREE.Vector3(0, 1, 0), 0x5fa8ff, t('app.axisZ'));
  }

  function addArrow(origin, dir, color, label) {
    const arrow = new THREE.ArrowHelper(dir, origin, 0.18, color, 0.035, 0.012);
    scene.add(arrow);

    const sprite = makeTextSprite(label, color);
    sprite.position.copy(origin).add(dir.clone().multiplyScalar(0.23));
    sprite.position.y += dir.y === 0 ? 0.018 : 0;
    sprite.userData.autoHideAt = performance.now() + 3000;
    sprite.userData.fadeDuration = 900;
    axisLabelSprites.push(sprite);
    scene.add(sprite);
  }

  function createEnvelope() {
    const group = new THREE.Group();
    const mainMat = new THREE.LineBasicMaterial({ color: 0x33d6b0, transparent: true, opacity: 0.32 });
    const guideMat = new THREE.LineBasicMaterial({ color: 0x33d6b0, transparent: true, opacity: 0.18 });
    const radius = workspacePlanarReach;
    const heightLimit = workspaceVerticalReach;

    [0, 0.25, 0.5, 0.75, 0.95].forEach((ratio) => {
      const height = heightLimit * ratio;
      const ringRadius = radius * Math.sqrt(Math.max(0, 1 - ratio * ratio));
      group.add(makeCircleLine(ringRadius, height, height === 0 ? mainMat : guideMat));
    });

    for (let i = 0; i < 12; i += 1) {
      group.add(makeVerticalArc(radius, heightLimit, (i / 12) * Math.PI * 2, i % 3 === 0 ? mainMat : guideMat));
    }
    return group;
  }

  function makeCircleLine(radius, y, mat) {
    const points = [];
    for (let i = 0; i <= 128; i++) {
      const a = (i / 128) * Math.PI * 2;
      points.push(new THREE.Vector3(Math.cos(a) * radius, y, Math.sin(a) * radius));
    }
    return new THREE.Line(new THREE.BufferGeometry().setFromPoints(points), mat);
  }

  function makeVerticalArc(radius, heightLimit, yaw, mat) {
    const points = [];
    for (let i = 0; i <= 72; i++) {
      const a = (i / 72) * Math.PI / 2;
      const r = Math.cos(a) * radius;
      const y = Math.sin(a) * heightLimit;
      points.push(new THREE.Vector3(Math.cos(yaw) * r, y, Math.sin(yaw) * r));
    }
    return new THREE.Line(new THREE.BufferGeometry().setFromPoints(points), mat);
  }

  function createTaskSpace() {
    const group = new THREE.Group();
    group.name = 'mujoco-task-objects';
    const objects = [
      {
        key: 'red',
        label: 'sim.redBlock',
        color: 0xeb140f,
        position: [0.31, -0.09, 0.1225],
        geometry: new THREE.BoxGeometry(0.045, 0.045, 0.045)
      },
      {
        key: 'blue',
        label: 'sim.blueBlock',
        color: 0x086bf0,
        position: [0.32, 0.08, 0.119],
        geometry: new THREE.BoxGeometry(0.052, 0.038, 0.032)
      },
      {
        key: 'yellow',
        label: 'sim.cylinder',
        color: 0xffb80a,
        position: [0.25, 0.10, 0.126],
        geometry: new THREE.CylinderGeometry(0.020, 0.020, 0.052, 32)
      }
    ];

    objects.forEach((object) => {
      const item = new THREE.Mesh(
        object.geometry,
        new THREE.MeshStandardMaterial({
          color: object.color,
          roughness: 0.34,
          metalness: 0.06
        })
      );
      const [rosX, rosY, rosZ] = object.position;
      item.position.set(rosX, rosZ, -rosY);
      item.castShadow = true;
      item.receiveShadow = true;
      item.userData.clickTarget = true;
      item.userData.targetKind = 'object';
      item.userData.targetLabel = t(object.label);
      item.userData.targetColor = object.key;
      item.userData.tableY = rosZ;
      item.userData.restPosition = item.position.clone();
      taskObjects.set(object.key, item);
      group.add(item);

      const outline = new THREE.LineSegments(
        new THREE.EdgesGeometry(object.geometry, 32),
        new THREE.LineBasicMaterial({ color: 0x18201e, transparent: true, opacity: 0.36 })
      );
      item.add(outline);
    });
    return group;
  }

  function loadRobot() {
    if (typeof URDFLoader === 'undefined') {
      failLoad('URDFLoader is not loaded.');
      return;
    }

    const manager = new THREE.LoadingManager();
    manager.onProgress = (url, loaded, total) => {
      els.loadingText.textContent = `Loading model ${Math.round((loaded / Math.max(total, 1)) * 100)}%`;
    };
    manager.onLoad = () => {
      if (!robot) return;
      finishRobotLoad();
    };

    const loader = new URDFLoader(manager);
    loader.packages = {
      rebotarm_bringup: `${window.location.origin}/api`
    };

    loader.load('/api/urdf', (loadedRobot) => {
      robot = loadedRobot;
      robotFrame.add(robot);
    }, undefined, (error) => {
      failLoad(`URDF load failed: ${error && error.message ? error.message : error}`);
    });
  }

  async function finishRobotLoad() {
    try {
      styleRobot(robot, false);
      if (URDF_HAS_BUILTIN_GRIPPER) {
        gripperGroup = robot.getObjectByName('gripper_end');
      } else {
        try {
          gripperGroup = await attachGripperVisual(robot, false);
        } catch (error) {
          console.warn('Gripper STL load failed, continuing with arm model only:', error);
        }
      }
      createGhostRobot();
      applyPreset('ready', true);
      estimateWorkspaceEnvelope();
      rebuildEnvelope();
      syncGhostToRobot();
    } catch (error) {
      console.warn('finishRobotLoad partial error:', error);
    }
    updateReadyState();
  }

  function estimateWorkspaceEnvelope() {
    if (!robot) return;

    const savedAngles = { ...currentAngles };
    let maxPlanar = NOMINAL_REACH;
    let maxVertical = NOMINAL_REACH;
    const movableJoints = jointDefs.filter((joint) => joint.name !== 'gripper');

    for (let i = 0; i < 960; i += 1) {
      const sample = { ...savedAngles };
      movableJoints.forEach((joint, index) => {
        const t = seededUnit(i + 1, index + 3);
        sample[joint.name] = joint.min + (joint.max - joint.min) * t;
      });
      applyRobotAngles(robot, sample);
      robot.updateMatrixWorld(true);

      const pos = getTcpPosition(robot);
      if (!pos) continue;
      maxPlanar = Math.max(maxPlanar, Math.sqrt(pos.x * pos.x + pos.z * pos.z));
      maxVertical = Math.max(maxVertical, Math.max(0, pos.y));
    }

    applyRobotAngles(robot, savedAngles);
    robot.updateMatrixWorld(true);
    workspacePlanarReach = clamp(Math.ceil(maxPlanar * 100) / 100, NOMINAL_REACH, 1.2);
    workspaceVerticalReach = clamp(Math.ceil(maxVertical * 100) / 100, NOMINAL_REACH, 1.2);
  }

  function seededUnit(a, b) {
    const x = Math.sin(a * 12.9898 + b * 78.233) * 43758.5453;
    return x - Math.floor(x);
  }

  function rebuildEnvelope() {
    if (!scene) return;
    const wasVisible = envelopeGroup ? envelopeGroup.visible : true;
    if (envelopeGroup) scene.remove(envelopeGroup);
    envelopeGroup = createEnvelope();
    const toggle = document.getElementById('toggle-envelope');
    envelopeGroup.visible = toggle ? toggle.checked && wasVisible : wasVisible;
    scene.add(envelopeGroup);
  }

 function styleRobot(root, ghost) {
   root.traverse((child) => {
     if (!child.isMesh) return;
     child.castShadow = !ghost;
     child.receiveShadow = !ghost;

     if (ghost) {
       child.material = new THREE.MeshStandardMaterial({
         color: 0x33d6b0,
         roughness: 0.28,
         metalness: 0.05,
         transparent: true,
         opacity: 0.22,
         side: THREE.DoubleSide
       });
       return;
     }

     const sourceMaterial = Array.isArray(child.material) ? child.material[0] : child.material;
     const materialName = sourceMaterial && sourceMaterial.name
       ? sourceMaterial.name
       : linkFinishName(child);
     const finish = URDF_FINISH_PARAMS[materialName];
     if (!finish) return;

     child.material = new THREE.MeshStandardMaterial({
       color: sourceMaterial.color || 0xcccccc,
       roughness: finish.roughness,
       metalness: finish.metalness,
       side: THREE.DoubleSide
     });
   });
 }

 function linkFinishName(object) {
   let node = object;
   while (node) {
     if (node.material && !Array.isArray(node.material) && node.material.name) {
       return node.material.name;
     }
     node = node.parent;
   }
   return '';
 }

  function createGhostRobot() {
    if (!robot) return;
    ghostRobot = robot.clone(true);
    styleRobot(ghostRobot, true);
    ghostGripperGroup = ghostRobot.getObjectByName('sim_gripper') || ghostRobot.getObjectByName('gripper_end');
    ghostRobot.visible = false;
    robotFrame.add(ghostRobot);
  }

  function setGhostDisplay(active) {
    ghostDisplayActive = Boolean(active);
    if (!ghostRobot) return;
    const toggle = document.getElementById('toggle-ghost');
    ghostRobot.visible = ghostDisplayActive && (!toggle || toggle.checked);
  }

  async function attachGripperVisual(root, ghost) {
    const endLink = root.getObjectByName('gripper_end') || root.getObjectByName('end_link') || root.getObjectByName('link6');
    if (!endLink || !THREE.STLLoader) return null;

    hideOriginalEndLinkMeshes(endLink);

    const group = new THREE.Group();
    group.name = 'sim_gripper';
    endLink.add(group);

    const loader = new THREE.STLLoader();
    const parts = [
     { name: 'gripper_base', file: 'gripper_base.stl', color: ghost ? 0x33d6b0 : 0x3d4745, moving: false },
     { name: 'left_finger', file: 'left_finger.stl', color: ghost ? 0x33d6b0 : 0x3d4745, moving: true },
     { name: 'right_finger', file: 'right_finger.stl', color: ghost ? 0x33d6b0 : 0x3d4745, moving: true }
    ];

    const meshes = await Promise.all(parts.map((part) => loadGripperMesh(loader, part, ghost)));
    meshes.forEach((mesh) => group.add(mesh));
    updateGripperVisual(group, currentAngles.gripper ?? 0);
    return group;
  }

  function hideOriginalEndLinkMeshes(endLink) {
    endLink.traverse((child) => {
      if (child !== endLink && child.isMesh) {
        child.visible = false;
      }
    });
  }

  function loadGripperMesh(loader, part, ghost) {
    return new Promise((resolve, reject) => {
      loader.load(`/api/gripper_meshes/${part.file}?v=${GRIPPER_MESH_VERSION}`, (geometry) => {
        geometry.computeVertexNormals();
        const material = new THREE.MeshStandardMaterial({
          color: part.color,
          roughness: part.moving ? 0.42 : 0.62,
          metalness: part.moving ? 0.18 : 0.25,
          transparent: ghost,
          opacity: ghost ? 0.22 : 1,
          side: THREE.DoubleSide
        });
        const mesh = new THREE.Mesh(geometry, material);
        mesh.name = part.name;
        mesh.castShadow = !ghost;
        mesh.receiveShadow = !ghost;
        mesh.userData.isMovingFinger = part.moving;
        resolve(mesh);
      }, undefined, reject);
    });
  }

  function updateGripperVisual(group, widthM) {
    if (!group) return;
    const commandWidth = clamp(widthM, 0, GRIPPER_COMMAND_MAX);
    const visualWidth = (commandWidth / GRIPPER_COMMAND_MAX) * GRIPPER_VISUAL_MAX;
    const half = visualWidth / 2;
    const left = group.getObjectByName('left_finger');
    const right = group.getObjectByName('right_finger');
    if (left) left.position.y = half;
    if (right) right.position.y = -half;
  }

  function updateReadyState() {
    els.status.classList.add('ready');
    els.status.lastChild.textContent = ' Ready';
    els.loading.classList.add('hidden');
  }

  function failLoad(message) {
    els.status.lastChild.textContent = ' Load failed';
    els.loadingText.textContent = message;
  }

  function setupEvents() {
    window.addEventListener('resize', resize);
    if (window.ResizeObserver) {
      sceneResizeObserver = new ResizeObserver(() => resize());
      sceneResizeObserver.observe(els.host);
    }
    document.getElementById('reset-camera').addEventListener('click', resetCamera);
    document.getElementById('play-path').addEventListener('click', playPath);
    document.getElementById('stop-path').addEventListener('click', () => {
      stopActiveMotion();
    });
    if (els.planTrajectory) els.planTrajectory.addEventListener('click', generateTrajectory);
    if (els.toggleDrag) els.toggleDrag.addEventListener('click', toggleDragMode);
    if (els.teachRecord) els.teachRecord.addEventListener('click', toggleTeachingRecord);
    if (els.teachReplay) els.teachReplay.addEventListener('click', replayTeaching);
    if (els.teachExport) els.teachExport.addEventListener('click', exportTeachingWaypoints);
    if (els.teachClear) els.teachClear.addEventListener('click', clearTeaching);
    if (els.dragMarker) {
      els.dragMarker.addEventListener('pointerdown', startTcpDrag);
    }
    window.addEventListener('pointermove', moveTcpDrag);
    window.addEventListener('pointerup', endTcpDrag);
    window.addEventListener('pointercancel', endTcpDrag);
    document.getElementById('open-gripper')?.addEventListener('click', () => setGripperWidth(GRIPPER_COMMAND_MAX));
    document.getElementById('close-gripper')?.addEventListener('click', () => setGripperWidth(0));

    document.getElementById('toggle-envelope').addEventListener('change', (event) => {
      envelopeGroup.visible = event.target.checked;
    });
    document.getElementById('toggle-mujoco-scene')?.addEventListener('change', (event) => {
      if (mujocoSceneGroup) mujocoSceneGroup.visible = event.target.checked;
    });
    document.getElementById('toggle-ghost').addEventListener('change', (event) => {
      if (ghostRobot) ghostRobot.visible = ghostDisplayActive && event.target.checked;
      if (targetGhost) targetGhost.visible = false;
    });
  }

  function applyPreset(key, immediate) {
    const preset = presets[key];
    if (!preset) return;
    const next = {};
    jointDefs.forEach((joint, index) => {
      const raw = preset.angles[index] || 0;
      next[joint.name] = clamp(joint.unit === 'm' ? raw / 1000 : raw * DEG, joint.min, joint.max);
    });
    moveToAngles(next, immediate ? 1 : 850, {
      source: immediate ? 'init' : 'preset',
      label: t(preset.label),
      emitBatch: !immediate
    });
  }

  function setJoint(name, rad, fromUser, options) {
    const def = jointDefs.find((item) => item.name === name);
    if (!def) return;
    const value = clamp(rad, def.min, def.max);
    currentAngles[name] = value;

    if (name === 'gripper') {
      if (URDF_HAS_BUILTIN_GRIPPER) {
        const gj1 = getJoint(robot, 'gripper_joint1');
        const gj2 = getJoint(robot, 'gripper_joint2');
        const ratio = value / GRIPPER_COMMAND_MAX;
        if (gj1) {
          if (typeof gj1.setJointValue === 'function') gj1.setJointValue(ratio * GRIPPER_FINGER_TRAVEL_M);
          else if (typeof gj1.setAngle === 'function') gj1.setAngle(ratio * GRIPPER_FINGER_TRAVEL_M);
        }
        if (gj2) {
          if (typeof gj2.setJointValue === 'function') gj2.setJointValue(ratio * GRIPPER_FINGER_TRAVEL_M);
          else if (typeof gj2.setAngle === 'function') gj2.setAngle(ratio * GRIPPER_FINGER_TRAVEL_M);
        }
      } else {
        updateGripperVisual(gripperGroup, value);
        updateGripperVisual(ghostGripperGroup, value);
      }
    }

    const joint = getJoint(robot, name);
    if (joint) {
      if (typeof joint.setJointValue === 'function') {
        joint.setJointValue(value);
      } else if (typeof joint.setAngle === 'function') {
        joint.setAngle(value);
      }
    }
    if (name === 'gripper' && !URDF_HAS_BUILTIN_GRIPPER) {
      const leftFinger = getJoint(robot, 'finger_left');
      const rightFinger = getJoint(robot, 'finger_right');
      const fingerTravel = (value / GRIPPER_COMMAND_MAX) * GRIPPER_VISUAL_MAX * 0.5;
      if (leftFinger) {
        if (typeof leftFinger.setJointValue === 'function') {
          leftFinger.setJointValue(fingerTravel);
        } else if (typeof leftFinger.setAngle === 'function') {
          leftFinger.setAngle(fingerTravel);
        }
      }
      if (rightFinger) {
        if (typeof rightFinger.setJointValue === 'function') {
          rightFinger.setJointValue(-fingerTravel);
        } else if (typeof rightFinger.setAngle === 'function') {
          rightFinger.setAngle(-fingerTravel);
        }
      }
    }

    if (!fromUser) {
      const slider = document.getElementById(name);
      if (slider) slider.value = def.unit === 'm' ? (value * 1000).toFixed(0) : (value * RAD).toFixed(1);
    }
    updateJointLabel(name);

    const source = options && options.source ? options.source : (fromUser ? 'user' : 'sim');
    if (source !== 'ros' && !(options && options.emit === false)) {
      emitCommand({
        type: 'joint',
        name,
        value,
        source,
        phase: options && options.phase ? options.phase : 'commit',
        stamp: performance.now()
      });
    }
  }

  function setGhostJoint(name, rad) {
    if (name === 'gripper') {
      if (URDF_HAS_BUILTIN_GRIPPER) {
        const gj1 = getJoint(ghostRobot, 'gripper_joint1');
        const gj2 = getJoint(ghostRobot, 'gripper_joint2');
        const ratio = rad / GRIPPER_COMMAND_MAX;
        if (gj1) {
          if (typeof gj1.setJointValue === 'function') gj1.setJointValue(ratio * GRIPPER_FINGER_TRAVEL_M);
          else if (typeof gj1.setAngle === 'function') gj1.setAngle(ratio * GRIPPER_FINGER_TRAVEL_M);
        }
        if (gj2) {
          if (typeof gj2.setJointValue === 'function') gj2.setJointValue(ratio * GRIPPER_FINGER_TRAVEL_M);
          else if (typeof gj2.setAngle === 'function') gj2.setAngle(ratio * GRIPPER_FINGER_TRAVEL_M);
        }
      } else {
        const leftFinger = getJoint(ghostRobot, 'finger_left');
        const rightFinger = getJoint(ghostRobot, 'finger_right');
        const fingerTravel = (rad / GRIPPER_COMMAND_MAX) * GRIPPER_VISUAL_MAX * 0.5;
        if (leftFinger) {
          if (typeof leftFinger.setJointValue === 'function') {
            leftFinger.setJointValue(fingerTravel);
          } else if (typeof leftFinger.setAngle === 'function') {
            leftFinger.setAngle(fingerTravel);
          }
        }
        if (rightFinger) {
          if (typeof rightFinger.setJointValue === 'function') {
            rightFinger.setJointValue(-fingerTravel);
          } else if (typeof rightFinger.setAngle === 'function') {
            rightFinger.setAngle(-fingerTravel);
          }
        }
        updateGripperVisual(ghostGripperGroup, rad);
      }
      return;
    }
    const joint = getJoint(ghostRobot, name);
    if (!joint) return;
    if (typeof joint.setJointValue === 'function') {
      joint.setJointValue(rad);
    } else if (typeof joint.setAngle === 'function') {
      joint.setAngle(rad);
    }
  }

  function getJoint(root, name) {
    if (!root) return null;
    if (root.joints && root.joints[name]) return root.joints[name];
    return root.getObjectByName(name);
  }

  function moveToAngles(nextAngles, duration, options) {
    teachingPlayback = null;
    moveStartAngles = { ...currentAngles };
    targetAngles = { ...nextAngles };
    moveStart = performance.now();
    moveDuration = Math.max(duration || 850, 1);
    updateGhostTarget(nextAngles);
    setGhostDisplay(true);
    if (options && options.emitBatch) {
      emitJointBatch(nextAngles, options.source || 'trajectory-target', options.label || '');
    }
  }

  function updateMotion(now) {
    if (!moveStart) return;
    const t = clamp((now - moveStart) / moveDuration, 0, 1);
    const eased = t < 0.5 ? 4 * t * t * t : 1 - Math.pow(-2 * t + 2, 3) / 2;
    jointDefs.forEach((joint) => {
      const start = moveStartAngles[joint.name] ?? currentAngles[joint.name];
      const end = targetAngles[joint.name] ?? start;
      setJoint(joint.name, start + (end - start) * eased, false, { source: 'trajectory', emit: false });
    });
    if (t >= 1) {
      moveStart = 0;
      setGhostDisplay(false);
    }
  }

  function updateGhostTarget(angles) {
    if (!ghostRobot) return;
    jointDefs.forEach((joint) => setGhostJoint(joint.name, angles[joint.name] ?? 0));
    ghostRobot.updateMatrixWorld(true);

    const pos = getTcpPosition(ghostRobot);
    if (pos) {
      targetGhost.position.copy(pos);
      targetGhost.userData.active = true;
      targetGhost.visible = false;
    }
  }

  function syncGhostToRobot() {
    if (!ghostRobot) return;
    jointDefs.forEach((joint) => setGhostJoint(joint.name, currentAngles[joint.name] ?? 0));
    ghostRobot.updateMatrixWorld(true);
  }

  function generateTrajectory() {
    if (!robot) return;
    stopPath();
    draggingTcp = false;
    dragSettling = false;
    const destination = { ...currentAngles };
    const readyPreset = presets.ready;
    const readyAngles = {};

    jointDefs.forEach((joint, index) => {
      const raw = readyPreset.angles[index] || 0;
      readyAngles[joint.name] = clamp(joint.unit === 'm' ? raw / 1000 : raw * DEG, joint.min, joint.max);
      setJoint(joint.name, readyAngles[joint.name], false, { source: 'ros' });
    });

    syncGhostToRobot();
    updateGhostTarget(destination);
    moveToAngles(destination, 1200, {
      source: 'plan-current',
      label: t('adv.plan'),
      emitBatch: true
    });
    setDragStatus('已生成：Ready -> 当前姿态');
  }

  function toggleDragMode() {
    dragMode = !dragMode;
    draggingTcp = false;
    dragSettling = false;
    dragLastTime = 0;
    setGhostDisplay(false);

    if (els.toggleDrag) {
      els.toggleDrag.textContent = dragMode ? t('sim.exitDrag') : t('adv.drag');
      els.toggleDrag.classList.toggle('active', dragMode);
    }
    if (els.dragMarker) {
      els.dragMarker.classList.toggle('active', dragMode);
      els.dragMarker.classList.remove('dragging');
      if (!dragMode) els.dragMarker.style.display = 'none';
    }
    if (els.dragHud) {
      els.dragHud.classList.toggle('active', dragMode);
    }

    const pos = getTcpPosition(robot);
    if (pos) {
      dragTarget.copy(pos);
      showTargetGhost(pos);
    }
    updateDragErrorLine();
    setDragStatus(dragMode ? t('sim.dragGreen') : t('app.dragDisabled'));
    updateDragMarker();
  }

  function startTcpDrag(event) {
    if (!dragMode || !robot) return;
    event.preventDefault();
    event.stopPropagation();
    draggingTcp = true;
    setGhostDisplay(true);
    dragSettling = false;
    dragTargetClamped = false;
    stopPath();
    moveStart = 0;
    dragPointerId = event.pointerId;
    dragLastTime = performance.now();
    dragPlane = createDragPlane();
    recordTeachingWaypoint(true);
    if (els.dragMarker) {
      els.dragMarker.classList.add('dragging');
      els.dragMarker.setPointerCapture(event.pointerId);
    }
    moveTcpDrag(event);
  }

  function moveTcpDrag(event) {
    if (!draggingTcp || !dragPlane || !robot) return;
    dragSettling = false;
    const hit = screenToDragPlane(event.clientX, event.clientY, dragPlane);
    if (!hit) return;

    const boundedTarget = clampToWorkspaceEnvelope(hit);
    dragTarget.copy(boundedTarget.point);
    dragTargetClamped = boundedTarget.clamped;
    showTargetGhost(dragTarget, dragTargetClamped);

    const now = performance.now();
    const dt = Math.min(0.05, Math.max(0.012, (now - dragLastTime) / 1000 || 0.016));
    dragLastTime = now;

    let result = null;
    const substeps = Math.max(1, Math.ceil(dt / 0.016));
    for (let i = 0; i < substeps; i += 1) {
      result = IKSolver.servoStep(dragTarget, dt / substeps, {
        source: 'drag',
        emit: true,
        phase: 'input'
      });
    }

    syncGhostToRobot();
    recordTeachingWaypoint(false);
    updateDragMarker();
    updateDragErrorLine();
    if (result) {
      setDragStatus(`${dragTargetClamped ? t('sim.edgeSnap') : ''}误差 ${(result.error * 1000).toFixed(1)}mm`);
    }
  }

  function endTcpDrag(event) {
    if (!draggingTcp) return;
    draggingTcp = false;
    dragPlane = null;
    dragPointerId = null;
    const releasedTcp = getTcpPosition(robot);
    if (releasedTcp && releasedTcp.distanceTo(dragTarget) > DRAG_SETTLE_TARGET_ERROR) {
      dragSettling = true;
      dragSettleStart = performance.now();
      dragSettleLastTime = dragSettleStart;
      setDragStatus(t('sim.converging', { mm: (releasedTcp.distanceTo(dragTarget) * 1000).toFixed(1) }));
    }
    if (els.dragMarker) {
      els.dragMarker.classList.remove('dragging');
      if (event && els.dragMarker.hasPointerCapture(event.pointerId)) {
        els.dragMarker.releasePointerCapture(event.pointerId);
      }
    }
    recordTeachingWaypoint(true);
    if (dragSettling) return;
    dragTargetClamped = false;
    const tcp = getTcpPosition(robot);
    if (tcp) {
      setDragStatus(t('sim.doneMm', { mm: (tcp.distanceTo(dragTarget) * 1000).toFixed(1) }));
    }
    commitTcpDragTarget();
    setGhostDisplay(false);
  }

  function updateDragMarker() {
    if (!dragMode || !els.dragMarker || !camera || !robot) return;
    const pos = (draggingTcp || dragSettling) ? dragTarget : getTcpPosition(robot);
    if (!pos) return;

    const hostRect = els.host.getBoundingClientRect();
    const viewportRect = document.getElementById('viewport').getBoundingClientRect();
    const projected = pos.clone().project(camera);
    const x = hostRect.left - viewportRect.left + ((projected.x + 1) / 2) * hostRect.width;
    const y = hostRect.top - viewportRect.top + ((1 - projected.y) / 2) * hostRect.height;

    els.dragMarker.style.left = `${x}px`;
    els.dragMarker.style.top = `${y}px`;
    els.dragMarker.style.display = projected.z < 1 ? 'block' : 'none';
  }

  function updateDragSettling(now) {
    if (!dragMode || !dragSettling || draggingTcp || !robot) return;

    const dt = Math.min(0.05, Math.max(0.012, (now - dragSettleLastTime) / 1000 || 0.016));
    dragSettleLastTime = now;

    let result = null;
    const substeps = Math.max(1, Math.ceil(dt / 0.016));
    for (let i = 0; i < substeps; i += 1) {
      result = IKSolver.servoStep(dragTarget, dt / substeps, {
        source: 'drag-settle',
        emit: true,
        phase: 'input'
      });
    }

    syncGhostToRobot();
    recordTeachingWaypoint(false);
    showTargetGhost(dragTarget, dragTargetClamped);
    updateDragErrorLine();

    const elapsed = now - dragSettleStart;
    const fallbackTcp = result ? null : getTcpPosition(robot);
    const error = result ? result.error : (fallbackTcp ? fallbackTcp.distanceTo(dragTarget) : 0);
    if ((result && result.reached) || error <= DRAG_SETTLE_TARGET_ERROR) {
      dragSettling = false;
      dragTargetClamped = false;
      updateDragErrorLine();
      setDragStatus(t('sim.doneMm', { mm: (error * 1000).toFixed(1) }));
      commitTcpDragTarget();
      setGhostDisplay(false);
    } else if (elapsed >= DRAG_SETTLE_TIMEOUT_MS) {
      dragSettling = false;
      dragTargetClamped = false;
      updateDragErrorLine();
      setDragStatus(t('sim.bestEffortMm', { mm: (error * 1000).toFixed(1) }));
      commitTcpDragTarget();
      setGhostDisplay(false);
    } else {
      setDragStatus(t('sim.converging', { mm: (error * 1000).toFixed(1) }));
    }
  }

  function createDragPlane() {
    const tcp = getTcpPosition(robot) || new THREE.Vector3();
    const normal = new THREE.Vector3();
    camera.getWorldDirection(normal);
    return new THREE.Plane().setFromNormalAndCoplanarPoint(normal, tcp);
  }

  function commitTcpDragTarget() {
    emitJointBatch({ ...currentAngles }, 'tcp-drag-commit', 'TCP 拖拽');
  }

  function screenToDragPlane(clientX, clientY, plane) {
    const rect = els.host.getBoundingClientRect();
    const mouse = new THREE.Vector2(
      ((clientX - rect.left) / rect.width) * 2 - 1,
      -((clientY - rect.top) / rect.height) * 2 + 1
    );
    const raycaster = new THREE.Raycaster();
    raycaster.setFromCamera(mouse, camera);
    const hit = new THREE.Vector3();
    return raycaster.ray.intersectPlane(plane, hit) ? hit : null;
  }

  function clampToWorkspaceEnvelope(pos) {
    const point = pos.clone();
    let clamped = false;
    const radius = Math.max(workspacePlanarReach, 0.05);
    const heightLimit = Math.max(workspaceVerticalReach, 0.05);

    if (point.y < 0) {
      point.y = 0;
      clamped = true;
    } else if (point.y > heightLimit) {
      point.y = heightLimit;
      clamped = true;
    }

    const verticalRatio = clamp(point.y / heightLimit, 0, 1);
    const planarLimit = Math.max(0.03, radius * Math.sqrt(Math.max(0, 1 - verticalRatio * verticalRatio)));
    const planar = Math.sqrt(point.x * point.x + point.z * point.z);
    if (planar > planarLimit) {
      const scale = planarLimit / planar;
      point.x *= scale;
      point.z *= scale;
      clamped = true;
    }

    return { point, clamped };
  }

  function updateDragErrorLine() {
    if (!dragErrorLine || !robot) return;
    const active = dragMode && (draggingTcp || dragSettling || dragTargetClamped);
    const tcp = active ? getTcpPosition(robot) : null;
    if (!active || !tcp) {
      dragErrorLine.visible = false;
      return;
    }

    const error = tcp.distanceTo(dragTarget);
    if (error < 0.001) {
      dragErrorLine.visible = false;
      return;
    }

    dragErrorLine.geometry.setFromPoints([tcp, dragTarget]);
    dragErrorLine.material.opacity = clamp(error / 0.08, 0.28, 0.9);
    dragErrorLine.visible = true;
  }

  function showTargetGhost(pos, clamped) {
    if (!targetGhost || !pos) return;
    targetGhost.position.copy(pos);
    targetGhost.userData.active = true;
    targetGhost.userData.clamped = !!clamped;
    if (targetGhost.material && targetGhost.material.color) {
      targetGhost.material.color.set(clamped ? 0xff6b5f : 0xf2a541);
      targetGhost.material.opacity = clamped ? 0.95 : 0.85;
    }
    targetGhost.visible = false;
  }

  function setDragStatus(text) {
    if (els.dragStatus) els.dragStatus.textContent = text;
  }

  function toggleTeachingRecord() {
    if (teachingRecording) {
      // Capture the release/final pose while recording is still active.
      recordTeachingWaypoint(true);
      teachingRecording = false;
    } else {
      teachingWaypoints = [];
      teachingStart = performance.now();
      teachingLastSample = 0;
      teachingPlayback = null;
      teachingRecording = true;
      if (!dragMode) toggleDragMode();
      recordTeachingWaypoint(true);
      if (els.teachExportText) els.teachExportText.value = '';
    }
    updateTeachingStatus();
  }

  function recordTeachingWaypoint(force) {
    if (!teachingRecording || !robot) return;
    const now = performance.now();
    if (!force && now - teachingLastSample < TEACH_SAMPLE_INTERVAL_MS) return;

    const tcp = getTcpPosition(robot);
    if (!tcp) return;
    const last = teachingWaypoints[teachingWaypoints.length - 1];
    if (!force && last && last.tcp && new THREE.Vector3(last.tcp.x, last.tcp.y, last.tcp.z).distanceTo(tcp) < TEACH_MIN_TCP_STEP) {
      return;
    }

    teachingLastSample = now;
    const ros = threeToRos(tcp);
    teachingWaypoints.push({
      t: Math.max(0, now - teachingStart),
      joints: { ...currentAngles },
      tcp: { x: tcp.x, y: tcp.y, z: tcp.z },
      tcp_ros: { x: ros.x, y: ros.y, z: ros.z }
    });
    updateTeachingStatus();
  }

  function replayTeaching() {
    if (!teachingWaypoints.length || !robot) {
      updateTeachingStatus('没有可回放的 waypoint');
      return;
    }
    teachingRecording = false;
    stopPath();
    moveStart = 0;
    const replayPoints = prepareTeachingReplay(teachingWaypoints);
    const playback = {
      points: replayPoints,
      index: 0,
      startedAt: performance.now(),
      startAngles: { ...currentAngles },
      rosControlled: false,
      feedbackDriven: false,
      smoothed: true
    };
    // DM starts its local playback state before doing any controller work.
    // Keep that ordering here so sending the ROS action cannot get a head
    // start over the browser animation.
    teachingPlayback = playback;
    let claimedByController = false;
    emitCommand({
      type: 'teaching-replay',
      waypoints: replayPoints,
      claim(options) {
        claimedByController = true;
        playback.rosControlled = true;
        playback.feedbackDriven = Boolean(options && options.feedbackDriven);
        updateTeachingStatus('示教轨迹已交给 ROS 执行');
      },
      complete(success, message) {
        if (teachingPlayback === playback && (playback.feedbackDriven || !success)) {
          teachingPlayback = null;
        }
        updateTeachingStatus(
          message || (success ? '真机示教回放完成' : '真机示教回放失败')
        );
      },
      stamp: performance.now()
    });
    updateTeachingStatus(
      claimedByController ? '正在同步回放网页动画和真机轨迹' : '正在回放示教轨迹'
    );
  }

  function prepareTeachingReplay(sourcePoints) {
    const source = sourcePoints
      .filter((point) => point && point.joints)
      .map((point) => ({
        t: Number(point.t) || 0,
        joints: { ...point.joints },
        tcp: point.tcp ? { ...point.tcp } : null,
        tcp_ros: point.tcp_ros ? { ...point.tcp_ros } : null
      }));
    if (source.length < 2) return source;

    const names = jointDefs.map((joint) => joint.name);
    const firstTime = source[0].t;
    source.forEach((point, index) => {
      point.t = Math.max(index ? source[index - 1].t + 1 : 0, point.t - firstTime);
    });

    // Zero-phase-like centered smoothing removes small IK reversals without
    // moving the recorded start/end poses.
    const smoothed = source.map((point, index) => {
      if (index === 0 || index === source.length - 1) return point;
      const joints = {};
      names.forEach((name) => {
        let weighted = 0;
        let totalWeight = 0;
        for (let offset = -2; offset <= 2; offset += 1) {
          const sampleIndex = clamp(index + offset, 0, source.length - 1);
          const value = Number(source[sampleIndex].joints[name]);
          if (!Number.isFinite(value)) continue;
          const weight = 3 - Math.abs(offset);
          weighted += value * weight;
          totalWeight += weight;
        }
        joints[name] = totalWeight ? weighted / totalWeight : Number(point.joints[name]) || 0;
      });
      return { ...point, joints };
    });

    const rawDurationMs = Math.max(1, smoothed[smoothed.length - 1].t);
    const sampleIntervalMs = Math.max(
      1000 / TEACH_REPLAY_SAMPLE_HZ,
      rawDurationMs / Math.max(TEACH_REPLAY_MAX_POINTS - 1, 1)
    );
    const resampled = [];
    let segment = 0;
    for (let sampleTime = 0; sampleTime < rawDurationMs; sampleTime += sampleIntervalMs) {
      while (segment < smoothed.length - 2 && smoothed[segment + 1].t < sampleTime) {
        segment += 1;
      }
      resampled.push(sampleTeachingSegment(smoothed, segment, sampleTime, names));
    }
    resampled.push({
      ...smoothed[smoothed.length - 1],
      joints: { ...smoothed[smoothed.length - 1].joints },
      rawT: rawDurationMs
    });

    // Match the controller's 0.6 rad/s action limit before sending. This keeps
    // the browser animation and hardware on the same (possibly stretched)
    // timeline instead of letting the backend silently change each segment.
    let scheduledSeconds = Math.max(
      0.15,
      maxArmJointDelta(currentAngles, resampled[0].joints) / TEACH_REPLAY_ARM_SPEED_RAD_S
    );
    resampled[0].t = scheduledSeconds * 1000;
    for (let index = 1; index < resampled.length; index += 1) {
      const recordedStep = Math.max(
        0.001,
        (resampled[index].rawT - resampled[index - 1].rawT) / 1000
      );
      const velocityStep = maxArmJointDelta(
        resampled[index - 1].joints,
        resampled[index].joints
      ) / TEACH_REPLAY_ARM_SPEED_RAD_S;
      scheduledSeconds += Math.max(recordedStep, velocityStep);
      resampled[index].t = scheduledSeconds * 1000;
    }
    return resampled.map(({ rawT, ...point }) => point);
  }

  function sampleTeachingSegment(points, index, sampleTime, names) {
    const p0 = points[Math.max(0, index - 1)];
    const p1 = points[index];
    const p2 = points[Math.min(points.length - 1, index + 1)];
    const p3 = points[Math.min(points.length - 1, index + 2)];
    const span = Math.max(1, p2.t - p1.t);
    const u = clamp((sampleTime - p1.t) / span, 0, 1);
    const joints = {};
    names.forEach((name) => {
      const v0 = Number(p0.joints[name]) || 0;
      const v1 = Number(p1.joints[name]) || 0;
      const v2 = Number(p2.joints[name]) || 0;
      const v3 = Number(p3.joints[name]) || 0;
      const u2 = u * u;
      const u3 = u2 * u;
      const value = 0.5 * (
        2 * v1 +
        (-v0 + v2) * u +
        (2 * v0 - 5 * v1 + 4 * v2 - v3) * u2 +
        (-v0 + 3 * v1 - 3 * v2 + v3) * u3
      );
      joints[name] = clamp(value, Math.min(v1, v2), Math.max(v1, v2));
    });
    return { t: sampleTime, rawT: sampleTime, joints };
  }

  function maxArmJointDelta(left, right) {
    let maximum = 0;
    jointDefs.forEach((joint) => {
      if (joint.name === 'gripper') return;
      const a = Number(left && left[joint.name]);
      const b = Number(right && right[joint.name]);
      if (Number.isFinite(a) && Number.isFinite(b)) {
        maximum = Math.max(maximum, Math.abs(b - a));
      }
    });
    return maximum;
  }

  function updateTeachingPlayback(now) {
    if (!teachingPlayback) return;
    const playback = teachingPlayback;
    // In hardware mode the solid robot follows interpolated /joint_states.
    // Keep this object only as replay lifecycle state; never overwrite actual
    // feedback with the ideal local trajectory.
    if (playback.feedbackDriven) return;
    const points = playback.points;
    if (!points.length) {
      teachingPlayback = null;
      updateTeachingStatus('回放完成');
      return;
    }

    // Use the absolute replay clock, matching FollowJointTrajectory's
    // time_from_start. A delayed frame therefore catches up immediately
    // instead of adding its delay to every following segment.
    const elapsed = Math.max(0, now - playback.startedAt);
    while (playback.index < points.length && elapsed >= Number(points[playback.index].t || 0)) {
      playback.index += 1;
    }

    if (playback.index >= points.length) {
      const finalPoint = points[points.length - 1];
      jointDefs.forEach((joint) => {
        const value = finalPoint.joints[joint.name];
        if (Number.isFinite(Number(value))) {
          setJoint(joint.name, Number(value), false, { source: 'teach-replay', emit: false });
        }
      });
      teachingPlayback = null;
      syncGhostToRobot();
      updateTeachingStatus('回放完成');
      return;
    }

    const right = points[playback.index];
    const left = playback.index > 0
      ? points[playback.index - 1]
      : { t: 0, joints: playback.startAngles };
    const leftTime = Math.max(0, Number(left.t) || 0);
    const rightTime = Math.max(leftTime + 1, Number(right.t) || leftTime + 1);
    const t = clamp((elapsed - leftTime) / (rightTime - leftTime), 0, 1);
    const eased = playback.smoothed
      ? t
      : t < 0.5
        ? 2 * t * t
        : 1 - Math.pow(-2 * t + 2, 2) / 2;
    jointDefs.forEach((joint) => {
      const start = left.joints[joint.name] ?? currentAngles[joint.name] ?? 0;
      const end = right.joints[joint.name] ?? start;
      setJoint(joint.name, start + (end - start) * eased, false, {
        source: 'teach-replay',
        emit: false
      });
    });
  }

  function exportTeachingWaypoints() {
    if (!teachingWaypoints.length) {
      updateTeachingStatus('没有可导出的 waypoint');
      return;
    }
    const jointNames = jointDefs.map((joint) => joint.name);
    const payload = {
      format: 'rebotarm_ros_waypoints_v1',
      frame_id: 'base_link',
      joint_names: jointNames,
      count: teachingWaypoints.length,
      waypoints: teachingWaypoints.map((point) => ({
        time_from_start: {
          sec: Math.floor(point.t / 1000),
          nanosec: Math.round((point.t % 1000) * 1e6)
        },
        positions: jointNames.map((name) => point.joints[name] ?? 0),
        tcp_ros: point.tcp_ros
      }))
    };
    const text = JSON.stringify(payload, null, 2);
    if (els.teachExportText) {
      els.teachExportText.value = text;
      els.teachExportText.focus();
      els.teachExportText.select();
    }
    if (navigator.clipboard && navigator.clipboard.writeText) {
      navigator.clipboard.writeText(text).catch(() => {});
    }
    updateTeachingStatus(t('sim.exported', { n: teachingWaypoints.length }));
  }

  function clearTeaching() {
    teachingRecording = false;
    teachingPlayback = null;
    teachingWaypoints = [];
    if (els.teachExportText) els.teachExportText.value = '';
    updateTeachingStatus();
  }

  function updateTeachingStatus(message) {
    if (els.teachRecord) {
      els.teachRecord.textContent = teachingRecording ? t('sim.stopRecord') : t('teach.record');
      els.teachRecord.classList.toggle('active', teachingRecording);
    }
    if (!els.teachStatus) return;
    if (message) {
      els.teachStatus.textContent = message;
    } else if (teachingRecording) {
      els.teachStatus.textContent = t('sim.recording', { n: teachingWaypoints.length });
    } else if (teachingPlayback) {
      els.teachStatus.textContent = t('sim.replaying');
    } else if (teachingWaypoints.length) {
      const duration = teachingWaypoints[teachingWaypoints.length - 1].t / 1000;
      els.teachStatus.textContent = t('sim.recorded', { n: teachingWaypoints.length, sec: duration.toFixed(1) });
    } else {
      els.teachStatus.textContent = t('teach.status');
    }
  }

  function planTcpMoveTo(target, label) {
    stopPath();
    moveStart = 0;
    showTargetGhost(target);

    const start = { ...currentAngles };
    const solved = solveIKTarget(target, 240, 900);
    applyRobotAngles(robot, start);
    Object.entries(start).forEach(([name, value]) => {
      setJoint(name, value, false, { source: 'ros' });
    });

    if (!solved || !solved.angles) {
      setDragStatus(t('sim.unreachable', { label: label || '点击位置' }));
      return;
    }

    moveToAngles({ ...currentAngles, ...solved.angles }, 900);
    setDragStatus(t('sim.planTarget', { label: label || '点击位置', mm: (solved.error * 1000).toFixed(1) }));
  }

  function solveIKTarget(target, maxIter, timeoutMs) {
    const started = performance.now();
    let result = null;

    for (let i = 0; i < maxIter; i += 1) {
      result = IKSolver.servoStep(target, 0.016, { source: 'solver' });
      if (result && result.reached) break;
      if (performance.now() - started > timeoutMs) break;
    }

    return {
      angles: { ...currentAngles },
      error: result ? result.error : Infinity,
      reached: result ? result.reached : false
    };
  }

  function applyRobotAngles(root, angles) {
    if (!root) return;
    IKSolver.jointNames.forEach((name) => {
      const joint = getJoint(root, name);
      if (!joint) return;
      const value = angles[name] ?? 0;
      if (typeof joint.setJointValue === 'function') {
        joint.setJointValue(value);
      } else if (typeof joint.setAngle === 'function') {
        joint.setAngle(value);
      }
    });
    root.updateMatrixWorld(true);
  }

  const IKSolver = {
    jointNames: jointDefs.filter((joint) => joint.name !== 'gripper').map((joint) => joint.name),
    gain: 12,
    minDamping: 0.018,
    maxDamping: 0.075,
    singularityThreshold: 0.08,
    maxJointSpeed: 2.8,

    servoStep(target, dt, options) {
      if (!target || !robot || dt <= 0) return null;
      const current = getTcpPosition(robot);
      if (!current) return null;
      const error = new THREE.Vector3().subVectors(target, current);
      const errorNorm = error.length();
      if (errorNorm < 0.0015) return { error: errorNorm, reached: true };

      const stepError = error.multiplyScalar(Math.min(0.65, Math.max(0.08, this.gain * dt)));
      const jacobian = this.computeJacobian(currentAngles);
      const delta = this.solveDampedLeastSquares(jacobian, stepError);
      if (!delta) return { error: errorNorm, reached: false };

      this.jointNames.forEach((name, index) => {
        const def = jointDefs.find((joint) => joint.name === name);
        const limitedDelta = clamp(delta[index] || 0, -this.maxJointSpeed * dt, this.maxJointSpeed * dt);
        setJoint(
          name,
          clamp((currentAngles[name] || 0) + limitedDelta, def.min, def.max),
          false,
          {
            source: options && options.source ? options.source : 'drag',
            emit: Boolean(options && options.emit),
            phase: options && options.phase ? options.phase : 'commit'
          }
        );
      });

      robot.updateMatrixWorld(true);
      const after = getTcpPosition(robot);
      const afterError = after ? after.distanceTo(target) : errorNorm;
      return { error: afterError, reached: afterError < 0.0015 };
    },

    computeJacobian(baseAngles) {
      const eps = 0.004;
      const saved = { ...baseAngles };
      const rows = [[], [], []];

      this.jointNames.forEach((name, index) => {
        const plus = { ...saved, [name]: (saved[name] || 0) + eps };
        const minus = { ...saved, [name]: (saved[name] || 0) - eps };

        applyRobotAngles(robot, plus);
        const plusPos = getTcpPosition(robot);
        applyRobotAngles(robot, minus);
        const minusPos = getTcpPosition(robot);

        rows[0][index] = plusPos && minusPos ? (plusPos.x - minusPos.x) / (2 * eps) : 0;
        rows[1][index] = plusPos && minusPos ? (plusPos.y - minusPos.y) / (2 * eps) : 0;
        rows[2][index] = plusPos && minusPos ? (plusPos.z - minusPos.z) / (2 * eps) : 0;
      });

      applyRobotAngles(robot, saved);
      return rows;
    },

    solveDampedLeastSquares(j, error) {
      const gram = [
        [dotRows(j[0], j[0]), dotRows(j[0], j[1]), dotRows(j[0], j[2])],
        [dotRows(j[1], j[0]), dotRows(j[1], j[1]), dotRows(j[1], j[2])],
        [dotRows(j[2], j[0]), dotRows(j[2], j[1]), dotRows(j[2], j[2])]
      ];
      // det(JJ^T) normalized by its mean eigenvalue is scale-independent:
      // close to zero means the TCP Jacobian is approaching a singular pose.
      const scale = Math.max((gram[0][0] + gram[1][1] + gram[2][2]) / 3, 1e-9);
      const normalizedDeterminant = Math.max(0, determinant3x3(gram)) /
        Math.max(scale * scale * scale, 1e-12);
      const singularity = clamp(
        1 - normalizedDeterminant / this.singularityThreshold,
        0,
        1
      );
      const damping = this.minDamping +
        (this.maxDamping - this.minDamping) * singularity * singularity;
      const lambda2 = damping * damping;
      const a = [
        [
          gram[0][0] + lambda2,
          gram[0][1],
          gram[0][2]
        ],
        [
          gram[1][0],
          gram[1][1] + lambda2,
          gram[1][2]
        ],
        [
          gram[2][0],
          gram[2][1],
          gram[2][2] + lambda2
        ]
      ];
      const y = solve3x3(a, [error.x, error.y, error.z]);
      if (!y) return null;
      return this.jointNames.map((name, index) => j[0][index] * y[0] + j[1][index] * y[1] + j[2][index] * y[2]);
    }
  };

  function dotRows(a, b) {
    return a.reduce((sum, value, index) => sum + value * (b[index] || 0), 0);
  }

  function determinant3x3(a) {
    return (
      a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) -
      a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) +
      a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0])
    );
  }

  function solve3x3(a, b) {
    const det =
      a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) -
      a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) +
      a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
    if (Math.abs(det) < 1e-9) return null;

    const inv = [
      [
        (a[1][1] * a[2][2] - a[1][2] * a[2][1]) / det,
        (a[0][2] * a[2][1] - a[0][1] * a[2][2]) / det,
        (a[0][1] * a[1][2] - a[0][2] * a[1][1]) / det
      ],
      [
        (a[1][2] * a[2][0] - a[1][0] * a[2][2]) / det,
        (a[0][0] * a[2][2] - a[0][2] * a[2][0]) / det,
        (a[0][2] * a[1][0] - a[0][0] * a[1][2]) / det
      ],
      [
        (a[1][0] * a[2][1] - a[1][1] * a[2][0]) / det,
        (a[0][1] * a[2][0] - a[0][0] * a[2][1]) / det,
        (a[0][0] * a[1][1] - a[0][1] * a[1][0]) / det
      ]
    ];

    return [
      inv[0][0] * b[0] + inv[0][1] * b[1] + inv[0][2] * b[2],
      inv[1][0] * b[0] + inv[1][1] * b[1] + inv[1][2] * b[2],
      inv[2][0] * b[0] + inv[2][1] * b[1] + inv[2][2] * b[2]
    ];
  }

  function updateJointLabel(name) {
    const def = jointDefs.find((item) => item.name === name);
    const label = document.getElementById(`${name}-value`);
    if (!label || !def) return;
    if (def.unit === 'm') {
      const widthMm = currentAngles[name] * 1000;
      label.textContent = t('joint.gripSuffix', { val: widthMm.toFixed(0) });
      const readout = document.getElementById('gripper-width');
      if (readout) readout.textContent = t('joint.gripSuffix', { val: widthMm.toFixed(0) });
      return;
    }
    label.textContent = t('joint.degSuffix', { val: (currentAngles[name] * RAD).toFixed(1) });
  }

  function setGripperWidth(widthM) {
    stopPath();
    moveToAngles({ ...currentAngles, gripper: clamp(widthM, 0, GRIPPER_COMMAND_MAX) }, 450);
  }

 function emitCommand(command) {
   commandListeners.forEach((listener) => {
     try {
       listener({ ...command });
     } catch (error) {
       console.warn('Command listener failed:', error);
     }
   });
 }

  function emitJointBatch(angles, source, label) {
    const joints = {};
    jointDefs.forEach((joint) => {
      if (typeof angles[joint.name] === 'number') {
        joints[joint.name] = angles[joint.name];
      }
    });
    if (!Object.keys(joints).length) return;
    emitCommand({
      type: 'joint-batch',
      joints,
      source,
      label,
      stamp: performance.now()
    });
  }

  function playPath() {
    const sequence = ['ready', 'left', 'inspect', 'forward', 'right', 'ready'];
    animation = { sequence, index: 0, nextAt: 0 };
  }

  function stopPath() {
    animation = null;
  }

  function stopActiveMotion() {
    stopPath();
    teachingPlayback = null;
    moveStart = 0;
    gripperMotion = null;
    draggingTcp = false;
    dragSettling = false;
    setGhostDisplay(false);
    if (dragMode) toggleDragMode();
    updateTeachingStatus();
  }

  function updatePath(now) {
    if (!animation || moveStart) return;
    if (now < animation.nextAt) return;
    applyPreset(animation.sequence[animation.index]);
    animation.index = (animation.index + 1) % animation.sequence.length;
    animation.nextAt = now + 1350;
  }

  function updateTcpHud() {
    const pos = getTcpPosition(robot);
    if (!pos) return;
    tcpMarker.position.copy(pos);

    const ros = threeToRos(pos);
    const planar = Math.sqrt(ros.x * ros.x + ros.y * ros.y);
    const spatial = Math.sqrt(ros.x * ros.x + ros.y * ros.y + ros.z * ros.z);
    els.tcp.textContent = `X ${mm(ros.x)} / Y ${mm(ros.y)} / Z ${mm(ros.z)}`;
    els.reach.textContent = t('sim.reachText', { planar: Math.round(planar * 1000), workspace: Math.round(workspacePlanarReach * 1000), spatial: Math.round(spatial * 1000) });
    els.reach.style.color = planar <= workspacePlanarReach ? '#d7fff4' : '#ffd1c9';
  }

  function getTcpPosition(root) {
    if (!root) return null;
    const link = root.getObjectByName('gripper_end') || root.getObjectByName('end_link') || root.getObjectByName('link6') || root;
    link.updateMatrixWorld(true);
    const pos = new THREE.Vector3();
    link.getWorldPosition(pos);
    return pos;
  }

  function threeToRos(v) {
    return { x: v.x, y: -v.z, z: v.y };
  }

  function mm(value) {
    return t('sim.mmShort', { val: Math.round(value * 1000) });
  }

  function getEndLink(root) {
    if (!root) return null;
    return root.getObjectByName('end_link') || root.getObjectByName('link6') || root;
  }

  function getFakeGraspPosition(root) {
    const link = getEndLink(root);
    if (!link) return null;
    link.updateMatrixWorld(true);
    return link.localToWorld(FAKE_GRASP_LOCAL_OFFSET.clone());
  }

  function updateCarriedObject() {
    if (!carriedObject || !carriedObject.mesh || !robot) return;
    const grip = getFakeGraspPosition(robot) || getTcpPosition(robot);
    if (!grip) return;
    carriedObject.mesh.position.lerp(grip, 0.55);
  }

  function attachObject(color) {
    const key = String(color || '').toLowerCase();
    const mesh = taskObjects.get(key);
    if (!mesh) return false;
    if (carriedObject && carriedObject.mesh !== mesh) {
      releaseObject({ settleOnTable: true });
    }
    carriedObject = { color: key, mesh };
    mesh.userData.fakeCarried = true;
    updateCarriedObject();
    return true;
  }

  function releaseObject(options) {
    if (!carriedObject || !carriedObject.mesh) {
      carriedObject = null;
      return false;
    }
    const mesh = carriedObject.mesh;
    mesh.userData.fakeCarried = false;
    if (!options || options.settleOnTable !== false) {
      mesh.position.y = Number(mesh.userData.tableY) || mesh.position.y;
    }
    carriedObject = null;
    return true;
  }

  function animate(now) {
    requestAnimationFrame(animate);
    const frameNow = now || performance.now();
    updateMotion(frameNow);
    updateGripperMotion(frameNow);
    updatePath(frameNow);
    updateTeachingPlayback(frameNow);
    updateDragSettling(frameNow);
    updateAxisLabelVisibility(frameNow);
    if (robot) {
      robot.updateMatrixWorld(true);
      if (teachingRecording) recordTeachingWaypoint(false);
      updateTcpHud();
      updateCarriedObject();
      updateDragMarker();
      updateDragErrorLine();
    }
    if (controls) controls.update();
    renderer.render(scene, camera);
  }

  function updateAxisLabelVisibility(now) {
    axisLabelSprites.forEach((sprite) => {
      const hideAt = sprite.userData.autoHideAt || 0;
      const fadeDuration = sprite.userData.fadeDuration || 900;
      if (now <= hideAt) return;

      const progress = clamp((now - hideAt) / fadeDuration, 0, 1);
      const opacity = 1 - progress;
      sprite.material.opacity = opacity;
      sprite.scale.set(0.16 * (1 + progress * 0.12), 0.05 * (1 + progress * 0.12), 1);
      sprite.visible = opacity > 0.02;
    });
  }

  function resize() {
    camera.aspect = getAspect();
    camera.updateProjectionMatrix();
    renderer.setSize(els.host.clientWidth, els.host.clientHeight);
  }

  function resetCamera() {
    if (!camera) return;
    camera.position.set(-0.72, 0.48, 0.74);
    camera.lookAt(0.18, 0.18, 0);
    if (controls) {
      controls.target.set(0.18, 0.18, 0);
      controls.sync();
    }
  }

  function getAspect() {
    return Math.max(1, els.host.clientWidth) / Math.max(1, els.host.clientHeight);
  }

  function clamp(value, min, max) {
    return Math.max(min, Math.min(max, value));
  }

  function makeTextSprite(text, color) {
    const canvas = document.createElement('canvas');
    canvas.width = 512;
    canvas.height = 160;
    const ctx = canvas.getContext('2d');
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.fillStyle = 'rgba(17, 18, 17, 0.72)';
    roundRect(ctx, 10, 24, 492, 92, 14);
    ctx.fill();
    ctx.font = '700 38px "Microsoft YaHei", Arial';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillStyle = `#${color.toString(16).padStart(6, '0')}`;
    ctx.fillText(text, 256, 70);

    const texture = new THREE.CanvasTexture(canvas);
    const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: texture, transparent: true, depthTest: false }));
    sprite.scale.set(0.16, 0.05, 1);
    return sprite;
  }

  function roundRect(ctx, x, y, width, height, radius) {
    ctx.beginPath();
    ctx.moveTo(x + radius, y);
    ctx.lineTo(x + width - radius, y);
    ctx.quadraticCurveTo(x + width, y, x + width, y + radius);
    ctx.lineTo(x + width, y + height - radius);
    ctx.quadraticCurveTo(x + width, y + height, x + width - radius, y + height);
    ctx.lineTo(x + radius, y + height);
    ctx.quadraticCurveTo(x, y + height, x, y + height - radius);
    ctx.lineTo(x, y + radius);
    ctx.quadraticCurveTo(x, y, x + radius, y);
    ctx.closePath();
  }

  function setGripperWidth(widthM, options) {
    const target = clamp(widthM, 0, GRIPPER_COMMAND_MAX);
    const source = options && options.source ? options.source : 'gripper';
    const immediate = options && options.immediate;
    const emit = !(options && options.emit === false);

    gripperMotion = null;
    if (immediate) {
      setJoint('gripper', target, false, { source, emit });
      syncGhostToRobot();
      return;
    }

    gripperMotion = {
      start: currentAngles.gripper || 0,
      target,
      startedAt: performance.now(),
      duration: options && options.duration ? Math.max(Number(options.duration), 1) : GRIPPER_ANIMATION_MS,
      source,
      emit
    };
  }

  function updateGripperMotion(now) {
    if (!gripperMotion) return;

    const t = clamp((now - gripperMotion.startedAt) / gripperMotion.duration, 0, 1);
    const eased = t < 0.5 ? 2 * t * t : 1 - Math.pow(-2 * t + 2, 2) / 2;
    const value = gripperMotion.start + (gripperMotion.target - gripperMotion.start) * eased;
    setJoint('gripper', value, false, {
      source: gripperMotion.source,
      emit: gripperMotion.emit && t >= 1
    });
    syncGhostToRobot();

    if (t >= 1) gripperMotion = null;
  }

  window.reBotSim = {
    getAngles() {
      return { ...currentAngles };
    },
    getJointDefs() {
      return jointDefs.map((joint) => ({ ...joint }));
    },
    getTeachingWaypoints() {
      return teachingWaypoints.map((point) => ({
        ...point,
        joints: { ...point.joints },
        tcp_ros: { ...point.tcp_ros }
      }));
    },
    setAngles(angles, options) {
      if (!angles || typeof angles !== 'object') return;
      const source = options && options.source ? options.source : 'api';
      const forceFeedback = Boolean(options && options.forceFeedback);
      if (
        source === 'ros' &&
        !forceFeedback &&
        (teachingPlayback || moveStart || animation || draggingTcp || dragSettling || gripperMotion)
      ) return;
      if (options && options.animate) {
        if (source !== 'ros') {
          stopPath();
          teachingPlayback = null;
        }
        // Keep targets already in flight so feedback arriving joint-by-joint
        // converges as one transition instead of cancelling earlier joints.
        const nextAngles = { ...(moveStart ? targetAngles : currentAngles) };
        Object.entries(angles).forEach(([name, value]) => {
          if (Number.isFinite(Number(value))) nextAngles[name] = Number(value);
        });
        moveToAngles(nextAngles, Number(options.duration) || 420, {
          source,
          emitBatch: false
        });
        return;
      }
      if (source !== 'ros') {
        stopPath();
        teachingPlayback = null;
        moveStart = 0;
      }
      Object.entries(angles).forEach(([name, value]) => {
        setJoint(name, value, Boolean(options && options.preserveControls), options || {});
      });
      if (!(options && options.preserveGhost)) syncGhostToRobot();
    },
    setGripperWidth(widthM, options) {
      const source = options && options.source ? options.source : 'api';
      const forceFeedback = Boolean(options && options.forceFeedback);
      if (
        source === 'ros' &&
        !forceFeedback &&
        (teachingPlayback || moveStart || animation || draggingTcp || dragSettling || gripperMotion)
      ) return;
      if (source !== 'ros') {
        stopPath();
        teachingPlayback = null;
        moveStart = 0;
      }
      if (options && options.animate) {
        setGripperWidth(widthM, options);
      } else {
        setJoint(
          'gripper',
          clamp(widthM, 0, GRIPPER_COMMAND_MAX),
          Boolean(options && options.preserveControls),
          options || {}
        );
        if (!(options && options.preserveGhost)) syncGhostToRobot();
      }
    },
    setFeedbackTarget(angles) {
      if (!angles || typeof angles !== 'object') return;
      const target = { ...currentAngles };
      Object.entries(angles).forEach(([name, value]) => {
        if (Number.isFinite(Number(value))) target[name] = Number(value);
      });
      updateGhostTarget(target);
      setGhostDisplay(true);
    },
    setGhostVisible(visible) {
      setGhostDisplay(visible);
    },
    setHardwareFeedbackDriven(enabled) {
      hardwareFeedbackDriven = Boolean(enabled);
      if (hardwareFeedbackDriven) setGhostDisplay(false);
    },
    isGhostVisible() {
      return Boolean(ghostRobot && ghostRobot.visible);
    },
    getVisualDiagnostics() {
      const left = getJoint(robot, 'gripper_joint1');
      const right = getJoint(robot, 'gripper_joint2');
      const readJoint = (joint) => {
        const value = joint?.jointValue?.[0];
        return Number.isFinite(Number(value)) ? Number(value) : NaN;
      };
      return {
        mujocoSceneVisible: Boolean(mujocoSceneGroup && mujocoSceneGroup.visible),
        table: {
          centerX: TABLE_CENTER_X,
          width: TABLE_WIDTH,
          depth: TABLE_DEPTH,
          surfaceY: TABLE_SURFACE_Y,
          thickness: TABLE_THICKNESS
        },
        taskObjects: Object.fromEntries(
          Array.from(taskObjects.entries()).map(([key, mesh]) => [key, {
            x: mesh.position.x,
            y: mesh.position.y,
            z: mesh.position.z
          }])
        ),
        gripper: {
          commandWidth: currentAngles.gripper,
          leftTravel: readJoint(left),
          rightTravel: readJoint(right)
        }
      };
    },
    generateTrajectory,
    setDragMode(enabled) {
      if (Boolean(enabled) !== dragMode) toggleDragMode();
    },
    attachObject(color) {
      return attachObject(color);
    },
    releaseObject(options) {
      return releaseObject(options || {});
    },
    getCarriedObject() {
      return carriedObject ? carriedObject.color : null;
    },
    stopMotion() {
      stopActiveMotion();
    },
    onCommand(listener) {
      if (typeof listener !== 'function') return () => {};
      commandListeners.add(listener);
      return () => commandListeners.delete(listener);
    }
  };
  if (window.rebotI18n) {
    window.rebotI18n.onLangChange(() => {
      Object.entries(presets).forEach(([key, preset], index) => {
        const btn = els.presets && els.presets.children[index];
        if (btn) btn.textContent = t(preset.label);
      });
      jointDefs.forEach((joint, index) => {
        const wrap = els.joints && els.joints.children[index];
        if (wrap) {
          const strong = wrap.querySelector('strong');
          if (strong) strong.textContent = t(joint.label);
        }
        updateJointLabel(joint.name);
      });
      if (typeof updateTeachingStatus === 'function') updateTeachingStatus();
      if (els.toggleDrag) {
        els.toggleDrag.textContent = dragMode ? t('sim.exitDrag') : t('adv.drag');
        els.toggleDrag.classList.toggle('active', dragMode);
      }
      if (!draggingTcp && !dragSettling) {
        setDragStatus(dragMode ? t('sim.dragGreen') : t('app.dragDisabled'));
      }
      if (robot && typeof updateTcpHud === 'function') updateTcpHud();
    });
  }



  function createOrbit(cam, dom, initialTarget) {
    let rotating = false;
    let panning = false;
    let lastX = 0;
    let lastY = 0;
    const target = initialTarget.clone();
    const spherical = new THREE.Spherical();
    const offset = new THREE.Vector3();

    function sync() {
      offset.copy(cam.position).sub(target);
      spherical.setFromVector3(offset);
    }
    sync();

    dom.addEventListener('pointerdown', (event) => {
      dom.setPointerCapture(event.pointerId);
      rotating = event.button === 0;
      panning = event.button === 2;
      lastX = event.clientX;
      lastY = event.clientY;
    });

    dom.addEventListener('pointermove', (event) => {
      const dx = event.clientX - lastX;
      const dy = event.clientY - lastY;
      lastX = event.clientX;
      lastY = event.clientY;

      if (rotating) {
        spherical.theta -= dx * 0.006;
        spherical.phi = clamp(spherical.phi - dy * 0.006, 0.12, Math.PI - 0.08);
      }

      if (panning) {
        const distance = cam.position.distanceTo(target);
        const right = new THREE.Vector3();
        const up = new THREE.Vector3(0, 1, 0);
        cam.getWorldDirection(right).cross(up).normalize();
        target.add(right.multiplyScalar(-dx * distance * 0.0015));
        target.y += dy * distance * 0.0015;
      }
    });

    dom.addEventListener('pointerup', (event) => {
      rotating = false;
      panning = false;
      if (dom.hasPointerCapture(event.pointerId)) dom.releasePointerCapture(event.pointerId);
    });

    dom.addEventListener('wheel', (event) => {
      event.preventDefault();
      spherical.radius = clamp(spherical.radius * (event.deltaY > 0 ? 1.08 : 0.92), 0.24, 4);
    }, { passive: false });

    dom.addEventListener('contextmenu', (event) => event.preventDefault());

    return {
      target,
      sync,
      update() {
        offset.setFromSpherical(spherical);
        cam.position.copy(target).add(offset);
        cam.lookAt(target);
      }
    };
  }
})();
