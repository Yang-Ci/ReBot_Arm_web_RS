#!/usr/bin/env node

const fs = require('fs');
const path = require('path');

const ROOT = path.resolve(__dirname, '..');
const PUBLIC_DIR = path.join(ROOT, 'public');
const ROS_BRINGUP_DIR = path.join(
  ROOT,
  '..',
  'rebotarm_ros2_RS',
  'src',
  'rebotarm_bringup'
);
const ROS_URDF = path.join(
  ROS_BRINGUP_DIR,
  'description',
  'urdf',
  'ReBot_Arm_RS.urdf'
);
const ROS_MESHES = path.join(
  ROS_BRINGUP_DIR,
  'description',
  'meshes_rs'
);
const FALLBACK_URDF = path.join(
  ROOT,
  'description',
  'urdf',
  'ReBot_Arm_RS.urdf'
);
const FALLBACK_MESHES = path.join(ROOT, 'description', 'meshes_rs');
const GRIPPER_MESHES = path.join(
  ROOT,
  'split_meshes',
  'grouped_gripper'
);
const outputDir = path.resolve(process.argv[2] || path.join(ROOT, 'dist'));

function requiredFile(filePath) {
  if (!fs.existsSync(filePath) || !fs.statSync(filePath).isFile()) {
    throw new Error(`Required file is missing: ${filePath}`);
  }
  return filePath;
}

function requiredDir(filePath) {
  if (!fs.existsSync(filePath) || !fs.statSync(filePath).isDirectory()) {
    throw new Error(`Required directory is missing: ${filePath}`);
  }
  return filePath;
}

function copyTree(source, destination) {
  fs.mkdirSync(path.dirname(destination), { recursive: true });
  fs.cpSync(source, destination, { recursive: true });
}

const urdfFile = requiredFile(
  fs.existsSync(ROS_URDF) ? ROS_URDF : FALLBACK_URDF
);
const meshesDir = requiredDir(
  fs.existsSync(ROS_MESHES) ? ROS_MESHES : FALLBACK_MESHES
);

requiredDir(PUBLIC_DIR);
requiredDir(GRIPPER_MESHES);

fs.rmSync(outputDir, { recursive: true, force: true });
fs.mkdirSync(outputDir, { recursive: true });
copyTree(PUBLIC_DIR, outputDir);
copyTree(urdfFile, path.join(outputDir, 'api', 'urdf'));
copyTree(meshesDir, path.join(outputDir, 'api', 'description', 'meshes_rs'));
copyTree(GRIPPER_MESHES, path.join(outputDir, 'api', 'gripper_meshes'));

console.log(`RS static site written to ${outputDir}`);
