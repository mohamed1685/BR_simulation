#!/usr/bin/env python3
"""Generate urdf/due_wheel.urdf from the raw SolidWorks export + the STL meshes.

Run from the package root after re-exporting from SolidWorks:

    python3 tools/generate_urdf.py

What it does (all numbers are measured, nothing is hand-copied):
  * base_link origin  = geometric (bounding-box) centre of base_link.STL
  * base_link axes    = REP-103: +X forward, +Y left, +Z up
                        (CAD "front" parts sit at lower X, so the CAD frame is rotated 180 deg about Z)
  * wheel / due link frames sit on their own axle, axes parallel to base_link, joint axis = +Y
  * mass, COM and inertia are computed from each mesh (uniform density) and scaled to MASS below
  * mecanum friction + gz-sim plugins are written into the <gazebo> blocks
"""
import os
import struct
import xml.etree.ElementTree as ET

import numpy as np

# ------------------------------------------------------------------ EDIT ME
MASS = {'base': 22.0, 'wheel': 1.0, 'due': 2.0}     # kg  (total = 30 kg). PLACEHOLDERS - use real weights
WHEEL_RADIUS = 0.0635                               # m   (nm127 mecanum wheel, 127 mm)
WHEEL_WIDTH = 0.0687                                # m
MU, MU2 = 1.0, 0.0                                  # roller friction: grip along fdir1 / free rolling across
FRONT_IS_MINUS_X = True                             # CAD: 'front' parts sit at lower X
# ----------------------------------------------------------------------------

PKG = 'robocon27_simulation'
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MESH_DIR = os.path.join(ROOT, 'meshes')
SRC_URDF = os.path.join(ROOT, 'cad_export', 'solidworks_export.urdf')
OUT = os.path.join(ROOT, 'urdf', 'due_wheel.urdf')

# CAD link name -> (our link name, mesh kind)
ROLES_DUE = {'frontdue': 'front_due', 'backduewheels': 'back_due'}
WHEEL_LINKS = ['1normalwheelf', '2normalwheelf', '1normalwheelb', '2wheelnormalbac']


# ------------------------------------------------------------------ maths helpers
def load_stl(fn):
    raw = open(fn, 'rb').read()
    n = struct.unpack('<I', raw[80:84])[0]
    assert 84 + 50 * n == len(raw), f'{fn}: not a binary STL'
    d = np.frombuffer(raw[84:84 + 50 * n],
                      dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]))
    return d['v'].astype(float)                      # (n,3,3)


def rot(r, p, y):
    cx, sx, cy, sy, cz, sz = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def rpy_from(R):
    """Inverse of rot(); handles gimbal lock."""
    if abs(R[2, 0]) < 0.999999:
        p = -np.arcsin(R[2, 0])
        return np.array([np.arctan2(R[2, 1], R[2, 2]), p, np.arctan2(R[1, 0], R[0, 0])])
    if R[2, 0] < 0:                                   # pitch = +90 deg
        return np.array([np.arctan2(R[0, 1], R[1, 1]), np.pi / 2, 0.0])
    return np.array([np.arctan2(-R[0, 1], R[1, 1]), -np.pi / 2, 0.0])   # pitch = -90 deg


def mesh_mass_props(tri, mass):
    """COM and inertia about COM of a closed triangle mesh, uniform density."""
    a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
    v = np.einsum('ij,ij->i', a, np.cross(b, c)) / 6.0
    V = v.sum()
    if V <= 0:
        raise ValueError('non-positive mesh volume (open mesh or inverted normals)')
    S = a + b + c
    com = (v[:, None] * S / 4.0).sum(0) / V
    Q = (np.einsum('ni,nj->nij', a, a) + np.einsum('ni,nj->nij', b, b) + np.einsum('ni,nj->nij', c, c))
    C = (v[:, None, None] / 20.0 * (np.einsum('ni,nj->nij', S, S) + Q)).sum(0)
    C = C - V * np.outer(com, com)
    I = (np.trace(C) * np.eye(3) - C) * (mass / V)
    return com, I, V


def f(x):
    x = round(float(x), 6)
    if x == 0:
        x = 0.0
    return f'{x:.6f}'.rstrip('0').rstrip('.') if x != int(x) else str(int(x))


def vec(v):
    return ' '.join(f(x) for x in v)


def inertial_xml(com, mass, I):
    return (f'    <inertial>\n'
            f'      <origin xyz="{vec(com)}" rpy="0 0 0"/>\n'
            f'      <mass value="{f(mass)}"/>\n'
            f'      <inertia ixx="{f(I[0,0])}" ixy="{f(I[0,1])}" ixz="{f(I[0,2])}" '
            f'iyy="{f(I[1,1])}" iyz="{f(I[1,2])}" izz="{f(I[2,2])}"/>\n'
            f'    </inertial>')


def mesh_uri(name):
    return f'package://{PKG}/meshes/{name}'


# ------------------------------------------------------------------ read CAD export
root = ET.parse(SRC_URDF).getroot()
links = {l.get('name'): l for l in root.findall('link')}
joint_of = {}                                       # child link -> (xyz, rpy)
for j in root.findall('joint'):
    o = j.find('origin')
    joint_of[j.find('child').get('link')] = (
        np.array([float(x) for x in o.get('xyz').split()]),
        np.array([float(x) for x in o.get('rpy').split()]))


def mesh_file(link_name):
    return os.path.basename(links[link_name].find('visual/geometry/mesh').get('filename'))


# ------------------------------------------------------------------ base frame
base_tri = load_stl(os.path.join(MESH_DIR, mesh_file('base_link')))
pts = base_tri.reshape(-1, 3)
C0 = (pts.max(0) + pts.min(0)) / 2.0                # geometric centre of the base (CAD frame)
FLIP = np.diag([-1.0, -1.0, 1.0]) if FRONT_IS_MINUS_X else np.eye(3)   # Rz(pi) or identity
print('base geometric centre in CAD frame:', C0.round(5))

# base_link: p_new = FLIP @ (p_cad - C0)
base_tri_new = np.einsum('ij,nkj->nki', FLIP, base_tri - C0)
com_b, I_b, vol = mesh_mass_props(base_tri_new, MASS['base'])
print('base mesh volume %.4f m^3, COM offset from centre %s' % (vol, com_b.round(4)))
base_vis_xyz = FLIP @ (-C0)
base_vis_rpy = rpy_from(FLIP)

parts = []
parts.append(f'''<?xml version="1.0"?>
<!--
  Generated by tools/generate_urdf.py from cad_export/solidworks_export.urdf and meshes/*.STL.  Do not hand-edit.
  Frames: every link frame is axis-aligned with base_link (X forward, Y left, Z up) and sits on its own axle.
  base_link origin = geometric (bounding-box) centre of base_link.STL; the floor is @@DROP@@ m below it.
  All wheel / due joints rotate about +Y: a POSITIVE velocity drives the robot FORWARD (+X).
-->
<robot name="due_wheel" xmlns:gz="http://gazebosim.org/schema">
''')


parts.append(f'''  <!-- ============================== BASE ============================== -->
  <link name="base_link">
{inertial_xml(com_b, MASS['base'], I_b)}
    <visual>
      <origin xyz="{vec(base_vis_xyz)}" rpy="{vec(base_vis_rpy)}"/>
      <geometry><mesh filename="{mesh_uri(mesh_file('base_link'))}"/></geometry>
      <material name="base_mat"><color rgba="0.792 0.820 0.933 1"/></material>
    </visual>
    <collision>
      <origin xyz="{vec(base_vis_xyz)}" rpy="{vec(base_vis_rpy)}"/>
      <geometry><mesh filename="{mesh_uri(mesh_file('base_link'))}"/></geometry>
    </collision>
  </link>
''')


# ------------------------------------------------------------------ generic child (axle-centred frame)
def child(link_name, pivot_local, mass):
    """Return (joint_xyz_in_base, M, mesh_tri_local) for a CAD child link.

    M maps mesh-local coords into base_link-aligned axes: p_base = M p_mesh + t.
    The new link frame sits at the point `pivot_local` (mesh coords) and is aligned with base_link.
    """
    o_cad, rpy_cad = joint_of[link_name]
    M = FLIP @ rot(*rpy_cad)
    t = FLIP @ (o_cad - C0)
    tri = load_stl(os.path.join(MESH_DIR, mesh_file(link_name)))
    ctr = (tri.reshape(-1, 3).max(0) + tri.reshape(-1, 3).min(0)) / 2.0
    pivot = ctr if pivot_local == 'centre' else np.zeros(3)
    w = M @ pivot + t                                # link frame origin, in base_link
    return M, t, w, tri


wheel_info = {}
for ln in WHEEL_LINKS:
    M, t, w, tri = child(ln, 'centre', MASS['wheel'])
    ax = M @ np.array([0, 0, 1.0])
    assert np.allclose(ax, [0, 1, 0], atol=1e-3), f'{ln}: spin axis is {ax}, expected +Y'
    wheel_info[ln] = (M, t, w, tri)

# geometric role of each wheel (front/rear by X, left/right by Y)
xs = sorted({round(wheel_info[l][2][0], 2) for l in WHEEL_LINKS})
role = {}
for ln in WHEEL_LINKS:
    w = wheel_info[ln][2]
    fr = 'front' if w[0] > 0 else 'rear'
    lr = 'left' if w[1] > 0 else 'right'
    role[ln] = f'{fr}_{lr}'
assert sorted(role.values()) == ['front_left', 'front_right', 'rear_left', 'rear_right'], role
print('wheel roles:', role)
for ln, r in role.items():
    print('  %-16s -> %-12s centre in base_link %s' % (ln, r, wheel_info[ln][2].round(4)))

wl = {r: wheel_info[ln][2] for ln, r in role.items()}
wheelbase = wl['front_left'][0] - wl['rear_left'][0]
wheel_sep = wl['front_left'][1] - wl['front_right'][1]
ground_z = min(w[2] for w in wl.values()) - WHEEL_RADIUS
print('wheelbase %.4f  wheel_separation %.4f  floor is %.4f m below base_link origin' % (wheelbase, wheel_sep, -ground_z))

# fdir1: direction of GRIP (roller axis projected on the ground), expressed in base_link.
FDIR = {'front_left': '1 -1 0', 'rear_left': '1 1 0', 'front_right': '1 1 0', 'rear_right': '1 -1 0'}

order = ['front_left', 'front_right', 'rear_left', 'rear_right']
inv_role = {r: ln for ln, r in role.items()}
for r in order:
    ln = inv_role[r]
    M, t, w, tri = wheel_info[ln]
    tri_loc = tri                                    # mesh coords
    tri_link = np.einsum('ij,nkj->nki', M, tri_loc) + (t - w)   # in the new, base-aligned link frame
    com, I, V = mesh_mass_props(tri_link, MASS['wheel'])
    vis_xyz = t - w
    vis_rpy = rpy_from(M)
    assert np.allclose(rot(*vis_rpy), M, atol=1e-6)
    parts.append(f'''  <!-- ============================== WHEEL {r} ({ln}.STL) ============================== -->
  <link name="{r}_wheel">
{inertial_xml(com, MASS['wheel'], I)}
    <visual>
      <origin xyz="{vec(vis_xyz)}" rpy="{vec(vis_rpy)}"/>
      <geometry><mesh filename="{mesh_uri(mesh_file(ln))}"/></geometry>
      <material name="{r}_wheel_mat"><color rgba="0.25 0.25 0.28 1"/></material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="1.570796 0 0"/>
      <geometry><cylinder radius="{f(WHEEL_RADIUS)}" length="{f(WHEEL_WIDTH)}"/></geometry>
    </collision>
  </link>
  <joint name="{r}_wheel_joint" type="continuous">
    <origin xyz="{vec(w)}" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="{r}_wheel"/>
    <axis xyz="0 1 0"/>
    <limit effort="30" velocity="60"/>
    <dynamics damping="0.0" friction="0.0"/>
  </joint>
''')

for cad, name in ROLES_DUE.items():
    M, t, w, tri = child(cad, 'origin', MASS['due'])
    ax = M @ np.array([0, 0, 1.0])
    assert np.allclose(ax, [0, 1, 0], atol=1e-3), f'{cad}: spin axis is {ax}, expected +Y'
    tri_link = np.einsum('ij,nkj->nki', M, tri) + (t - w)
    com, I, V = mesh_mass_props(tri_link, MASS['due'])
    vis_rpy = rpy_from(M)
    assert np.allclose(rot(*vis_rpy), M, atol=1e-6)
    print('due %-14s -> %-10s joint at %s' % (cad, name, w.round(4)))
    parts.append(f'''  <!-- ============================== {name} ({cad}.STL) ============================== -->
  <link name="{name}">
{inertial_xml(com, MASS['due'], I)}
    <visual>
      <origin xyz="0 0 0" rpy="{vec(vis_rpy)}"/>
      <geometry><mesh filename="{mesh_uri(mesh_file(cad))}"/></geometry>
      <material name="{name}_mat"><color rgba="0.75 0.75 0.75 1"/></material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="{vec(vis_rpy)}"/>
      <geometry><mesh filename="{mesh_uri(mesh_file(cad))}"/></geometry>
    </collision>
  </link>
  <joint name="{name}_joint" type="continuous">
    <origin xyz="{vec(w)}" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="{name}"/>
    <axis xyz="0 1 0"/>
    <limit effort="30" velocity="30"/>
    <dynamics damping="0.02" friction="0.0"/>
  </joint>
''')

# ------------------------------------------------------------------ gazebo
g = ['  <!-- ============================== GAZEBO ============================== -->',
     '  <!-- Mecanum rollers are not modelled; instead each wheel has anisotropic friction:',
     '       grip (mu) along fdir1 = roller axis on the ground, free rolling (mu2 = 0) across it.',
     '       fdir1 is expressed in base_link so it does not spin with the wheel. Needs the DART physics engine. -->']
for r in order:
    g.append(f'''  <gazebo reference="{r}_wheel">
    <collision>
      <surface>
        <friction>
          <ode>
            <mu>{f(MU)}</mu>
            <mu2>{f(MU2)}</mu2>
            <fdir1 gz:expressed_in="base_link">{FDIR[r]}</fdir1>
          </ode>
        </friction>
      </surface>
    </collision>
  </gazebo>''')
g.append(f'''  <gazebo>
    <!-- 4-wheel mecanum kinematics: listens to Twist on /cmd_vel (vx, vy, wz), publishes /odom and odom->base_link tf -->
    <plugin filename="gz-sim-mecanum-drive-system" name="gz::sim::systems::MecanumDrive">
      <front_left_joint>front_left_wheel_joint</front_left_joint>
      <front_right_joint>front_right_wheel_joint</front_right_joint>
      <back_left_joint>rear_left_wheel_joint</back_left_joint>
      <back_right_joint>rear_right_wheel_joint</back_right_joint>
      <wheel_separation>{f(wheel_sep)}</wheel_separation>
      <wheelbase>{f(wheelbase)}</wheelbase>
      <wheel_radius>{f(WHEEL_RADIUS)}</wheel_radius>
      <topic>/cmd_vel</topic>
      <odom_topic>/odom</odom_topic>
      <tf_topic>/tf</tf_topic>
      <frame_id>odom</frame_id>
      <child_frame_id>base_link</child_frame_id>
    </plugin>

    <!-- publishes every joint (4 wheels + 2 due units) so robot_state_publisher can draw the tf tree -->
    <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher">
      <topic>/joint_states</topic>
    </plugin>

    <!-- the two due units stay independent velocity-controlled joints (rad/s) -->
    <plugin filename="gz-sim-joint-controller-system" name="gz::sim::systems::JointController">
      <joint_name>front_due_joint</joint_name>
      <topic>/due_wheel/front_due_joint/cmd_vel</topic>
    </plugin>
    <plugin filename="gz-sim-joint-controller-system" name="gz::sim::systems::JointController">
      <joint_name>back_due_joint</joint_name>
      <topic>/due_wheel/back_due_joint/cmd_vel</topic>
    </plugin>
  </gazebo>
</robot>
''')
parts.append('\n'.join(g))

text = '\n'.join(parts).replace('@@DROP@@', f'{-ground_z:.4f}')
open(OUT, 'w').write(text)
print('wrote', OUT)
print('SPAWN_Z (base_link origin height above floor) = %.4f' % (-ground_z))
