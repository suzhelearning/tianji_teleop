"""Prepare an offline Ceres trial using an explicitly supplied source checkout.

Does not start a viewer, socket or device. The current mapping and frozen model
are retained; this is an external-backend experiment, not a runtime migration.
"""
import argparse
import copy
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import yaml
import mujoco

CONTROL = Path(__file__).resolve().parents[1]


def mapping_rows(text):
    rows, segments, total = [], [], 0
    origin, previous = None, None
    for line in text.splitlines():
        if not line.startswith('mapping_frame '):
            continue
        fields = line.split()
        stamp, sequence, valid = map(int, fields[1:4])
        total += 1
        if previous is not None and stamp <= previous:
            raise ValueError('mapping timestamps must strictly increase')
        origin = stamp if origin is None else origin
        previous = stamp
        if valid not in (0, 1):
            raise ValueError('invalid mapping validity')
        if not valid:
            if len(fields) != 4:
                raise ValueError('invalid empty mapping frame')
            if rows:
                segments.append(rows)
                rows = []
            continue
        values = np.asarray(fields[4:], dtype=float)
        if values.size != 48 or not np.isfinite(values).all():
            raise ValueError('invalid mapping payload')
        row = [(stamp-origin)*1e-9]
        for arm in values.reshape(2, 24):
            rotation = arm[15:].reshape(3, 3)
            if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-8, rtol=0) or abs(np.linalg.det(rotation)-1)>1e-8:
                raise ValueError('mapping rotation is not SO(3)')
            row.extend(arm[12:15].tolist())
            row.extend(Rotation.from_matrix(rotation).as_quat().tolist())
        rows.append(row)
    if rows:
        segments.append(rows)
    rows = max(segments, key=len, default=[])
    if len(rows) < 2:
        raise ValueError('insufficient valid mapping frames')
    # EE replay CSV cannot represent invalidity. Select ONE contiguous valid
    # segment; never interpolate over invalid source frames or join segments.
    return rows, total-len(rows)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def arm_model(source, destination):
    tree=ET.parse(source)
    root=tree.getroot()
    compiler=root.find('compiler')
    compiler.set('meshdir',str((source.parent/compiler.get('meshdir','')).resolve()))
    names=[f'Joint{j}_{s}' for s in ('L','R') for j in range(1,8)]
    removed=[]
    for parent in root.iter():
        for node in list(parent):
            if node.tag=='joint' and node.get('name') and node.get('name') not in names:
                removed.append(node.get('name'))
                parent.remove(node)
    if len(removed)!=40:
        raise ValueError('unexpected hand joint layout')
    with destination.open('x') as f:
        f.write(ET.tostring(root,encoding='unicode'))
    original=mujoco.MjModel.from_xml_path(str(source))
    derived=mujoco.MjModel.from_xml_path(str(destination))
    if derived.nq!=14 or derived.nv!=14:
        raise ValueError('arm-only model dimension mismatch')
    datas=[mujoco.MjData(m) for m in (original,derived)]
    ids=[[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,n) for n in names] for m in (original,derived)]
    if min(ids[0]+ids[1])<0 or not np.array_equal(original.jnt_range[ids[0]],derived.jnt_range[ids[1]]):
        raise ValueError('arm limits changed')
    rng=np.random.default_rng(20260918)
    worst=0.0
    for _ in range(32):
        ranges=original.jnt_range[ids[0]]
        q=rng.uniform(ranges[:,0],ranges[:,1])
        for m,d,js in zip((original,derived),datas,ids):
            d.qpos[m.jnt_qposadr[js]]=q
            mujoco.mj_forward(m,d)
        for side in ('L','R'):
            sites=[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_SITE,'tcp_'+side) for m in (original,derived)]
            worst=max(worst,float(np.max(np.abs(datas[0].site_xpos[sites[0]]-datas[1].site_xpos[sites[1]]))),float(np.max(np.abs(datas[0].site_xmat[sites[0]]-datas[1].site_xmat[sites[1]]))))
    if worst>1e-12:
        raise ValueError('derived model FK changed')
    return dict(fixed_hand_joints=len(removed),fk_probe_count=32,max_tcp_component_difference=worst)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-root', type=Path, required=True)
    parser.add_argument('--trace', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--profile', type=Path, default=CONTROL/'config/qp_ik_pico_shared_root_reachable.yaml')
    args = parser.parse_args()
    source, trace, profile, output = [p.resolve() for p in (args.source_root,args.trace,args.profile,args.output)]
    source_profile=source/'config/qp_ik_pico_ee_franka_ceres_lm_ruckig_mujoco.yaml'
    base=yaml.safe_load(profile.read_text())
    geometry_path=(profile.parent/base['spark_shared_root']['robot_geometry_artifact']).resolve()
    geometry=yaml.safe_load(geometry_path.read_text())['robot_geometry']
    model=(geometry_path.parent/geometry['mujoco_xml_path']).resolve()
    if sha(model)!=geometry['mujoco_xml_sha256']:
        raise ValueError('frozen model hash mismatch')
    native=subprocess.run([str(CONTROL/'build/tianji_shared_root_trace_audit'),str(profile),str(trace),'--mapping-frames'],capture_output=True,text=True,check=True)
    if 'ik_executed=false' not in native.stdout or 'trace_sha256='+sha(trace) not in native.stdout:
        raise ValueError('missing native provenance')
    rows, excluded=mapping_rows(native.stdout)
    cfg=yaml.safe_load(source_profile.read_text())
    if cfg['ik']['algorithm']!='pico_ee_franka_ceres_lm' or cfg['pico_ee_franka_ceres_lm']['post_smoothing']['mode']!='ruckig':
        raise ValueError('unexpected source algorithm or smoother')
    output.mkdir(parents=True,exist_ok=False)
    derived_model=output/'shared_root_arms_only.xml'
    model_check=arm_model(model,derived_model)
    targets=output/'mapped_targets.csv'
    with targets.open('x') as f:
        writer=csv.writer(f,lineterminator='\n')
        writer.writerow(['timestamp_s']+[f'{s}_{k}' for s in ('left','right') for k in ('px','py','pz','qx','qy','qz','qw')])
        writer.writerows(rows)
    # Explicitly use the source's supported MuJoCo FK fallback: its default
    # Pinocchio TCP differs from the current frozen shared-root palm TCP.
    cfg['controller']['pico_ee_dls_kinematics_urdf_path']=''
    for key in ('initial_posture_enabled','initial_left_q_rad','initial_right_q_rad'):
        cfg['controller'][key]=copy.deepcopy(base['controller'][key])
    cfg['sequential_task_hqp'].update(replay_enabled=True,replay_csv_path=str(targets),replay_loop=False,
        replay_q_nominal_left=cfg['controller']['initial_left_q_rad'],
        replay_q_nominal_right=cfg['controller']['initial_right_q_rad'])
    for name in ('source_limits','matched_jerk'):
        variant=copy.deepcopy(cfg)
        if name=='matched_jerk':
            variant['pico_ee_franka_ceres_lm']['post_smoothing']['max_jerk_rad_s3']=copy.deepcopy(base['joint_limits']['max_jerk_rad_s3'])
        with (output/(name+'.yaml')).open('x') as f:
            yaml.safe_dump(variant,f,sort_keys=False)
    manifest=dict(source_root=str(source),source_profile=str(source_profile),source_profile_sha256=sha(source_profile),
        trace=str(trace),trace_sha256=sha(trace),profile=str(profile),profile_sha256=sha(profile),model=str(model),model_sha256=sha(model),
        targets_sha256=sha(targets),valid_frames=len(rows),excluded_frames=excluded,first_valid_recording_s=rows[0][0],last_valid_recording_s=rows[-1][0],
        derived_model=str(derived_model),derived_model_sha256=sha(derived_model),model_check=model_check,
        backend='MuJoCo FK fallback',scope='filtered projected mapping before guidance blend/hold; offline external Ceres + Ruckig',
        source_commit=subprocess.check_output(['git','-C',str(source),'rev-parse','HEAD'],text=True).strip(),
        source_files_sha256={p:sha(source/p) for p in ('src/controller_ceres_lm.cpp','src/pico_ee_franka_ceres_lm.cpp','config/qp_ik_pico_ee_franka_ceres_lm_ruckig_mujoco.yaml')})
    with (output/'manifest.json').open('x') as f:
        json.dump(manifest,f,indent=2)
    print(json.dumps(manifest,indent=2))


if __name__=='__main__':
    main()
