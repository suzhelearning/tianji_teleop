"""Explicit derived runtime policy; original calibration evidence stays unchanged."""
from dataclasses import replace
from pathlib import Path
import hashlib
import json
import math
import shutil
from pico_calibration_artifact import validate_artifact

POLICY_FILE='pico_geometry_policy.json'
FILES=tuple(f'pico_{side}_{name}.yaml' for side in ('left','right')
            for name in ('arm_geometry','palm_tcp','wrist_pivot'))

def symmetric_lengths(lengths):
    if set(lengths)!={'left','right'}:
        raise ValueError('both validated arm geometries required')
    for upper,forearm in lengths.values():
        if not math.isfinite(upper) or not .15<=upper<=.45:
            raise ValueError('upper arm length outside [0.15,0.45] m')
        if not math.isfinite(forearm) or not .15<=forearm<=.40:
            raise ValueError('forearm length outside [0.15,0.40] m')
    return tuple(max(lengths[s][i] for s in ('left','right')) for i in range(2))

def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def create_profile(source,output):
    import yaml
    source=Path(source).expanduser().resolve();output=Path(output).expanduser().absolute()
    if (source/POLICY_FILE).exists():
        raise ValueError('derive from original calibration, not an already derived profile')
    lengths={}
    hashes={name:digest(source/name) for name in FILES}
    for side in ('left','right'):
        path=source/f'pico_{side}_arm_geometry.yaml'
        validate_artifact(path,'geometry',side,tcp_path=source/f'pico_{side}_palm_tcp.yaml',
                          wrist_path=source/f'pico_{side}_wrist_pivot.yaml')
        doc=yaml.safe_load(path.read_text())
        lengths[side]=(float(doc['upper_arm_length_m']),float(doc['forearm_length_m']))
    common=symmetric_lengths(lengths)
    policy=dict(schema_version=1,complete=True,policy='symmetric_max',source_directory=str(source),
                source_sha256=hashes,original_lengths_m=lengths,
                effective_lengths_m=dict(upper_arm=common[0],forearm=common[1]),
                evidence='runtime override only; not a newly measured calibration')
    output.mkdir(parents=True,exist_ok=False) # Never overwrite any existing profile.
    # Interrupted generation must be rejected, not silently treated as original.
    (output/POLICY_FILE).write_text(json.dumps(dict(complete=False)))
    for name in FILES:
        shutil.copyfile(source/name,output/name)
        if digest(output/name)!=hashes[name]:raise ValueError('source calibration changed during copy')
    (output/POLICY_FILE).write_text(json.dumps(policy,indent=2)+'\n')
    return policy

def apply_profile(geometries,paths):
    if not any(paths.values()):return geometries,None
    directories={s:Path(str(paths[s])).expanduser().resolve().parent for s in ('left','right')}
    present=[(directory/POLICY_FILE).exists() for directory in directories.values()]
    if not any(present):return geometries,None
    if len(set(directories.values()))!=1 or not all(present):
        raise ValueError('symmetric profile must contain both arm geometry files')
    root=directories['left'];policy=json.loads((root/POLICY_FILE).read_text())
    for side in ('left','right'):
        if Path(paths[side]).expanduser().resolve()!=root/f'pico_{side}_arm_geometry.yaml':
            raise ValueError('symmetric profile requires standard geometry filenames')
    if not isinstance(policy,dict):raise ValueError('invalid symmetric profile document')
    if policy.get('schema_version')!=1 or policy.get('complete') is not True or policy.get('policy')!='symmetric_max':
        raise ValueError('invalid or incomplete symmetric profile')
    for name in FILES:
        if digest(root/name)!=policy.get('source_sha256',{}).get(name):
            raise ValueError('symmetric profile calibration fingerprint changed: '+name)
    if any(geometries[s] is None for s in ('left','right')):
        raise ValueError('symmetric profile requires both validated geometries')
    lengths={s:(g.upper_arm_length_m,g.forearm_length_m) for s,g in geometries.items()}
    if policy.get('original_lengths_m')!={s:list(v) for s,v in lengths.items()}:
        raise ValueError('symmetric profile original lengths mismatch')
    upper,forearm=symmetric_lengths(lengths)
    if policy.get('effective_lengths_m')!={'upper_arm':upper,'forearm':forearm}:
        raise ValueError('symmetric profile effective lengths mismatch')
    return {s:replace(g,upper_arm_length_m=upper,forearm_length_m=forearm)
            for s,g in geometries.items()},policy
