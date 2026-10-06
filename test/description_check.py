#!/usr/bin/env python3
import json
import math
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET
import numpy as np
import yaml

pkg=Path(__file__).resolve().parents[1]
out=Path(sys.argv[1]); out.mkdir(parents=True,exist_ok=True)
xml=subprocess.check_output(['xacro',str(pkg/'urdf/golf_cart.urdf.xacro'),
                             'config_file:='+str(pkg/'config/vehicle.yaml')],text=True)
(out/'golf_cart.urdf').write_text(xml)
root=ET.fromstring(xml); p=yaml.safe_load((pkg/'config/vehicle.yaml').read_text())
links={e.attrib['name']:e for e in root.findall('link')}
joints={e.attrib['name']:e for e in root.findall('joint')}
transforms={'base_link':np.eye(4)}


def origin(e):
    t=np.eye(4)
    if e is None: return t
    t[:3,3]=[float(x) for x in e.get('xyz','0 0 0').split()]
    r,p,y=[float(x) for x in e.get('rpy','0 0 0').split()]
    rx=np.array([[1,0,0],[0,math.cos(r),-math.sin(r)],[0,math.sin(r),math.cos(r)]])
    ry=np.array([[math.cos(p),0,math.sin(p)],[0,1,0],[-math.sin(p),0,math.cos(p)]])
    rz=np.array([[math.cos(y),-math.sin(y),0],[math.sin(y),math.cos(y),0],[0,0,1]])
    t[:3,:3]=rz@ry@rx
    return t


pending=list(joints.values())
while pending:
    progress=False
    for j in pending[:]:
        parent=j.find('parent').get('link'); child=j.find('child').get('link')
        if parent not in transforms: continue
        assert child not in transforms, 'Cycle or duplicate parent'
        transforms[child]=transforms[parent]@origin(j.find('origin'))
        pending.remove(j); progress=True
    assert progress,'Disconnected description'
assert set(transforms)==set(links)
positions={k:transforms[k][:3,3] for k in links if k.endswith('_wheel')}
assert np.allclose(positions['front_left_wheel'],[p['wheelbase'],p['front_track']/2,0])
assert np.allclose(positions['front_right_wheel'],[p['wheelbase'],-p['front_track']/2,0])
assert np.allclose(positions['rear_left_wheel'],[0,p['rear_track']/2,0])
assert np.allclose(positions['rear_right_wheel'],[0,-p['rear_track']/2,0])
assert sum(j.get('type')=='continuous' for j in joints.values())==4
assert sum(j.get('type')=='revolute' for j in joints.values())==2
mass=0; bounds=[]
for name,link in links.items():
    inertial=link.find('inertial'); mass+=float(inertial.find('mass').get('value'))
    i=inertial.find('inertia'); diag=[float(i.get(k)) for k in ('ixx','iyy','izz')]
    assert min(diag)>0 and 2*max(diag)<=sum(diag)+1e-9
    for c in link.findall('collision'):
        t=transforms[name]@origin(c.find('origin')); g=c.find('geometry')
        if g.find('box') is not None:
            extent=np.array([float(x) for x in g.find('box').get('size').split()])/2
            span=np.abs(t[:3,:3])@extent
        elif g.find('cylinder') is not None:
            cyl=g.find('cylinder'); radius=float(cyl.get('radius')); half=float(cyl.get('length'))/2
            axis=t[:3,2]; span=radius*np.sqrt(np.maximum(0,1-axis*axis))+half*np.abs(axis)
        else: raise AssertionError('Unsupported collision')
        bounds.extend([t[:3,3]-span,t[:3,3]+span])
extent=np.max(bounds,axis=0)-np.min(bounds,axis=0)
assert np.allclose(extent,[p['overall_length'],max(p['front_track'],p['rear_track'])+p['wheel_width'],p['overall_height']])
report={'passed':True,'wheel_centers_m':{k:v.tolist() for k,v in positions.items()},
        'collision_envelope_m':dict(zip(('length','width','height'),extent.tolist())),
        'estimated_total_mass_kg':mass,'links':len(links),'moving_joints':6,
        'wheelbase_m':p['wheelbase'],'front_track_m':p['front_track'],
        'rear_track_m':p['rear_track'],'rear_track_status':'measured from supplied GLB wheel instances',
        'wheel_diameter_m':2*p['wheel_radius'],'wheel_width_m':p['wheel_width']}
(out/'description_check.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
