"""Trace plateau and control state in the public simulator; no training."""
import argparse, csv
from pathlib import Path
import numpy as np
from _mars_rover_cpp import MarsRoverBatchEnv
from mars_rover_env.config import load_env_config
from rover_v5 import Pilot, X,Y,VX,VY,ENERGY,ANGLE,GEAR,RPM,STALLED,TEMP,OVERHEAT,PANEL,CHARGING,PROP_MODE,AIRBORNE,PREV_ACTION,THRUSTER,GAS,BRAKE,UP,IGNITION,PROPELLER,CHARGE

def run(seed, mode, steps, interval, out):
    env=MarsRoverBatchEnv(1,load_env_config())
    obs=np.empty((1,env.obs_dim),np.float32)
    rew=np.zeros(1,np.float32); done=np.zeros(1,np.uint8); trunc=np.zeros(1,np.uint8)
    act=np.zeros(1,np.int32)
    env.reset_all(seed,obs)
    pilot=Pilot(mode)
    best=-1e9; last_progress=0; first_plateau=None; rows=[]
    for t in range(steps):
        before=obs[0].copy()
        a=pilot.step(t,before)
        act[0]=a
        env.step(act,obs,rew,done,trunc)
        o=obs[0]; x=float(o[X])*1000
        if x>best+0.15:
            best=x; last_progress=t; first_plateau=None
        stalled_for=t-last_progress
        if stalled_for>=300 and first_plateau is None: first_plateau=t
        # Every interval and more often around a sustained plateau.
        if t%interval==0 or (stalled_for>=300 and t%60==0) or t==steps-1 or done[0] or trunc[0]:
            info=dict(env.debug_info(0))
            rows.append(dict(seed=seed,mode=mode,step=t+1,time_s=round((t+1)/60,2),
                x_m=round(x,3),best_x_m=round(best,3),vx_m_s=round(float(o[VX])*20,3),
                y_m=round(float(o[Y])*10,3),vy_m_s=round(float(o[VY])*20,3),
                energy=round(float(o[ENERGY]),4),angle_rad=round(float(o[ANGLE]),3),
                gear_norm=round(float(o[GEAR]),4),rpm_norm=round(float(o[RPM]),4),
                stalled=int(o[STALLED]>.5),temp_c_approx=round(float(o[TEMP])*120,2),
                overheated=int(o[OVERHEAT]>.5),airborne=int(o[AIRBORNE]>.5),
                propeller_on=int(o[PROP_MODE]>.5),panel=round(float(o[PANEL]),3),
                charging=int(o[CHARGING]>.5),gas=int(bool(a&GAS)),brake=int(bool(a&BRAKE)),
                up=int(bool(a&UP)),ignition=int(bool(a&IGNITION)),
                thruster=int(bool(a&THRUSTER)),propeller_toggle=int(bool(a&PROPELLER)),
                panel_toggle=int(bool(a&CHARGE)),action_mask=int(a),
                stagnant_steps=stalled_for,terrain_near_0=round(float(o[34]),3),
                terrain_near_1=round(float(o[35]),3),terrain_near_2=round(float(o[36]),3),
                debug_distance_m=info.get('distance_m','')))
        if done[0] or trunc[0]:break
    out.parent.mkdir(parents=True,exist_ok=True)
    with out.open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    print(f'{mode} seed={seed} best_x={best:.2f}m last_progress={last_progress/60:.1f}s '
          f'energy={float(obs[0,ENERGY]):.3f} trace={out}',flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--seed-start',type=int,default=2026)
    p.add_argument('--seeds',type=int,default=3)
    p.add_argument('--steps',type=int,default=18000)
    p.add_argument('--interval',type=int,default=120)
    p.add_argument('--modes',default='drive,fly_propeller')
    p.add_argument('--out-dir',type=Path,default=Path('plateau_traces'))
    a=p.parse_args()
    if a.seeds<1 or a.steps<1 or a.interval<1:p.error('positive values required')
    for seed in range(a.seed_start,a.seed_start+a.seeds):
        for mode in a.modes.split(','):
            mode=mode.strip()
            if not mode:continue
            run(seed,mode,a.steps,a.interval,a.out_dir/f'{mode}_seed{seed}.csv')
