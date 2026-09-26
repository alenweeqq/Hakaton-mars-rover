"""Local, non-training Mars Rover strategy benchmark, compatible with public v0.16.0.
Run from the ORIGINAL project root after installing its native Python extension.
This script does not connect to the competition platform or train a neural network.
"""
from __future__ import annotations
import argparse
import csv
import math
import statistics
from pathlib import Path

import numpy as np
from _mars_rover_cpp import MarsRoverBatchEnv
from mars_rover_env.config import load_env_config

# See cpp/include/mars/action.hpp. These are raw control bits, NOT ACTION_MACROS indices.
GAS=1<<0; BRAKE=1<<1; REVERSE=1<<2; CLUTCH=1<<3
LEFT=1<<4; RIGHT=1<<5; UP=1<<6; DOWN=1<<7; DRIVE_MODE=1<<8
IGNITION=1<<9; CHARGE=1<<10; LIDAR=1<<11; HEATER=1<<12
JUMP=1<<13; CLIMB=1<<14; PROPELLER=1<<15; PISTON=1<<16
JUMP_FRONT=1<<17; JUMP_REAR=1<<18; PISTON_FRONT=1<<19
PISTON_REAR=1<<20; BLOW=1<<21; FLOOD=1<<22; THRUSTER=1<<23

# See cpp/src/observation.cpp (eight possible wheels).
X=0; Y=1; VX=2; VY=3; ANGLE=4; ENERGY=6
TERRAIN=32; SLOPES=56
GEAR=104; RPM=107; STALLED=109; TEMP=111; OVERHEAT=113; COLD=114
PANEL=115; CHARGING=116; LIDAR_ACTIVE=118; LIDAR_COOLDOWN=119
PREV_ACTION=120; IMPACT=125; BELLY=127; CLIMB_MODE=145; PROP_MODE=146
JUMP_COOLDOWN=147; RECOVERY=148; AIRBORNE=151; PISTON_EXT=157

MODES = (
 'drive','careful_drive','fast_drive','eco_drive','drive_lidar',
 'drive_jump','drive_jump_lidar','drive_climb','drive_propeller',
 'fly_burst','fly_propeller','fly_balance','fly_charge','fly_glide',
 'hybrid','hybrid_lidar','hybrid_charge','hybrid_recover',
 'hop_safe','hop_charge','hop_propeller','hop_rescue',
 'v5_hop','v5_hop_charge','v5_rescue','v5_flight',
 'fix_ignition','fix_clutch','fix_downshift','fix_macro_downshift',
)

def toggle(action: int, bit: int, want: bool, is_on: bool, previous: int) -> int:
    # Native code switches these only on the rising edge.
    return action | bit if want != is_on and not (previous & bit) else action

class Pilot:
    def __init__(self, mode):
        self.mode=mode
        self.stagnation=0
        self.last_best=-1e9
        self.last_lidar=-999
        self.recover_until=-1
        self.charge_until=-1
        self.last_jump=-999
        self.hop_phase='DRIVE'
        self.phase_start=0
        self.last_launch=-999
        self.launches=0
        self.charge_events=0
        self.v5_phase='DRIVE'; self.v5_start=0; self.v5_last=-999
        self.v5_rescues=0; self.v5_boost_steps=0
        self.restart_start=-1

    def step(self, t, obs):
        x=float(obs[X])*1000; vx=float(obs[VX])*20; vy=float(obs[VY])*20
        y=float(obs[Y])*10; ang=math.atan2(math.sin(float(obs[ANGLE])),math.cos(float(obs[ANGLE])))
        energy=float(obs[ENERGY]); temp=float(obs[TEMP])*120
        airborne=bool(obs[AIRBORNE]>.5); prop_on=bool(obs[PROP_MODE]>.5)
        panel_on=float(obs[PANEL])>.02 or bool(obs[CHARGING]>.5)
        previous=round(float(obs[PREV_ACTION])*16777215)
        stalled=bool(obs[STALLED]>.5); hot=bool(obs[OVERHEAT]>.5)
        # The first 24 terrain heights are relative to the chassis; zeros may also mean unseen/gaps.
        heights=obs[TERRAIN:TERRAIN+24]
        near=np.asarray(heights[2:10],dtype=float)
        rough=bool(np.any(near>0.7) or (np.max(near)-np.min(near)>1.2))
        # Use precise ground/gap semantics only for available lidar biome samples.
        visible_gap=bool(obs[LIDAR_ACTIVE]>.5 and np.any(obs[81:105:3]<-.5))
        if x>self.last_best+.15:
            self.last_best=x; self.stagnation=0
        else: self.stagnation+=1
        stuck=self.stagnation>60
        flipped=abs(ang)>2.0 or (bool(obs[BELLY]>.5) and abs(ang)>1.2)
        flight='fly' in self.mode or self.mode.startswith('hybrid')
        hybrid=self.mode.startswith('hybrid')
        use_prop=self.mode in ('drive_propeller','fly_propeller','fly_charge','hybrid','hybrid_lidar','hybrid_charge','hybrid_recover')
        use_lidar='lidar' in self.mode
        recovery=self.mode=='hybrid_recover'
        if self.mode.startswith('fix_'):
            return self.fix_step(t,obs,ang,energy,previous,stalled,hot)
        if self.mode.startswith('v5_'):
            return self.v5_step(t,obs,vx,vy,ang,energy,temp,airborne,prop_on,panel_on,previous,stalled,hot,stuck,flipped)
        if self.mode.startswith('hop_'):
            return self.hop_step(t,obs,x,vx,vy,y,ang,energy,temp,airborne,prop_on,panel_on,previous,stalled,hot,stuck,flipped)
        a=0
        want_charge=False
        # A simple policy. This is NOT a trained RL model.
        if recovery and flipped:
            self.recover_until=max(self.recover_until,t+65)
        if recovery and stuck and not airborne:
            self.recover_until=max(self.recover_until,t+35)
        recovering=recovery and t<self.recover_until
        if recovering:
            a|=PISTON_FRONT|PISTON_REAR|JUMP_FRONT|JUMP_REAR
            if t%75<28: a|=GAS|REVERSE
            elif t%75<55: a|=GAS
            a|=LEFT if ang<0 else RIGHT
        else:
            if energy>.065 and not hot and not stalled: a|=GAS
            if stalled and not (previous & IGNITION): a|=IGNITION
            if t==0: a|=IGNITION
            if bool(obs[COLD]>.5) and energy>.08: a|=HEATER
            if self.mode in ('careful_drive','eco_drive') and (rough or vx>5): a=(a&~GAS)|BRAKE
            if self.mode=='fast_drive' and energy>.10: a|=GAS
            if t>0 and (t%90==1 or (float(obs[RPM])>0.80 and vx>1 and t%28==1)):
                if not(previous&UP): a|=UP
            if abs(ang)>.13:
                a|=LEFT if ang<0 else RIGHT
            if self.mode in ('drive_jump','drive_jump_lidar') or (hybrid and rough):
                if rough and not airborne and t-self.last_jump>90 and float(obs[JUMP_COOLDOWN])<.1:
                    a|=JUMP; self.last_jump=t
            if flight and energy>.25 and temp<95 and not panel_on:
                if self.mode in ('fly_burst','fly_propeller','fly_balance','fly_charge','fly_glide'):
                    launch= (45<=t<140)
                    if self.mode in ('fly_balance','fly_glide','fly_propeller','fly_charge'):
                        launch = (45<=t<145) or (airborne and vy<-.9 and y<4 and energy>.38 and temp<85)
                else:
                    launch=(rough or stuck or visible_gap or (airborne and vy<-.9 and y<3.5))
                if launch and abs(ang)<.85: a|=THRUSTER
            if self.mode=='fly_glide' and airborne and y>2: a&=~GAS
            if self.mode in ('fly_charge','hybrid_charge') and airborne and energy<.67 and vy>-.6 and y>2.0:
                self.charge_until=max(self.charge_until,t+42)
            if self.mode in ('fly_charge','hybrid_charge') and t<self.charge_until and airborne and y>1.1:
                want_charge=True
            if self.mode=='eco_drive' and energy<.52 and abs(vx)<.1:
                want_charge=True
        # Do not constantly re-trigger one-shot toggles.
        a=toggle(a,CHARGE,want_charge,panel_on,previous)
        if panel_on or want_charge: a &= ~(GAS|THRUSTER|JUMP|JUMP_FRONT|JUMP_REAR|PROPELLER)
        a=toggle(a,PROPELLER,use_prop and not(panel_on or want_charge) and energy>.21 and not hot,prop_on,previous)
        a=toggle(a,CLIMB,self.mode=='drive_climb' or (hybrid and stuck),bool(obs[CLIMB_MODE]>.5),previous)
        if use_lidar and not bool(obs[LIDAR_ACTIVE]>.5) and float(obs[LIDAR_COOLDOWN])<.05 and energy>.18 and t-self.last_lidar>70 and not(previous&LIDAR):
            a|=LIDAR; self.last_lidar=t
        if hybrid and stuck and not airborne and not panel_on:
            a|=JUMP_FRONT|JUMP_REAR
            # Short, energy-limited escape pulse rather than uncontrolled ascent.
            if energy>.40 and temp<80 and abs(ang)<.55 and self.stagnation%100<12:
                a|=THRUSTER
        if recovery and stuck and energy>.25 and not panel_on: a|=BLOW
        return a


    def fix_step(self,t,obs,ang,energy,previous,stalled,hot):
        """Local restart experiments; original drive remains untouched."""
        a=0
        if stalled:
            if self.restart_start<0: self.restart_start=t
            elapsed=t-self.restart_start
            # Edge-trigger ignition once per 24 frames. Never hold the bit continuously.
            if elapsed%24==0 and not(previous&IGNITION): a|=IGNITION
            if self.mode=='fix_ignition':
                # Test whether simultaneous throttle is necessary to recover.
                if energy>.07 and not hot: a|=GAS
            elif self.mode=='fix_clutch':
                # Test unloaded restart, then throttle after 12 frames.
                if elapsed%24<12: a|=CLUTCH
                elif energy>.07 and not hot: a|=GAS
            elif self.mode=='fix_downshift':
                # Test lower gearing on a stalled engine, avoiding shift spam.
                if elapsed%48==0 and not(previous&DOWN): a|=DOWN
                if energy>.07 and not hot: a|=GAS
            elif self.mode=='fix_macro_downshift':
                # Exact existing macro index 8 (CLUTCH | DOWN = 136) on shift frames.
                # Release DOWN between pulses so the native edge-trigger can fire again.
                # No GAS or other control bits on this frame: preserve exact macro 136.
                if elapsed%48==0 and not(previous&DOWN):
                    return CLUTCH | DOWN
                if energy>.07 and not hot: a|=GAS
        else:
            self.restart_start=-1
            if energy>.065 and not hot: a|=GAS
            if t==0: a|=IGNITION
            if t>0 and t%90==1 and not(previous&UP): a|=UP
        if abs(ang)>.13: a|=LEFT if ang<0 else RIGHT
        if bool(obs[COLD]>.5) and energy>.08: a|=HEATER
        return a

    def hop_step(self,t,obs,x,vx,vy,y,ang,energy,temp,airborne,prop_on,panel_on,previous,stalled,hot,stuck,flipped):
        """Finite-state hop controller. The modes are experimental, not trained."""
        mode=self.mode
        safe=mode=='hop_safe'
        charge=mode=='hop_charge'
        rescue=mode=='hop_rescue'
        # Normalized observation y is world altitude, not ground clearance.
        # Use airborne, vertical speed and elapsed phase time instead.
        if self.hop_phase=='DRIVE':
            if (not safe and t>75 and t-self.last_launch>130 and energy>.48 and temp<75
                and abs(ang)<.45 and (vx>2.0 or (rescue and stuck))):
                self.hop_phase='BOOST'; self.phase_start=t; self.last_launch=t; self.launches+=1
        elif self.hop_phase=='BOOST':
            if t-self.phase_start>=18 or energy<.38 or temp>85 or abs(ang)>.75:
                self.hop_phase='COAST'; self.phase_start=t
        elif self.hop_phase=='COAST':
            if charge and airborne and vy<0 and energy<.84 and t-self.phase_start>=8:
                self.hop_phase='CHARGE'; self.phase_start=t
            elif (not airborne and t-self.phase_start>12) or t-self.phase_start>95:
                self.hop_phase='LAND'; self.phase_start=t
        elif self.hop_phase=='CHARGE':
            # Charge is a toggle; release it before attempting another boost.
            if not airborne or t-self.phase_start>=28 or energy>.90:
                self.hop_phase='LAND'; self.phase_start=t
        elif self.hop_phase=='LAND':
            if not panel_on and (not airborne or t-self.phase_start>65):
                self.hop_phase='DRIVE'; self.phase_start=t
        if safe: self.hop_phase='DRIVE'
        want_charge=self.hop_phase=='CHARGE' and airborne and energy<.90
        a=0
        if stalled and not(previous&IGNITION): a|=IGNITION
        if t==0: a|=IGNITION
        if bool(obs[COLD]>.5) and energy>.15: a|=HEATER
        if not panel_on and energy>.08 and not hot and not stalled and self.hop_phase in ('DRIVE','BOOST','COAST','LAND'):
            a|=GAS
        if t>0 and t%90==1 and not(previous&UP): a|=UP
        if abs(ang)>.14 and not panel_on: a|=LEFT if ang<0 else RIGHT
        if self.hop_phase=='BOOST' and energy>.38 and temp<85 and abs(ang)<.75 and not panel_on:
            a|=THRUSTER
        if rescue and stuck and not airborne and self.hop_phase=='DRIVE' and not panel_on:
            if t%90<12: a|=JUMP_FRONT|JUMP_REAR
        # Charging panel deployment can take time; keep charge requested until deployed.
        a=toggle(a,CHARGE,want_charge,panel_on,previous)
        if panel_on or want_charge: a &= ~(GAS|THRUSTER|JUMP|JUMP_FRONT|JUMP_REAR|PROPELLER)
        want_prop=mode in ('hop_propeller','hop_charge','hop_rescue') and energy>.26 and not hot and not(panel_on or want_charge)
        a=toggle(a,PROPELLER,want_prop,prop_on,previous)
        return a

    def v5_step(self,t,obs,vx,vy,ang,energy,temp,airborne,prop_on,panel_on,previous,stalled,hot,stuck,flipped):
        """Experimental bounded flight controller; validate on local physics before training."""
        mode=self.mode
        allow_charge=mode=='v5_hop_charge'
        emergency=mode in ('v5_rescue','v5_flight')
        # The hard seed cannot build road speed: launch early instead of waiting for vx.
        if self.v5_phase=='DRIVE':
            early=emergency and t>=45 and t<150 and t-self.v5_last>100 and vx<2.0
            blocked=emergency and stuck and t-self.v5_last>105
            rolling=t>90 and vx>2.0 and t-self.v5_last>160
            if energy>.43 and temp<76 and not flipped and abs(ang)<.6 and (early or blocked or rolling):
                self.v5_phase='BOOST'; self.v5_start=t; self.v5_last=t; self.launches+=1
                if early or blocked: self.v5_rescues+=1
        elif self.v5_phase=='BOOST':
            if t-self.v5_start>= (35 if mode=='v5_flight' else 20) or energy<.32 or temp>83 or abs(ang)>.85:
                self.v5_phase='COAST'; self.v5_start=t
        elif self.v5_phase=='COAST':
            # Start panel deployment near the apex, not only after falling.
            if allow_charge and airborne and vy<.5 and energy<.76 and t-self.v5_start>6:
                self.v5_phase='CHARGE'; self.v5_start=t
            elif (not airborne and t-self.v5_start>12) or t-self.v5_start>95:
                self.v5_phase='LAND'; self.v5_start=t
        elif self.v5_phase=='CHARGE':
            if not airborne or t-self.v5_start>38 or energy>.86:
                self.v5_phase='LAND'; self.v5_start=t
        elif self.v5_phase=='LAND':
            if (not panel_on and not airborne) or t-self.v5_start>90:
                self.v5_phase='DRIVE'; self.v5_start=t
        want_panel=self.v5_phase=='CHARGE' and airborne and energy<.86
        a=0
        if t==0 or (stalled and not(previous&IGNITION)): a|=IGNITION
        if obs[COLD]>.5 and energy>.15: a|=HEATER
        if t>0 and t%90==1 and not(previous&UP): a|=UP
        if abs(ang)>.14 and not panel_on: a|=LEFT if ang<0 else RIGHT
        if not panel_on and not want_panel and energy>.09 and not hot and not stalled: a|=GAS
        if self.v5_phase=='BOOST' and energy>.32 and temp<83 and abs(ang)<.85 and not panel_on:
            a|=THRUSTER; self.v5_boost_steps+=1
        if emergency and stuck and not airborne and self.v5_phase=='DRIVE' and not panel_on and t%90<10:
            a|=JUMP_FRONT|JUMP_REAR
        a=toggle(a,CHARGE,want_panel,panel_on,previous)
        if want_panel or panel_on: a&=~(GAS|THRUSTER|JUMP|JUMP_FRONT|JUMP_REAR|PROPELLER)
        want_prop=energy>.23 and not hot and not (want_panel or panel_on)
        a=toggle(a,PROPELLER,want_prop,prop_on,previous)
        return a

def run(seed, mode, steps, snapshot_dir=None):
    cfg=load_env_config()
    env=MarsRoverBatchEnv(1,cfg)
    if env.obs_dim!=160: raise RuntimeError(f'Expected 160 observations, got {env.obs_dim}')
    obs=np.empty((1,env.obs_dim),np.float32)
    rew=np.zeros(1,np.float32); done=np.zeros(1,np.uint8); trunc=np.zeros(1,np.uint8)
    action=np.zeros(1,np.int32)
    env.reset_all(seed,obs)
    pilot=Pilot(mode)
    best=0.; total=0.; min_energy=1.; height=-1e9; flights=0; flips=0; overheats=0; charged=0; last_flip=False; last_hot=False
    for t in range(steps):
        action[0]=pilot.step(t,obs[0]); env.step(action,obs,rew,done,trunc)
        x=float(obs[0,X])*1000
        best=max(best,x-1.0)
        height=max(height,float(obs[0,Y])*10)
        min_energy=min(min_energy,float(obs[0,ENERGY]))
        total+=float(rew[0]); flights+=bool(obs[0,AIRBORNE]>.5)
        charged+=bool(obs[0,CHARGING]>.5)
        flipped=abs(float(obs[0,ANGLE]))>2.2
        hot=bool(obs[0,OVERHEAT]>.5)
        if flipped and not last_flip: flips+=1
        if hot and not last_hot: overheats+=1
        last_flip=flipped; last_hot=hot
        if done[0] or trunc[0]: break
    info=dict(env.debug_info(0))
    return dict(seed=seed, mode=mode, best_distance_m=round(float(info.get('best_distance_m',best)),3),
                final_distance_m=round(float(info.get('distance_m',x-1)),3),
                max_height_m=round(height,3), min_energy_fraction=round(min_energy,4),
                remaining_energy=round(float(obs[0,ENERGY]),4), reward=round(total,3),
                airborne_steps=flights, charging_steps=charged, launch_attempts=pilot.launches, rescue_launches=pilot.v5_rescues, boost_steps=pilot.v5_boost_steps, flips=flips, overheats=overheats,
                steps=t+1, trial_time_left_s=round(float(info.get('trial_time_left',0)),2))

def main():
    p=argparse.ArgumentParser(description='Offline public-simulator strategy comparison; no official quota usage.')
    p.add_argument('--seeds',type=int,default=3,help='Number of identical seeds used for every mode')
    p.add_argument('--steps',type=int,default=3600,help='Simulation steps per run; use 18000 for full trial')
    p.add_argument('--modes',default='all',help='Comma-separated names or all')
    p.add_argument('--seed-start',type=int,default=2026)
    p.add_argument('--out',type=Path,default=Path('rover_results.csv'))
    args=p.parse_args()
    if args.seeds<1 or args.steps<1: p.error('seeds and steps must be positive')
    modes=MODES if args.modes=='all' else tuple(s.strip() for s in args.modes.split(','))
    unknown=set(modes)-set(MODES)
    if unknown: p.error('Unknown modes: '+', '.join(sorted(unknown)))
    rows=[]
    for seed in range(args.seed_start,args.seed_start+args.seeds):
        for mode in modes:
            row=run(seed,mode,args.steps); rows.append(row)
            print(f"seed={seed} {mode:20} best={row['best_distance_m']:8.2f}m final={row['final_distance_m']:8.2f}m energy={row['remaining_energy']:.2f} flips={row['flips']} heat={row['overheats']}",flush=True)
    args.out.parent.mkdir(parents=True,exist_ok=True)
    with args.out.open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
    summary=[]
    for mode in modes:
        r=[row for row in rows if row['mode']==mode]
        summary.append(dict(mode=mode,median_m=round(statistics.median(v['best_distance_m'] for v in r),3),
            mean_m=round(statistics.mean(v['best_distance_m'] for v in r),3),
            min_m=round(min(v['best_distance_m'] for v in r),3),
            max_m=round(max(v['best_distance_m'] for v in r),3),
            mean_remaining_energy=round(statistics.mean(v['remaining_energy'] for v in r),3),
            mean_flips=round(statistics.mean(v['flips'] for v in r),2)))
    summary.sort(key=lambda r:r['median_m'],reverse=True)
    path=args.out.with_name(args.out.stem+'_summary.csv')
    with path.open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,fieldnames=list(summary[0])); w.writeheader(); w.writerows(summary)
    print('\nLOCAL STRATEGY SUMMARY (median best distance, not official score):')
    for row in summary: print(f"{row['mode']:22} median={row['median_m']:8.2f}m min={row['min_m']:8.2f}m max={row['max_m']:8.2f}m")
    print(f'CSV files: {args.out.resolve()} and {path.resolve()}')

if __name__=='__main__': main()
