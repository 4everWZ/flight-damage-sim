#!/usr/bin/env python3
"""Plot measured CSV output; no flight model in Python."""
import os
from experiments import ROOT, RESULTS
import csv
os.environ.setdefault('MPLCONFIGDIR',str(ROOT/'build'/'viewer-cache'))
os.environ.setdefault('XDG_CACHE_HOME',str(ROOT/'build'/'viewer-cache'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def load(name):
    return [{k:float(v) for k,v in row.items()} for row in csv.DictReader((RESULTS/(name+'.csv')).open())]


def main():
    fig,axes=plt.subplots(2,2,figsize=(12,8),layout='constrained')
    ax=axes[0,0]
    for name,label in [('roll','clean linkage'),('control_damage','left cable disconnected')]:
        rows=[r for r in load(name) if r['time']<=3]
        ax.plot([r['time'] for r in rows],[r['p']*180/3.141592653589793 for r in rows],label=label)
    ax.set(title='Same direct roll request; actual authority differs',xlabel='Time (s)',ylabel='Roll rate (deg/s)')
    ax.axvspan(1,2,color='grey',alpha=.1)
    ax.legend(fontsize=8)
    ax=axes[0,1]
    rows=[r for r in load('mixed') if .7<=r['time']<=2.7]
    ax.plot([r['time'] for r in rows],[r['roll_cmd'] for r in rows],label='roll command')
    ax.plot([r['time'] for r in rows],[r['roll_owner'] for r in rows],label='direct ownership weight')
    ax.plot([r['time'] for r in rows],[r['pitch_owner'] for r in rows],label='pitch direct weight',ls=':')
    ax.axvline(2,color='grey',ls='--')
    ax.set(title='Per-axis override and 0.25 s handoff',xlabel='Time (s)',ylabel='Normalized command / weight')
    ax.legend(fontsize=8)
    ax=axes[1,0]
    for name,label in [('highspeed_pull','intact'),('highspeed_weakened','front spar section 30%')]:
        rows=[r for r in load(name) if r['time']<=6]
        ax.plot([r['time'] for r in rows],[r['g'] for r in rows],label=label)
        if name=='highspeed_weakened':
            failure=next(r['time'] for r in rows if r['failed']>0)
            separation=next(r['time'] for r in rows if r['debris']>0)
            ax.axvline(failure,color='#c65746',ls='--',label=f'failure {failure:.3f}s')
            ax.axvline(separation,color='#813325',ls=':',label=f'separation {separation:.3f}s')
    ax.set(title='Coupled high-speed entry + pull; low-load phase survives',xlabel='Time (s)',ylabel='Body-axis normal load (G)')
    ax.legend(fontsize=8)
    ax=axes[1,1]
    rows=load('stall')
    ax.plot([r['time'] for r in rows],[r['stalled'] for r in rows],color='#b2742d')
    ax.set(title='40 m/s entry: local stall and subsequent recovery',xlabel='Time (s)',ylabel='Cells beyond table stall angle')
    for ax in axes.flat:ax.grid(alpha=.2)
    fig.suptitle('Measured synthetic fixture behavior · 152 cells · authoritative C++ core · 120 Hz',fontsize=13)
    output=RESULTS/'experiments.png'
    fig.savefig(output,dpi=150)
    if (RESULTS/'instructor-summary.json').exists():
        comparison,plots=plt.subplots(2,2,figsize=(12,8),layout='constrained')
        for mode,label in [('legacy','legacy guidance'),('flightpath-p','new P-only'),('flightpath','new PI, 40° limit')]:
            rows=load('mouse_'+mode)
            time=[r['time'] for r in rows]
            plots[0,0].plot(time,[r['altitude']-1000 for r in rows],label=label)
            plots[0,1].plot(time,[r['air_heading']*180/3.141592653589793 for r in rows],label=label)
        plots[0,0].set(title='Complete trajectory: height loss and recovery',xlabel='Time (s)',ylabel='Altitude change (m)')
        plots[0,1].set(title='Actual velocity heading; 90° target',xlabel='Time (s)',ylabel='Heading (deg)')
        for name,label in [('mouse_flightpath','clean'),('mouse_damaged_flightpath','25% left outer skin loss'),('mouse_disconnected_flightpath','elevator disconnected')]:
            rows=load(name)
            if name=='mouse_disconnected_flightpath':rows=[r for r in rows if r['altitude']>0]
            plots[1,0].plot([r['time'] for r in rows],[r['altitude']-1000 for r in rows],label=label)
        plots[1,0].axhline(-1000,color='grey',ls=':')
        plots[1,0].text(.02,.03,'Cut trace stops at reference ground; collision pending',transform=plots[1,0].transAxes,fontsize=8)
        plots[1,0].set(title='Damage retains real physical consequences',xlabel='Time (s)',ylabel='Altitude change (m)')
        for bank in [40,50,60]:
            rows=load('mouse_flightpath' if bank==40 else 'mouse_bank'+str(bank))
            plots[1,1].plot([r['time'] for r in rows],[r['pitch_cmd'] for r in rows],label=f'{bank}° bank limit')
        plots[1,1].set(title='Demand and normalized pitch saturation',xlabel='Time (s)',ylabel='Pitch command')
        for ax in plots.flat:ax.grid(alpha=.2);ax.legend(fontsize=8)
        comparison.suptitle('Same corrected dynamics · no pose/torque corrections · 120 Hz synthetic fixture')
        comparison.savefig(RESULTS/'instructor-comparison.png',dpi=150)
    if (RESULTS/'governor.csv').exists():
        rows=list(csv.DictReader((RESULTS/'governor.csv').open()))
        rotor,plots=plt.subplots(2,2,figsize=(12,8),layout='constrained')
        for col,cases in enumerate([['fixed','governed','jammed'],['windmill','limited']]):
            for case in cases:
                trace=[r for r in rows if r['case']==case and float(r['hz'])==120]
                time=[float(r['time']) for r in trace]
                plots[0,col].plot(time,[float(r['rpm']) for r in trace],label=case)
                plots[1,col].plot(time,[float(r['pitch'])*180/3.141592653589793 for r in trace],label=case)
            plots[0,col].axhline(1600,color='grey',ls=':',label='target 1600 RPM')
            plots[0,col].set(xlabel='Time (s)',ylabel='Relative shaft RPM')
            plots[1,col].set(xlabel='Time (s)',ylabel='Collective pitch (deg)')
        plots[0,0].set_title('70 → 110 m/s at 15 s; throttle .75 → .95 at 25 s')
        plots[0,1].set_title('Windmill: stopped/off at 80 m/s; limited: damaged travel')
        for ax in plots.flat:ax.grid(alpha=.2);ax.legend(fontsize=8)
        rotor.suptitle('Synthetic signed blade model · finite pitch governor · overspeed is numerical evidence, not transonic fidelity')
        rotor.savefig(RESULTS/'rotor-governor.png',dpi=150)
    if (RESULTS/'detached-flight.csv').exists():
        rows=load('detached-flight')
        fig,axes=plt.subplots(1,2,figsize=(12,4),layout='constrained')
        t=[r['time'] for r in rows]
        axes[0].plot(t,[-r['z'] for r in rows],label='own cell aerodynamics')
        axes[0].plot(t,[-r['ballistic_z'] for r in rows],ls='--',label='ballistic counterfactual')
        axes[0].set(xlabel='Simulation time (s)',ylabel='Altitude (m)',title='Same separation state; actual forces change continuation')
        for key in ['p','q','r']:
            axes[1].plot(t,[r[key]*180/3.141592653589793 for r in rows],label=key)
        axes[1].set(xlabel='Simulation time (s)',ylabel='Body rate (deg/s)',title='Detached wing integrates its own moment and inertia')
        for ax in axes:ax.legend(fontsize=8);ax.grid(alpha=.2)
        fig.suptitle('C++ 120 Hz · load-driven graph failure · synthetic airframe · no collision / secondary fracture')
        fig.savefig(RESULTS/'detached-flight.png',dpi=150)
    print(output)

if __name__=='__main__':main()
