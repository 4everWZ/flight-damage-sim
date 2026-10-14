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
    print(output)

if __name__=='__main__':main()
