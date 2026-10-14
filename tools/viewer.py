#!/usr/bin/env python3
"""Matplotlib engineering viewer/control adapter for the authoritative C++ core.

Run with a local GUI backend, or use --smoke for an Agg render and protocol test.
This is a debugging viewer, not the final playable game's camera/UI.
"""
import argparse
import os
import math
import time
from pathlib import Path
from live_client import CoreClient, ROOT


def rotate(v, q):
    # Display-only quaternion projection; this never feeds pose or forces to the core.
    w,x,y,z=q
    tx,ty,tz=2*(y*v[2]-z*v[1]),2*(z*v[0]-x*v[2]),2*(x*v[1]-y*v[0])
    return (v[0]+w*tx+y*tz-z*ty,v[1]+w*ty+z*tx-x*tz,v[2]+w*tz+x*ty-y*tx)


class Viewer:
    def __init__(self, core, plt):
        self.core=core
        self.plt=plt
        self.keys=set()
        self.aim=(1,0,0)
        self.paused=False
        self.previous=time.monotonic()
        self.accumulator=0
        self.fig=plt.figure(figsize=(14,8), facecolor='#101a29')
        self.flight=self.fig.add_axes([.02,.23,.48,.70],projection='3d',facecolor='#c2deed')
        self.aim_axis=self.fig.add_axes([.57,.68,.38,.24],facecolor='#e5f0f4')
        self.structure=self.fig.add_axes([.58,.22,.36,.37],facecolor='#e5f0f4')
        self.hud=self.fig.text(.035,.03,'',color='white',fontsize=10,family='monospace')
        self.fig.suptitle('Flight Damage Sim · synthetic 152-cell airframe · C++ 120 Hz',color='white',fontsize=15)
        self.fig.canvas.mpl_connect('key_press_event',self.key_down)
        self.fig.canvas.mpl_connect('key_release_event',lambda e:self.keys.discard(e.key))
        self.fig.canvas.mpl_connect('button_press_event',self.set_aim)
        self.fig.canvas.mpl_connect('close_event',lambda _:core.close())
        self.draw()

    def set_aim(self,event):
        if event.inaxes!=self.aim_axis or event.xdata is None:
            return
        heading,elevation=map(math.radians,(event.xdata,event.ydata))
        self.aim=(math.cos(elevation)*math.cos(heading),math.cos(elevation)*math.sin(heading),-math.sin(elevation))

    def key_down(self,event):
        self.keys.add(event.key)
        if event.key=='space':
            self.paused=not self.paused
        elif event.key=='r':
            self.core.command('reset');self.aim=(1,0,0)
        elif event.key=='f2':
            ids=[c['id'] for c in self.core.state['cells'] if c['node']==2][:8]
            for id_ in ids:self.core.command(f'damage cell {id_} 0 1')
        elif event.key=='f3':self.core.command('damage link left_aileron')
        elif event.key=='f4':self.core.command('damage spar left_root_front .3')
        elif event.key=='f5':self.core.command('damage link elevator')
        elif event.key=='f6':self.core.command('damage engine .5')

    def advance(self,ticks):
        direct=(int('right' in self.keys)-int('left' in self.keys),
                int('up' in self.keys)-int('down' in self.keys),
                int('e' in self.keys)-int('q' in self.keys))
        # Opposing pressed keys still own that axis and request zero.
        active=tuple(any(k in self.keys for k in pair) for pair in [('left','right'),('up','down'),('q','e')])
        throttle=max(0,min(1,self.core.state['throttle']+.2*ticks/120*(int('w' in self.keys)-int('s' in self.keys))))
        return self.core.step(ticks,aim=self.aim,direct=direct,active=active,throttle=throttle)

    def frame(self,_):
        now=time.monotonic()
        if not self.paused:self.accumulator+=now-self.previous
        self.previous=now
        ticks=min(120,int(self.accumulator*120))
        if ticks:
            self.advance(ticks)
            self.accumulator-=ticks/120
        self.draw()

    def draw(self):
        s=self.core.state
        self.flight.clear()
        self.flight.set_facecolor('#c2deed')
        cg=s['cg']
        visible=[c for c in s['cells'] if c['attached']]
        positions=[rotate([c['position'][i]-cg[i] for i in range(3)],s['quaternion']) for c in visible]
        colors=['#ed6e5d' if c['area_fraction']<1 else '#d79b36' if c['stall'] else '#276788' for c in visible]
        if positions:
            self.flight.scatter([p[0] for p in positions],[p[1] for p in positions],[-p[2] for p in positions],s=14,c=colors,depthshade=False)
        sample=list(zip(visible,positions))[::8]
        if sample:
            forces=[rotate(c['force'],s['quaternion']) for c,_ in sample]
            scale=1.5/max(1,max(math.sqrt(sum(v*v for v in f)) for f in forces))
            self.flight.quiver([p[0] for _,p in sample],[p[1] for _,p in sample],[-p[2] for _,p in sample],
                               [f[0]*scale for f in forces],[f[1]*scale for f in forces],[-f[2]*scale for f in forces],color='#925571',linewidth=.7)
        nose=rotate([2-cg[0],-cg[1],-cg[2]],s['quaternion'])
        tail=rotate([-4.8-cg[0],-cg[1],-cg[2]],s['quaternion'])
        self.flight.plot([nose[0],tail[0]],[nose[1],tail[1]],[-nose[2],-tail[2]],color='#15283e',lw=3)
        self.flight.set(xlim=(-6,6),ylim=(-6,6),zlim=(-6,6),xlabel='North relative (m)',ylabel='East relative (m)',zlabel='Up relative (m)')
        self.flight.view_init(elev=22,azim=155)
        self.flight.set_title('Attitude + cell forces · red = missing skin · amber = local stall',fontsize=9,color='white')
        self.aim_axis.clear()
        heading=math.degrees(math.atan2(self.aim[1],self.aim[0]))
        elevation=math.degrees(math.atan2(-self.aim[2],math.hypot(*self.aim[:2])))
        self.aim_axis.set(xlim=(-180,180),ylim=(-30,45),xlabel='Aim heading (deg)',ylabel='Elevation (deg)')
        self.aim_axis.grid(alpha=.3)
        self.aim_axis.scatter([heading],[elevation],s=70,marker='+',color='#b64335')
        self.aim_axis.set_title('Click to aim · real surfaces drive all motion',fontsize=10,color='white')
        self.structure.clear()
        edges=s['edges']
        self.structure.barh(range(len(edges)),[min(e['ultimate_utilization'],3) for e in edges],color=['#b64335' if e['failed'] else '#426e79' for e in edges])
        self.structure.set_yticks(range(len(edges)),[e['name'] for e in edges],fontsize=7)
        self.structure.axvline(1,color='#b64335',ls='--',lw=1)
        self.structure.set(xlim=(0,3),xlabel='Applied equivalent stress / ultimate stress')
        self.structure.set_title('Independent structural graph · strain drives failure',fontsize=10,color='white')
        for axis in [self.flight,self.aim_axis,self.structure]:
            axis.tick_params(colors='white')
            axis.xaxis.label.set_color('white')
            axis.yaxis.label.set_color('white')
        self.flight.zaxis.label.set_color('white')
        owners=' / '.join(f'{a}: {o["direct_weight"]:.2f}' for a,o in zip(('roll','pitch','yaw'),s['ownership']))
        surfaces=' '.join(f'{c["name"]}={math.degrees(c["deflection"]):+.1f}°' for c in s['chains'])
        speed=math.sqrt(sum((v-w)**2 for v,w in zip(s['velocity'],s['wind'])))
        self.hud.set_text(f't={s["tick"]/120:6.2f}s  TAS={speed:6.1f}m/s  ALT={-s["position"][2]:7.1f}m  G={s["g"]:5.2f}  RPM={s["rpm"]:5.0f}  throttle={s["throttle"]:.2f}  pieces={s["debris"]}\n'
                          f'Direct ownership weights: {owners}\nSurfaces: {surfaces}\n'
                          'Arrows: pitch/roll   Q/E: yaw   W/S: throttle   Space: pause   R: reset\n'
                          'F2: left skin loss   F3: left aileron cable   F4: spar section 30%   F5: elevator cable   F6: half engine power')


def smoke(core,plt):
    viewer=Viewer(core,plt)
    start=core.state['tick']
    viewer.keys={'left'}
    s=viewer.advance(120)
    assert s['tick']==start+120 and s['ownership'][0]['command']==-1
    assert all(o['direct_weight']==0 for o in s['ownership'][1:])
    viewer.keys=set()
    s=viewer.advance(32)
    assert s['ownership'][0]['direct_weight']==0
    viewer.key_down(type('Event',(),{'key':'f2'})())
    s=viewer.advance(1)
    assert sum(c['area_fraction']==0 for c in s['cells'])==8
    assert not any(e['failed'] for e in s['edges'])
    viewer.key_down(type('Event',(),{'key':'f3'})())
    s=viewer.advance(120)
    assert s['chains'][0]['connected'] is False and abs(s['chains'][0]['deflection'])<1e-5
    viewer.key_down(type('Event',(),{'key':'f4'})())
    assert core.state['edges'][0]['section']==.3
    viewer.advance(1)
    viewer.set_aim(type('AimEvent',(),{'inaxes':viewer.aim_axis,'xdata':90,'ydata':0})())
    assert abs(viewer.aim[0])<1e-10 and viewer.aim[1]==1
    before=core.state['tick']
    try:
        core.command('step invalid')
        raise AssertionError('invalid command must be rejected')
    except ValueError:
        pass
    assert core.step(0)['tick']==before
    viewer.draw()
    output=ROOT/'results'/'viewer-smoke.png'
    output.parent.mkdir(exist_ok=True)
    viewer.fig.savefig(output,dpi=120,facecolor=viewer.fig.get_facecolor())
    plt.close(viewer.fig)
    print(f'PASS: live intent, per-axis handoff, cell/link/spar injection, telemetry and Agg rendering → {output}')


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--smoke',action='store_true')
    args=parser.parse_args()
    cache=ROOT/'build'/'viewer-cache'
    cache.mkdir(parents=True,exist_ok=True)
    os.environ.setdefault('MPLCONFIGDIR',str(cache))
    os.environ.setdefault('XDG_CACHE_HOME',str(cache))
    import matplotlib
    if args.smoke:matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    for key in ('quit','fullscreen','home','back','forward','pan','zoom','save'):
        plt.rcParams['keymap.'+key]=[]
    with CoreClient() as core:
        if args.smoke:
            smoke(core,plt)
        else:
            if 'agg' == matplotlib.get_backend().lower():
                raise SystemExit('No interactive display backend. Run locally with Tk/Qt, or use --smoke in Cloud.')
            from matplotlib.animation import FuncAnimation
            viewer=Viewer(core,plt)
            animation=FuncAnimation(viewer.fig,viewer.frame,interval=33,cache_frame_data=False)
            plt.show()
            _=animation
if __name__=='__main__':main()
