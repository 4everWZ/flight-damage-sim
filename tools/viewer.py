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


# Disable every built-in Matplotlib shortcut: q/s/g/arrows/trim keys belong to flight.
def configure_keymaps(plt):
    for key in plt.rcParams:
        if key.startswith('keymap.'):
            plt.rcParams[key]=[]


class Viewer:
    HELD_KEYS={'left','right','up','down','q','e','w','s','pageup','pagedown','i','k','j','l','u','o'}
    ACTION_KEYS={'space','r','m','f2','f3','f4','f5','f6','f7','g'}
    def __init__(self, core, plt):
        self.core=core
        self.plt=plt
        configure_keymaps(plt)
        self.keys=set()
        self.blocked_keys=set()
        self.pressed_actions=set()
        self.pending_releases={}
        self.tk_widget=None
        self.aim=(1,0,0)
        self.prop_rpm_ratio=1
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
        self.fig.canvas.mpl_connect('key_release_event',self.key_up)
        self.fig.canvas.mpl_connect('figure_leave_event',self.focus_lost)
        # Tk FocusOut also catches Alt-Tab while the pointer remains over the canvas.
        if hasattr(self.fig.canvas,'get_tk_widget'):
            self.tk_widget=self.fig.canvas.get_tk_widget()
            self.tk_widget.bind('<FocusOut>',self.focus_lost,add='+')
        self.fig.canvas.mpl_connect('button_press_event',self.set_aim)
        self.fig.canvas.mpl_connect('close_event',lambda _:core.close())
        self.draw()

    def set_aim(self,event):
        if event.inaxes!=self.aim_axis or event.xdata is None or event.ydata is None or getattr(event,'button',1)!=1:
            return
        heading,elevation=map(math.radians,(event.xdata,event.ydata))
        self.aim=(math.cos(elevation)*math.cos(heading),math.cos(elevation)*math.sin(heading),-math.sin(elevation))

    def release_inputs(self):
        # A held key must be released before its auto-repeat can reactivate after reset/pause.
        self.blocked_keys.update(self.keys)
        self.keys.clear()

    def focus_lost(self,_):
        self.release_inputs()
        self.paused=True
        self.accumulator=0
        self.previous=time.monotonic()

    def finish_release(self,key):
        self.pending_releases.pop(key,None)
        self.keys.discard(key)
        self.blocked_keys.discard(key)
        self.pressed_actions.discard(key)

    def key_up(self,event):
        key=(event.key or '').lower()
        native=getattr(event,'guiEvent',None)
        if self.tk_widget is not None and native is not None:
            # X11/Tk may repeat as a release/press pair with the same timestamp.
            # Defer release until idle so the paired press can keep its latch.
            if key in self.pending_releases:
                self.tk_widget.after_cancel(self.pending_releases[key][0])
            callback=self.tk_widget.after_idle(lambda:self.finish_release(key))
            self.pending_releases[key]=(callback,native.time)
        else:
            self.finish_release(key)

    def key_down(self,event):
        key=(event.key or '').lower()
        if key in self.pending_releases:
            callback,stamp=self.pending_releases.pop(key)
            self.tk_widget.after_cancel(callback)
            if getattr(getattr(event,'guiEvent',None),'time',None)!=stamp:
                self.finish_release(key)
        if key in self.HELD_KEYS:
            if key not in self.blocked_keys and not self.paused:
                self.keys.add(key)
            return
        if key not in self.ACTION_KEYS or key in self.pressed_actions:
            return
        self.pressed_actions.add(key)
        if key=='space':
            self.paused=not self.paused
            self.release_inputs()
            self.accumulator=0
            self.previous=time.monotonic()
        elif key=='r':
            self.release_inputs()
            self.core.command('reset');self.aim=(1,0,0);self.prop_rpm_ratio=1
            self.accumulator=0
            self.previous=time.monotonic()
        elif key=='m':
            modes=['flightpath','legacy','off']
            current=self.core.state['controller']
            self.core.command('controller '+modes[(modes.index(current)+1)%3])
        elif key=='f2':
            ids=[c['id'] for c in self.core.state['cells'] if c['node']==2][:8]
            for id_ in ids:self.core.command(f'damage cell {id_} 0 1')
        elif key=='f3':self.core.command('damage link left_aileron')
        elif key=='f4':self.core.command('damage spar left_root_front .3')
        elif key=='f5':self.core.command('damage link elevator')
        elif key=='g' and self.core.state['propeller_model']=='blade_element':self.core.command('governor '+('off' if self.core.state['governor'] else 'on'))
        elif key=='f7' and self.core.state['propeller_model']=='blade_element':self.core.command('damage governor')
        elif key=='f6':self.core.command('damage engine .5')

    def advance(self,ticks):
        direct=(int('right' in self.keys)-int('left' in self.keys),
                int('up' in self.keys)-int('down' in self.keys),
                int('e' in self.keys)-int('q' in self.keys))
        # Opposing pressed keys still own that axis and request zero.
        active=tuple(any(k in self.keys for k in pair) for pair in [('left','right'),('up','down'),('q','e')])
        throttle=max(0,min(1,self.core.state['throttle']+.2*ticks/120*(int('w' in self.keys)-int('s' in self.keys))))
        self.prop_rpm_ratio=max(.5,min(1.1,self.prop_rpm_ratio+.15*ticks/120*(int('pageup' in self.keys)-int('pagedown' in self.keys))))
        trim=list(self.core.state['trim'])
        for axis,(minus,plus) in enumerate([('j','l'),('k','i'),('u','o')]):
            trim[axis]=max(-.9,min(.9,trim[axis]+.15*ticks/120*(int(plus in self.keys)-int(minus in self.keys))))
        return self.core.step(ticks,aim=self.aim,direct=direct,active=active,throttle=throttle,trim=trim,prop_rpm_ratio=self.prop_rpm_ratio)

    def frame(self,_,now=None):
        now=time.monotonic() if now is None else now
        if self.paused:
            self.accumulator=0
        else:
            # Presentation stalls slow wall-time playback rather than creating a catch-up backlog.
            self.accumulator+=min(.1,max(0,now-self.previous))
            ticks=min(12,int((self.accumulator+1e-10)*120))
            if ticks:
                self.advance(ticks)
                self.accumulator-=ticks/120
        self.previous=now
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
        self.aim_axis.scatter([heading],[elevation],s=70,marker='+',color='#b64335',label='aim')
        velocity=[v-w for v,w in zip(s['velocity'],s['wind'])]
        self.actual_path=None
        if sum(v*v for v in velocity)>1e-12:
            actual_heading=math.degrees(math.atan2(velocity[1],velocity[0]))
            actual_elevation=math.degrees(math.atan2(-velocity[2],math.hypot(*velocity[:2])))
            self.actual_path=self.aim_axis.scatter([actual_heading],[actual_elevation],s=32,color='#276788',
                                                 label=f'actual {actual_heading:+.1f}° / {actual_elevation:+.1f}°')
        self.aim_axis.legend(loc='upper left',fontsize=8)
        self.aim_axis.set_title('Aim (+) and actual air path (o) · heading / elevation',fontsize=10,color='white')
        self.structure.clear()
        edges=s['edges']
        self.structure.barh(range(len(edges)),[min(e['ultimate_utilization'],3) for e in edges],color=['#b64335' if e['failed'] else '#426e79' for e in edges])
        self.structure.set_yticks(range(len(edges)),[e['name'] for e in edges],fontsize=7)
        self.structure.axvline(1,color='#b64335',ls='--',lw=1)
        self.structure.set(xlim=(0,3),xlabel='Applied equivalent stress / ultimate stress')
        self.structure.set_title('Independent structural graph · strain drives failure',fontsize=10,color='white')
        for axis in [self.flight,self.aim_axis,self.structure]:
            color='#20354a' if axis is self.flight else 'white'
            axis.tick_params(colors=color)
            axis.xaxis.label.set_color(color)
            axis.yaxis.label.set_color(color)
        self.flight.zaxis.label.set_color('#20354a')
        owners=' / '.join(f'{a}: {o["direct_weight"]:.2f}' for a,o in zip(('roll','pitch','yaw'),s['ownership']))
        surfaces=' '.join(f'{c["name"]}={math.degrees(c["deflection"]):+.1f}°' for c in s['chains'])
        speed=math.sqrt(sum((v-w)**2 for v,w in zip(s['velocity'],s['wind'])))
        self.hud.set_text(f'{"PAUSED · Space resumes" if self.paused else "RUNNING"}  {s["propeller_model"]}/{s["controller"]}  t={s["tick"]/120:6.2f}s  TAS={speed:6.1f}m/s  ALT={-s["position"][2]:7.1f}m  G={s["g"]:5.2f}  RPM={s["rpm"]:5.0f}  throttle={s["throttle"]:.2f}  pieces={s["debris"]}\n'
                          f'Trim (roll/pitch/yaw)={s["trim"][0]:+.3f}/{s["trim"][1]:+.3f}/{s["trim"][2]:+.3f}  Prop pitch={math.degrees(s["prop_pitch"]):.1f}° target={s["target_rpm"]:.0f}rpm limited={s["governor_saturated"]} tipMach={s["tip_mach"]:.2f}   Direct: {owners}\nSurfaces: {surfaces}\n'
                          'Arrows: pitch/roll   Q/E: yaw   W/S: throttle   I/K: pitch trim   J/L: roll trim   U/O: yaw trim\n'
                          'Space: pause/resume   R: reset   M: controller   Loss of canvas focus/pointer pauses and releases input\n'
                          'F2: left skin loss   F3: left aileron cable   F4: spar section 30%   F5: elevator cable   F6: half power   G: governor   F7: pitch jam   PgUp/Dn: RPM request')


def delivery_checks(viewer):
    """Real Matplotlib callbacks → live C++ transport; no mock flight model."""
    from matplotlib.backend_bases import Event, KeyEvent
    import json
    core=viewer.core
    def press(key,native=None):
        KeyEvent('key_press_event',viewer.fig.canvas,key=key,guiEvent=native)._process()
    def release(key,native=None):
        KeyEvent('key_release_event',viewer.fig.canvas,key=key,guiEvent=native)._process()
    def tap(key):
        press(key);release(key)
    tap('r')
    assert all(not v for k,v in viewer.plt.rcParams.items() if k.startswith('keymap.'))
    for key in ['left','right','up','down','q','e']:press(key)
    state=viewer.advance(1)
    assert all(o['direct_weight']==1 and o['command']==0 for o in state['ownership'])
    release('left');state=viewer.advance(1)
    assert state['ownership'][0]['command']==1 and state['ownership'][0]['direct_weight']==1
    for key in ['right','up','down','q','e']:release(key)
    viewer.advance(32)
    assert all(o['direct_weight']==0 for o in core.state['ownership'])
    original_mode=core.state['controller']
    press('m');mode=core.state['controller'];press('m')
    assert mode!=original_mode and core.state['controller']==mode
    release('m');core.command('controller flightpath')
    if core.state['propeller_model']=='blade_element':
        before=core.state['governor'];press('g');press('g')
        assert core.state['governor']!=before
        release('g');tap('g');assert core.state['governor']==before
    # All knobs share the actual core intent, and opposing knobs request no change.
    base=core.state
    for key in ['w','s','i','k','j','l','u','o','pageup','pagedown']:press(key)
    state=viewer.advance(12)
    assert state['throttle']==base['throttle'] and state['trim']==base['trim']
    for key in ['s','k','j','u','pagedown']:release(key)
    state=viewer.advance(60)
    assert state['throttle']>base['throttle'] and all(a>b for a,b in zip(state['trim'],base['trim']))
    if state['propeller_model']=='blade_element':assert state['target_rpm']>base['target_rpm']
    viewer.keys.add('left');viewer.aim=(0,1,0);viewer.accumulator=.003
    tap('r');reset=core.state
    assert not viewer.keys and viewer.accumulator==0 and viewer.aim==(1,0,0) and viewer.prop_rpm_ratio==1
    press('left');assert 'left' not in viewer.keys
    state=viewer.advance(1)
    assert all(o['direct_weight']==0 for o in state['ownership']) and state['trim']==reset['trim'] and state['throttle']==reset['throttle']
    for key in list(viewer.blocked_keys):release(key)
    press('left');viewer.advance(1)
    assert core.state['ownership'][0]['command']==-1
    # Focus loss freezes time and clears input; release/resume returns through core handoff.
    before=core.state['tick']
    Event('figure_leave_event',viewer.fig.canvas)._process()
    assert viewer.paused and not viewer.keys
    viewer.frame(None,viewer.previous+5)
    assert core.state['tick']==before
    release('left');press('space');press('space')
    assert not viewer.paused
    release('space');viewer.previous=0
    viewer.frame(None,viewer.previous+1/120)
    assert core.state['tick']==before+1
    viewer.frame(None,viewer.previous+1/120)
    assert core.state['tick']==before+2 and core.state['ownership'][0]['direct_weight']<1
    viewer.advance(32);assert core.state['ownership'][0]['direct_weight']==0
    # No fractional pause residue or catch-up burst; every physical step remains 1/120 s.
    viewer.accumulator=.003;tap('space');before=core.state['tick']
    viewer.frame(None,viewer.previous+20);assert core.state['tick']==before
    tap('space');viewer.previous=0
    viewer.frame(None,viewer.previous+1/240);assert core.state['tick']==before
    viewer.frame(None,viewer.previous+1/240);assert core.state['tick']==before+1
    before=core.state['tick'];viewer.frame(None,viewer.previous+30)
    assert core.state['tick']==before+12 and abs(viewer.accumulator)<1/120
    viewer.frame(None,viewer.previous);assert core.state['tick']==before+12
    # Exercise native repeat coalescing with a real headless Tcl idle queue, when available.
    tk_repeat=None
    try:
        import tkinter
        from types import SimpleNamespace
        widget=tkinter.Tcl()
    except ImportError:
        widget=None
    if widget is not None:
        viewer.tk_widget=widget
        press('space',SimpleNamespace(time=10));paused=viewer.paused
        release('space',SimpleNamespace(time=11));press('space',SimpleNamespace(time=11))
        widget.eval('update idletasks');assert viewer.paused==paused
        release('space',SimpleNamespace(time=12));widget.eval('update idletasks')
        press('space',SimpleNamespace(time=13));assert viewer.paused!=paused
        release('space',SimpleNamespace(time=14));widget.eval('update idletasks')
        viewer.tk_widget=None;tk_repeat=True
    # Pointer exit must not turn a held/repeating Space into an automatic resume.
    press('space');Event('figure_leave_event',viewer.fig.canvas)._process();press('space')
    assert viewer.paused
    release('space');tap('space');assert not viewer.paused
    tap('r');before=core.state['shaft_power']
    tap('f5');tap('f6');viewer.advance(1)
    assert not next(c for c in core.state['chains'] if c['name']=='elevator')['connected']
    assert core.state['shaft_power']<before*.75
    if core.state['propeller_model']=='blade_element':
        tap('f7');assert core.state['pitch_jammed']
    tap('space');tap('r');assert viewer.paused
    tap('space');assert not viewer.paused
    assert all(c['connected'] for c in core.state['chains']) and all(c['area_fraction']==1 for c in core.state['cells'])
    assert all(e['section']==1 and not e['failed'] for e in core.state['edges']) and not core.state['pitch_jammed']
    report={'backend':'Agg','core':'live C++ 120 Hz','propeller_model':core.state['propeller_model'],'human_gui_verified':False,
            'checks':{'shortcut_conflicts':True,'opposing_keys_per_axis':True,'action_press_repeat':True,
                      'throttle_trim_rpm_inputs':True,'reset_clears_and_blocks_held_input':True,
                      'focus_loss_releases_and_pauses':True,'pause_resume_fixed_ticks_no_backlog':True,
                      'damage_reset_controls':True},'native_tk_repeat_idle_queue':tk_repeat}
    (ROOT/'results'/('viewer-input-summary'+('-legacy' if core.state['propeller_model']=='legacy' else '')+'.json')).write_text(json.dumps(report,indent=2)+'\n')


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
    viewer.keys={'left'}
    previous=core.state['rate_bias'][0]
    s=viewer.advance(24)
    assert s['ownership'][0]['integral_paused'] and s['rate_bias'][0]==previous
    core.command('controller off')
    viewer.keys=set()
    viewer.advance(32)
    assert all(o['command']==0 for o in core.state['ownership'])
    core.command('controller legacy')
    assert core.command('reset')['controller']=='legacy'
    core.command('controller flightpath')
    # A requested prop speed changes a finite pitch actuator, never writes measured rotor RPM.
    if core.state['propeller_model']=='blade_element':
        before=core.state['rpm']
        s=core.step(1,prop_rpm_ratio=.9)
        assert abs(s['target_rpm']-1440)<1e-6 and abs(s['rpm']-before)<5
        core.command('damage governor')
        jam=s['prop_pitch']
        s=core.step(24,prop_rpm_ratio=1.1)
        assert s['pitch_jammed'] and abs(s['prop_pitch']-jam)<1e-12
        core.command('governor off')
        assert core.command('reset')['governor'] is False
        core.command('governor on')
    delivery_checks(viewer)
    viewer.draw()
    # Display-only direction is derived from core air-relative velocity, never attitude.
    v=[a-b for a,b in zip(core.state['velocity'],core.state['wind'])]
    point=viewer.actual_path.get_offsets()[0]
    assert abs(point[0]-math.degrees(math.atan2(v[1],v[0])))<1e-9
    assert abs(point[1]-math.degrees(math.atan2(-v[2],math.hypot(*v[:2]))))<1e-9
    viewer.fig.text(.99,.97,'OFFLINE AGG INPUT TEST · HUMAN GUI NOT VERIFIED',ha='right',color='white',fontsize=9)
    output=ROOT/'results'/('viewer-smoke'+('-legacy' if core.state['propeller_model']=='legacy' else '')+'.png')
    output.parent.mkdir(exist_ok=True)
    viewer.fig.savefig(output,dpi=120,facecolor=viewer.fig.get_facecolor())
    plt.close(viewer.fig)
    print(f'PASS: live intent, key conflicts/repeat, focus/reset/pause, trim/prop/damage, fixed ticks and Agg render → {output}')


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--smoke',action='store_true')
    parser.add_argument('--airframe',default='validation-bem.airframe')
    parser.add_argument('--controller',choices=['flightpath','legacy','off'],default='flightpath')
    args=parser.parse_args()
    cache=ROOT/'build'/'viewer-cache'
    cache.mkdir(parents=True,exist_ok=True)
    os.environ.setdefault('MPLCONFIGDIR',str(cache))
    os.environ.setdefault('XDG_CACHE_HOME',str(cache))
    import matplotlib
    if args.smoke:matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    configure_keymaps(plt)
    with CoreClient(args.airframe) as core:
        core.command('controller '+args.controller)
        if args.smoke:
            smoke(core,plt)
        else:
            if matplotlib.get_backend().lower()!='tkagg':
                raise SystemExit('This delivery uses TkAgg focus handling. Start with MPLBACKEND=TkAgg on a desktop with Tk, or use --smoke in Cloud.')
            from matplotlib.animation import FuncAnimation
            viewer=Viewer(core,plt)
            animation=FuncAnimation(viewer.fig,viewer.frame,interval=33,cache_frame_data=False)
            plt.show()
            _=animation
if __name__=='__main__':main()
