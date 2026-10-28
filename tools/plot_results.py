#!/usr/bin/env python3
"""Plot actual CSV output from the C++ simulator. No fabricated jitter data."""
import argparse
import csv
from pathlib import Path
import math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def load_csv(path):
    with open(path, newline='', encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError(f"Empty CSV: {path}")
    return rows


def col(rows, name, missing=math.nan):
    return [float(r[name]) if r.get(name, '') else missing for r in rows]


def event_times(path):
    if not path.exists():
        return []
    with open(path, newline='', encoding='utf-8') as f:
        return [(float(r['time_s']), r['event']) for r in csv.DictReader(f)]


def decorate(title, xlabel, ylabel):
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.grid(True, alpha=.25)
    plt.tight_layout()


def plot_tracking(rows, events, out, scenario):
    ts = col(rows, 't_s')
    stop_events = [t for t, what in events if what.startswith('SAFE_STOP:')]
    # After SAFE_STOP the nominal planned reference is canceled. The C++ CSV
    # retains it for debugging; plot only a short window after the trigger to
    # avoid presenting inactive-plan mismatch as control-loop tracking error.
    end = min(ts[-1], stop_events[0]+.15) if stop_events else ts[-1]
    count = sum(t<=end for t in ts)
    fig = plt.figure(figsize=(11.5, 5.8))
    for i in range(1, 21):
        ys = col(rows, f'error_{i}')
        plt.plot(ts[:count], ys[:count], linewidth=1.1, label=f'J{i:02d}')
    for t in stop_events:
        plt.axvline(t, linestyle='--', linewidth=1.5)
        plt.axvspan(t, end, alpha=.06)
        plt.text(t + .008, plt.ylim()[1]*.75, 'SAFE_STOP', rotation=90, va='center', fontsize=8)
    if stop_events:
        plt.xlim(max(0,stop_events[0]-.8),end)
    decorate(f'Nominal-reference mismatch — {scenario}', 'Simulation time (s)', 'planned q - actual q (rad)')
    fig.legend(loc='outside right center', ncol=2, fontsize=8, frameon=False)
    fig.subplots_adjust(right=.80)
    fig.savefig(out, dpi=160, bbox_inches='tight')
    plt.close(fig)



def plot_braking(rows, events, out, scenario):
    triggers=[t for t,ev in events if ev.startswith('SAFE_STOP:')]
    if not triggers:
        return
    trigger=triggers[0]
    t=col(rows,'t_s')
    speeds=col(rows,'max_abs_vel_rad_s')
    fig=plt.figure(figsize=(11.5,4.5))
    plt.plot(t,speeds,label='Maximum |joint velocity|')
    plt.axvline(trigger,linestyle='--',label='SAFE_STOP triggered')
    plt.axhline(.006,linestyle=':',label='STOPPED speed threshold')
    plt.xlim(max(0,trigger-.5),min(t[-1],trigger+.6))
    plt.legend()
    decorate(f'Group braking response — {scenario}', 'Simulation time (s)', 'Max absolute velocity (rad/s)')
    plt.savefig(out,dpi=160,bbox_inches='tight')
    plt.close(fig)

def plot_arrivals(rows, out):
    ts = col(rows, 't_s')
    pos_tol, vel_tol, window = .006, .014, 40
    target = [(.20 + .023*i) * (1 if i%2==0 else -1) for i in range(20)]
    arrivals = []
    for i in range(20):
        q = col(rows, f'q_{i+1}')
        v = col(rows, f'v_{i+1}')
        streak = 0
        arrival = None
        for k, t in enumerate(ts):
            if t < 2.95:
                continue
            if abs(q[k]-target[i]) < pos_tol and abs(v[k]) < vel_tol and rows[k]['state']=='RUNNING':
                streak += 1
                if streak == window:
                    arrival = ts[k-window+1]
                    break
            else:
                streak = 0
        arrivals.append(arrival)
    if any(x is None for x in arrivals):
        raise ValueError('At least one nominal joint has not arrived. Cannot plot sync spread.')
    spread = (max(arrivals)-min(arrivals))*1000
    fig = plt.figure(figsize=(11.3, 5.0))
    plt.bar(range(1,21), [(a-3.25)*1000 for a in arrivals])
    plt.axhline(0, linestyle='--', linewidth=1)
    plt.xticks(range(1,21))
    decorate(f'Actual arrival vs planned finish | spread = {spread:.1f} ms',
             'Joint index', 'Actual arrival time - 3.25 s (ms)')
    plt.savefig(out,dpi=160,bbox_inches='tight')
    plt.close(fig)
    return spread


def plot_jitter(rows, out, name, ylabel, title):
    values=col(rows,name)
    ts=col(rows,'t_s')
    samples=[(t,v) for t,v in zip(ts,values) if math.isfinite(v)]
    if not samples:
        return False
    t2,v2=zip(*samples)
    fig=plt.figure(figsize=(11.5,4.5))
    plt.plot(t2,v2,linewidth=.7)
    plt.axhline(0,linewidth=.8,linestyle='--')
    decorate(title,'Simulation time (s)',ylabel)
    plt.savefig(out,dpi=160,bbox_inches='tight')
    plt.close(fig)
    return True



def plot_jam_torque(rows, events, out):
    t=col(rows,'t_s')
    torque=col(rows,'torque_8')
    trigger=[x for x,e in events if e.startswith('SAFE_STOP:')]
    fig=plt.figure(figsize=(11.5,4.5))
    plt.plot(t,torque,label='Joint 8 actuator torque (simulation)')
    if trigger:
        plt.axvline(trigger[0],linestyle='--',label='SAFE_STOP')
    plt.axvline(1.45,linestyle=':',label='Jam injected')
    plt.xlim(1.2,2.3)
    plt.legend()
    decorate('Torque buildup during joint 8 mechanical jam', 'Simulation time (s)', 'Motor torque (N m)')
    plt.savefig(out,dpi=160,bbox_inches='tight')
    plt.close(fig)

def plot_watchdog(rows, events, out):
    t=col(rows,'t_s')
    ages=col(rows,'controller_hb_age_ms')
    fig=plt.figure(figsize=(11.5,4.5))
    plt.plot(t,ages,label='Controller heartbeat age')
    plt.axhline(50.0,linestyle='--',linewidth=1.2,label='Timeout threshold 50 ms')
    for time, ev in events:
        if ev.startswith('SAFE_STOP:'):
            plt.axvline(time,linestyle=':',label=f'Detected at {time:.3f} s')
    plt.legend()
    decorate('Heartbeat monitor & watchdog response','Simulation time (s)','Heartbeat age (ms)')
    plt.savefig(out,dpi=160,bbox_inches='tight')
    plt.close(fig)


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--input',default='results',type=Path)
    ap.add_argument('--output',default='plots',type=Path)
    args=ap.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    files=['nominal','jam','controller_heartbeat','drive_heartbeat']
    loaded={k:load_csv(args.input/f'{k}.csv') for k in files if (args.input/f'{k}.csv').exists()}
    for k,rows in loaded.items():
        ev=event_times(args.input/f'{k}_events.csv')
        plot_tracking(rows,ev,args.output/f'tracking_{k}.png',k)
        plot_braking(rows,ev,args.output/f'braking_{k}.png',k)
    if 'nominal' in loaded:
        nominal=loaded['nominal']
        spread=plot_arrivals(nominal,args.output/'arrival_times_nominal.png')
        print(f'Nominal arrival spread = {spread:.1f} ms (tolerance-based, not 0 by definition)')
        for name,fn,ylabel,title in [
            ('wakeup_lateness_us','wakeup_lateness_nominal.png','Wakeup lateness (us)','Observed wakeup lateness (OS-dependent)'),
            ('period_error_us','period_error_nominal.png','Actual period - 1000 us','Measured cycle-period deviation')
        ]:
            if not plot_jitter(nominal,args.output/fn,name,ylabel,title):
                print(f'No measured {name}: run nominal with --realtime')
    if 'jam' in loaded:
        plot_jam_torque(loaded['jam'],event_times(args.input/'jam_events.csv'),args.output/'jam_motor_torque.png')
    if 'controller_heartbeat' in loaded:
        plot_watchdog(loaded['controller_heartbeat'],
                      event_times(args.input/'controller_heartbeat_events.csv'),
                      args.output/'heartbeat_watchdog.png')
    print('Saved plots to:',args.output.resolve())


if __name__=='__main__':
    main()
