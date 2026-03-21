#!/usr/bin/env python3
"""
KPI analysis script for COMP11132 mission telemetry.
Usage: python3 analyse_kpis.py ~/mission_log_YYYYMMDD_HHMMSS.csv
"""
import csv
import sys
import math
from collections import defaultdict

# ── Declared KPI targets (define BEFORE looking at data) ─────────────────
KPI_TARGETS = {
    'stationkeep_duration_tolerance_s': 0.5,  # configured ± this
    'configured_stationkeep_s':         3.0,  # must match mission_executor param
    'configured_altitude_m':           10.0,  # must match altitude param
    'max_altitude_deviation_m':         0.5,  # target: < 0.5m
    'max_takeoff_time_s':              15.0,  # target: < 15s
    'max_position_error_m':             0.5,  # acceptance_radius
}


def load_csv(path):
    rows = []
    with open(path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append({
                'time':  float(row['ros_time']),
                'state': row['state'],
                'x':     float(row['x']),
                'y':     float(row['y']),
                'z':     float(row['z']),
            })
    return rows


def find_first_transition(rows, from_state, to_state):
    """Return timestamp when state changes from from_state to to_state."""
    prev = None
    for row in rows:
        if prev == from_state and row['state'] == to_state:
            return row['time']
        prev = row['state']
    return None


def find_all_transitions(rows, to_state):
    """Return list of timestamps for every transition INTO to_state."""
    times = []
    prev = None
    for row in rows:
        if prev != to_state and row['state'] == to_state:
            times.append(row['time'])
        prev = row['state']
    return times


def compute_kpis(rows):
    results = {}

    # ── KPI-1: Takeoff time ───────────────────────────────────────────────
    armed_t = find_first_transition(rows, 'IDLE',    'ARMED')
    nav_t   = find_first_transition(rows, 'TAKEOFF', 'NAVIGATING')
    if armed_t and nav_t:
        results['takeoff_time_s'] = nav_t - armed_t
    else:
        results['takeoff_time_s'] = None

    # ── KPI-2: Altitude accuracy ──────────────────────────────────────────
    target_z     = KPI_TARGETS['configured_altitude_m']
    flying_rows  = [r for r in rows if r['state'] in
                    ('NAVIGATING', 'STATIONKEEPING', 'RTL')]
    if flying_rows:
        deviations = [abs(r['z'] - target_z) for r in flying_rows]
        results['max_altitude_deviation_m']  = max(deviations)
        results['mean_altitude_deviation_m'] = sum(deviations) / len(deviations)
    else:
        results['max_altitude_deviation_m'] = None

    # ── KPI-3: Stationkeep duration per waypoint ──────────────────────────
    sk_starts    = find_all_transitions(rows, 'STATIONKEEPING')
    sk_ends_all  = find_all_transitions(rows, 'NAVIGATING') + \
                   find_all_transitions(rows, 'RTL')
    sk_ends_all.sort()

    durations = []
    for start in sk_starts:
        later = [t for t in sk_ends_all if t > start]
        if later:
            durations.append(later[0] - start)

    results['stationkeep_durations_s'] = durations
    if durations:
        results['stationkeep_mean_s']  = sum(durations) / len(durations)
        results['stationkeep_max_err'] = max(
            abs(d - KPI_TARGETS['configured_stationkeep_s']) for d in durations
        )

    # ── KPI-4: Total mission time ─────────────────────────────────────────
    armed_t2   = find_first_transition(rows, 'IDLE', 'ARMED')
    complete_t = find_first_transition(rows, 'RTL',  'COMPLETE')
    if armed_t2 and complete_t:
        results['total_mission_time_s'] = complete_t - armed_t2

    # ── KPI-5: Number of waypoints visited ────────────────────────────────
    results['waypoints_visited'] = len(sk_starts)

    return results


def print_report(results):
    t = KPI_TARGETS
    print('\n' + '=' * 60)
    print(' KPI EVALUATION REPORT')
    print('=' * 60)

    def row(name, value, target, unit=''):
        if value is None:
            status  = '⚠  NO DATA'
            val_str = 'N/A'
        else:
            val_str = f'{value:.2f}{unit}'
            status  = '✅ PASS' if value <= target else '❌ FAIL'
        print(f'  {name:<35} {val_str:<12} target:<{target}{unit}  {status}')

    row('Takeoff time',
        results.get('takeoff_time_s'),
        t['max_takeoff_time_s'], 's')

    row('Max altitude deviation',
        results.get('max_altitude_deviation_m'),
        t['max_altitude_deviation_m'], 'm')

    row('Max stationkeep error',
        results.get('stationkeep_max_err'),
        t['stationkeep_duration_tolerance_s'], 's')

    wps = results.get('waypoints_visited', 0)
    print(f'  {"Waypoints visited":<35} {wps}')

    total = results.get('total_mission_time_s')
    if total:
        print(f'  {"Total mission time":<35} {total:.1f}s')

    sks = results.get('stationkeep_durations_s', [])
    if sks:
        print(f'\n  Stationkeep durations (s): {[round(d, 2) for d in sks]}')
        print(f'  Mean: {sum(sks)/len(sks):.2f}s  '
              f'Configured: {t["configured_stationkeep_s"]}s')

    print('=' * 60 + '\n')


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage: python3 analyse_kpis.py <path_to_csv>')
        sys.exit(1)

    rows = load_csv(sys.argv[1])
    print(f'Loaded {len(rows)} rows from {sys.argv[1]}')
    results = compute_kpis(rows)
    print_report(results)