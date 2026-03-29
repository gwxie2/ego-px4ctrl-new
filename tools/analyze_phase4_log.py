import re, json, sys, argparse
from pathlib import Path

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('logfile', type=str)
    parser.add_argument('--out', type=str, default='metrics.json')
    args = parser.parse_args()

    s = Path(args.logfile).read_text(errors='ignore')
    metrics = {
        'obstacle_events': len(re.findall(r'Goal blocked and no nearby free voxel found|ERROR! the drone is in obstacle|terminal point of the current trajectory is in obstacle|Suddenly discovered obstacles\. emergency stop!', s, flags=re.IGNORECASE)),
        'replan_events': len(re.findall(r'to REPLAN_TRAJ', s)),
        'wait_to_gen': len(re.findall(r'from WAIT_TARGET to GEN_NEW_TRAJ', s)),
        'takeoff_published': len(re.findall(r'takeoff_land_trigger published takeoff', s)),
    }
    metrics['pass_obstacle_lt20'] = metrics['obstacle_events'] < 20
    metrics['pass_replan_lt100'] = metrics['replan_events'] < 100
    
    Path(args.out).write_text(json.dumps(metrics, indent=2, ensure_ascii=False))
    print(json.dumps(metrics, indent=2, ensure_ascii=False))

if __name__ == '__main__':
    main()
