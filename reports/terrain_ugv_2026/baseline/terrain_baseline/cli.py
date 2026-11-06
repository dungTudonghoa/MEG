from __future__ import annotations
import argparse
from pathlib import Path
import yaml
from .experiments import run


def main():
    p=argparse.ArgumentParser(description='Measured synthetic wheeled UGV planning baseline')
    p.add_argument('--config',default='configs/benchmark.yaml')
    p.add_argument('--output',default='results')
    p.add_argument('--no-plots',action='store_true')
    args=p.parse_args()
    with open(args.config,encoding='utf-8') as f:config=yaml.safe_load(f)
    raw,summary=run(config,Path(args.output),make_plots=not args.no_plots)
    print(f'Executed {len(raw)} paired synthetic trials; outputs at {args.output}')
    for r in summary:
        print(f"{r['scenario']:8} {r['planner']:8} success={r['success_rate']:.2f} unsafe_plan={r['unsafe_plan_rate']:.2f} median_plan_ms={r['median_planning_ms']:.2f}")

if __name__=='__main__':main()
