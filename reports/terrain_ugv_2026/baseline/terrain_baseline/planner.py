from __future__ import annotations
import heapq
import math
import time
import numpy as np
from .model import TerrainGrid, RiskMap

MOVES = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]


def astar(grid: TerrainGrid, risk: RiskMap, start: tuple[int,int], goal: tuple[int,int],
          mode: str = 'terrain') -> tuple[list[tuple[int,int]],dict]:
    """8-connected geometric A* vs terrain-weighted A*. Not Hybrid A*, not RRT*."""
    if mode not in ('geometry','terrain'): raise ValueError(mode)
    t0 = time.perf_counter_ns()
    if not grid.inside(start) or not grid.inside(goal): raise ValueError('start or goal outside grid')
    # Never use unknown true obstacle or true hazard for geometry baseline.
    blocked = risk.forbidden if mode=='terrain' else grid.obstacle
    if blocked[start] or blocked[goal]:
        return [],{'expanded':0,'plan_latency_ms':(time.perf_counter_ns()-t0)/1e6,'reason':'start_or_goal_blocked'}
    dist = {start:0.0}
    came = {}
    sequence=0
    pq=[(math.hypot(start[0]-goal[0],start[1]-goal[1])*grid.resolution,sequence,start)]
    expanded=0
    while pq:
        f,_,p=heapq.heappop(pq)
        heuristic=math.hypot(p[0]-goal[0],p[1]-goal[1])*grid.resolution
        if f>dist[p]+heuristic+1e-9: continue
        expanded+=1
        if p==goal:
            out=[p]
            while p in came:
                p=came[p]
                out.append(p)
            out.reverse()
            return out,{'expanded':expanded,'plan_latency_ms':(time.perf_counter_ns()-t0)/1e6,'reason':'ok'}
        for di,dj in MOVES:
            q=(p[0]+di,p[1]+dj)
            if not grid.inside(q) or blocked[q]: continue
            # prevent passing through obstacle corners
            if di!=0 and dj!=0 and (blocked[p[0]+di,p[1]] or blocked[p[0],p[1]+dj]):
                continue
            length=math.hypot(di,dj)*grid.resolution
            if mode=='terrain':
                # consistent nonnegative edge cost. Slope has rad units (normalized).
                t=0.5*(risk.slope_rad[p]+risk.slope_rad[q])
                rr=0.5*(risk.roughness_m[p]+risk.roughness_m[q])
                slip=0.5*(risk.slip_proxy[p]+risk.slip_proxy[q])
                penalty= 1.0 + 2.0*t/0.30 + 1.5*rr/0.10 + 2.5*slip
            else: penalty=1.0
            cost=length*penalty
            alt=dist[p]+cost
            if alt+1e-12<dist.get(q,float('inf')):
                dist[q]=alt
                came[q]=p
                sequence+=1
                h=math.hypot(q[0]-goal[0],q[1]-goal[1])*grid.resolution
                heapq.heappush(pq,(alt+h,sequence,q))
    return [],{'expanded':expanded,'plan_latency_ms':(time.perf_counter_ns()-t0)/1e6,'reason':'no_path'}
