"""Select nearby goals from connected, known free occupancy cells; no commands."""
from collections import deque
import math
import numpy as np

NAVIGATION_FIELDS = [('clearance','Obstacle clearance · metres',1.),
                     ('min_distance','Minimum goal distance · metres',.75),
                     ('max_distance','Maximum goal distance · metres',2.),
                     ('tolerance','Arrival tolerance · metres',.3),
                     ('timeout','Navigation timeout · seconds',120.),
                     ('halt_timeout','Stop deadline · seconds',10.)]


def choose_map_goals(grid, current, recipe):
    resolution = grid.resolution
    if not math.isfinite(resolution) or resolution <= 0:
        raise ValueError('Map resolution is invalid')
    if current.frame != grid.origin.frame:
        raise ValueError('Robot pose and occupancy map need the same frame')
    if grid.data.shape != (grid.height, grid.width):
        raise ValueError('Map dimensions disagree with its data')
    clearance = recipe.get('clearance',1.)
    minimum = recipe.get('min_distance',.75)
    maximum = recipe.get('max_distance',2.)
    if not all(math.isfinite(v) and v > 0 for v in (clearance,minimum,maximum)) or minimum >= maximum:
        raise ValueError('Navigation clearance and goal distances are invalid')
    yaw = grid.origin.yaw
    c,s = math.cos(yaw), math.sin(yaw)
    dx,dy = current.x-grid.origin.x,current.y-grid.origin.y
    col = math.floor((c*dx+s*dy)/resolution)
    row = math.floor((-s*dx+c*dy)/resolution)
    if not (0 <= row < grid.height and 0 <= col < grid.width):
        raise ValueError('Robot pose is outside the occupancy map')
    radius = math.ceil(clearance/resolution)
    if grid.width <= 2*radius or grid.height <= 2*radius:
        raise ValueError('Mapped area is smaller than the requested clearance region')
    extent = math.ceil(maximum/resolution)+radius+1
    r0,r1 = max(0,row-extent),min(grid.height,row+extent+1)
    c0,c1 = max(0,col-extent),min(grid.width,col+extent+1)
    # Unknown, occupied and partially occupied cells are all excluded.
    # A square clearance region is conservative relative to a circular footprint.
    if (r1-r0+2*radius)*(c1-c0+2*radius) > 2000000:
        raise ValueError('Map resolution exceeds the bounded automatic goal search; supply reviewed goals')
    free = grid.data[r0:r1,c0:c1] == 0
    blocked = np.pad(~free,radius,constant_values=True)
    integral = np.pad(blocked.astype(np.int64),((1,0),(1,0))).cumsum(0).cumsum(1)
    size = 2*radius+1
    counts = integral[size:,size:] - integral[:-size,size:] - integral[size:,:-size] + integral[:-size,:-size]
    allowed = counts == 0
    start = (row-r0,col-c0)
    if not allowed[start]:
        raise ValueError(f'Current robot pose lacks {clearance:g} m known-free clearance; adjust the clearance or choose reviewed goals')
    queue,seen = deque([start]),{start}
    candidates=[]
    def world(cell):
        y,x = cell
        local_x,local_y = (x+c0+.5)*resolution,(y+r0+.5)*resolution
        return grid.origin.x+c*local_x-s*local_y, grid.origin.y+s*local_x+c*local_y
    while queue:
        cell=queue.popleft();x,y=world(cell);distance=math.hypot(x-current.x,y-current.y)
        if distance >= minimum:
            candidates.append((distance,cell,x,y))
        for dr,dc in ((0,1),(1,0),(0,-1),(-1,0)):
            neighbor=(cell[0]+dr,cell[1]+dc)
            if neighbor in seen or not (0<=neighbor[0]<allowed.shape[0] and 0<=neighbor[1]<allowed.shape[1]) or not allowed[neighbor]:
                continue
            nx,ny=world(neighbor)
            if math.hypot(nx-current.x,ny-current.y)>maximum:
                continue
            seen.add(neighbor);queue.append(neighbor)
    candidates.sort(reverse=True)
    chosen=[]
    for _,cell,x,y in candidates:
        if all(math.hypot(x-g['x'],y-g['y'])>=minimum for g in chosen):
            chosen.append({'x':x,'y':y,'z':current.z,'yaw':current.yaw,'frame':current.frame})
        if len(chosen)==2:
            break
    if len(chosen)<2:
        raise ValueError('No two nearby goals with connected known-free clearance; map more space or supply reviewed goals')
    return chosen


def live_map_goals(robot, recipe):
    grid=robot.get_map()
    current=robot.get_pose()
    if current.frame != grid.origin.frame:
        current=robot.tf.pose_of('@base',in_frame=grid.origin.frame)
    return choose_map_goals(grid,current,recipe)
