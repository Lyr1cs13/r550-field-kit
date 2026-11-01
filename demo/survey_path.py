"""Geometry shared by the continuous survey and its ROS bridge."""
import math

def checked_path(grid, values, endpoint, clearance=.23):
    if not isinstance(values,list) or not 2<=len(values)<=5000:raise ValueError('连续采集路线格式无效')
    result=[]
    for p in values:
        if not isinstance(p,(list,tuple)) or len(p)!=2:raise ValueError('路线坐标格式无效')
        x,y=map(float,p)
        if not all(math.isfinite(v) for v in (x,y)):raise ValueError('路线坐标无效')
        grid.validate(x,y,clearance)
        if result:
            distance=math.dist(result[-1],(x,y))
            if distance>.12:raise ValueError('连续采集路线有跳点')
        result.append([x,y])
    if math.dist(result[-1],endpoint)>.025:raise ValueError('路线终点与引导点不一致')
    return result

def advance_index(path, index, pose, window=1.):
    """Search forward by arc length, never jump onto the adjacent return lane."""
    end=index;distance=0.
    while end+1<len(path) and distance<window:
        distance+=math.dist(path[end],path[end+1]);end+=1
    return min(range(index,end+1),key=lambda i:math.dist(path[i],pose[:2]))

def ahead(path,index,length):
    distance=0.
    while index+1<len(path) and distance<length:
        distance+=math.dist(path[index],path[index+1]);index+=1
    return index
