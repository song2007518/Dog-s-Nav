import math
import unittest

from rqt_robot_console.core import GridMap, demo_map, demo_plan
from rqt_robot_console.planning import CollisionField, segment_clear


def segment_box_distance(a, b, box):
    """Independent continuous geometry oracle, without a planning-grid mask."""
    xmin, ymin, xmax, ymax = box
    low, high = 0., 1.
    for start, end, left, right in ((a[0],b[0],xmin,xmax),(a[1],b[1],ymin,ymax)):
        delta = end-start
        if abs(delta)<1e-12:
            if not left<=start<=right:
                low,high=1,0
                break
        else:
            first,last=sorted(((left-start)/delta,(right-start)/delta))
            low,high=max(low,first),min(high,last)
    if low<=high:
        return 0.
    def point_box(p):
        return math.hypot(max(xmin-p[0],0,p[0]-xmax),max(ymin-p[1],0,p[1]-ymax))
    def point_segment(p):
        dx,dy=b[0]-a[0],b[1]-a[1]
        t=max(0,min(1,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/(dx*dx+dy*dy))) if dx or dy else 0
        return math.hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dy)
    return min(point_box(a),point_box(b),*(point_segment(p) for p in
               ((xmin,ymin),(xmin,ymax),(xmax,ymin),(xmax,ymax))))


def obstacle_boxes(grid):
    boxes=[]
    # Merge horizontal occupied/unknown runs into rectangles; exact same union.
    for y in range(grid.height):
        x=0
        while x<grid.width:
            if grid.cells[y*grid.width+x]==0:
                x+=1
                continue
            start=x
            while x<grid.width and grid.cells[y*grid.width+x]!=0:
                x+=1
            r=grid.resolution
            boxes.append((start*r,y*r,x*r,(y+1)*r))
    return boxes


class PlanningTests(unittest.TestCase):
    def assert_safe(self,grid,path,radius=.25):
        self.assertTrue(path)
        local=[tuple(v*grid.resolution for v in grid.world_to_cell_float(*p)) for p in path]
        for x,y in local:
            self.assertGreaterEqual(min(x,y,grid.width*grid.resolution-x,grid.height*grid.resolution-y),radius-1e-8)
        boxes=obstacle_boxes(grid)
        for a,b in zip(local,local[1:]):
            for box in boxes:
                self.assertGreaterEqual(segment_box_distance(a,b,box),radius-1e-8,(a,b,box))

    def test_open_space_is_straight_and_keeps_exact_endpoints(self):
        grid=GridMap('open',80,60,.1,(-4,-3,0),[0]*4800)
        a,b=(-2.,-1.),(2.,1.)
        path=demo_plan(grid,a,b)
        self.assertEqual((path[0],path[-1]),(a,b))
        self.assertTrue(all(abs(2*x-4*y)<1e-8 for x,y in path))
        self.assertTrue(all(math.dist(x,y)<=.060001 for x,y in zip(path,path[1:])))
        self.assert_safe(grid,path)

    def test_rounded_detour_keeps_full_footprint_clear(self):
        grid=demo_map()
        path=demo_plan(grid,(-.3,-2.5),(3.6,1.7))
        self.assert_safe(grid,path)
        angles=[math.atan2(b[1]-a[1],b[0]-a[0]) for a,b in zip(path,path[1:])]
        turns=[abs(math.atan2(math.sin(b-a),math.cos(b-a))) for a,b in zip(angles,angles[1:])]
        self.assertLess(max(turns),math.radians(15))

    def test_narrow_passage_falls_back_from_optional_clearance(self):
        width,height=80,50
        cells=[100 if y<17 or y>=24 else 0 for y in range(height) for x in range(width)]
        grid=GridMap('corridor',width,height,.1,(0,0,0),cells)
        path=demo_plan(grid,(1.05,2.05),(6.55,2.05))
        self.assert_safe(grid,path)

    def test_unknown_barrier_is_not_crossed(self):
        grid=GridMap('unknown',60,40,.1,(0,0,0),[0]*2400)
        for y in range(grid.height):grid.cells[y*grid.width+30]=-1
        self.assertEqual(demo_plan(grid,(1,2),(5,2)),[])

    def test_in_place_obstacle_edit_invalidates_inflation(self):
        grid=GridMap('edit',60,40,.1,(0,0,0),[0]*2400)
        self.assertTrue(demo_plan(grid,(1,2),(5,2)))
        for y in range(grid.height):grid.cells[y*grid.width+30]=100
        self.assertEqual(demo_plan(grid,(1,2),(5,2)),[])

    def test_map_boundary_and_invalid_start_are_rejected(self):
        grid=GridMap('edge',40,40,.1,(0,0,0),[0]*1600)
        self.assertEqual(demo_plan(grid,(.1,2),(3,2)),[])
        self.assertEqual(demo_plan(grid,(math.nan,2),(3,2)),[])
        self.assertFalse(segment_clear(grid,(1,2),(4.01,2)))

    def test_segment_checker_includes_corner_and_grid_line_touches(self):
        grid=GridMap('touch',12,12,1,(0,0,0),[0]*144)
        field=CollisionField(grid,0)
        mask=bytearray(field.blocked);mask[5*12+6]=1;field.blocked=bytes(mask)
        self.assertFalse(field.segment_free((4.5,4.5),(7.5,7.5)))
        self.assertFalse(field.segment_free((4.5,6),(7.5,6)))
        self.assertTrue(field.segment_free((4.5,6.01),(7.5,6.01)))

    def test_rotated_map_and_odd_dimensions(self):
        grid=GridMap('rotated',41,33,.1,(10,-8,.6),[0]*(41*33))
        for y in range(10,24):grid.cells[y*grid.width+19]=100
        a,b=grid.cell_to_world(8.5,8.5),grid.cell_to_world(32.5,25.5)
        path=demo_plan(grid,a,b)
        self.assertEqual((path[0],path[-1]),(a,b))
        self.assert_safe(grid,path)
