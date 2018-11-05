# Blender 2.8 modification: splines as hair example
# Requires to compile https://github.com/nantille/blender_28_mod

import numpy as np
import bpy
import random
import math

def create_spline(curve_data, s_type='NURBS', len_nodes=100, spline_id=0, splines_count=1, bud_position=None):
    """
    Create a spline of given type with n nodes to form a path made of sin and cos
    """
    spline = curve_data.splines.new(type=s_type)
    
    # Regular spline points need xyz + weight
    got_points = 1
    co_dimension = 4
    pts = spline.points
    if s_type == 'BEZIER':
        got_points = 2
        # Bezier control points accept only xyz
        co_dimension = 3
        # Left and right handles are not handled here
        pts = spline.bezier_points
    
    # This is the len for numpy arrays
    len_nodes = len_nodes - got_points + 1

    # Every spline already has got point(s) when created
    # This was compensated with got_points
    pts.add(len_nodes - 1)

    if bud_position is None:
        bud_position = np.random.rand(co_dimension) * 1000

    # Below is a play with random, sin and cos just for demo.
    # Replace with your own data and if you have none, it's pretty easy
    # to generate a bunch of points in space with Sverchok or Animation Nodes
    
    radii = np.random.rand(len_nodes) + 1
    radii *= radii**4 / 10
    
    dir_walk = np.arange(len_nodes) / 10 + np.random.rand(len_nodes)
    pi_walk = (np.arange(len_nodes)+1) * int(math.pi / len_nodes * 100)/(100+len_nodes)
    pi_walk += random.random()*math.pi
    
    nodes = np.random.rand(len_nodes, co_dimension)
    nodes[:, 0] += bud_position[0]
    nodes[:, 1] += bud_position[1]
    nodes[:, 2] += bud_position[2]
    
    rf1 = int(random.random()*3 + 1)
    rf2 = int(random.random()*3 + 1)
    nodes[:, 0] += np.sin(np.cos(pi_walk)) * random.random()*300+200
    nodes[:, 1] += (np.cos(np.sin(pi_walk)**rf1) + np.sin(pi_walk*rf2)) * random.random()*300+200
    nodes[:, 2] += np.sin(pi_walk*rf2) * np.cos(pi_walk*rf1) * random.random()*300+200
    nodes [:, 0] += np.random.rand(len_nodes) * (random.random()*20+20)
    nodes [:, 1] += np.random.rand(len_nodes) * (random.random()*20+20)
    nodes [:, 2] += np.random.rand(len_nodes) * (random.random()*20+20)
    
    #nodes[:, 0] += np.sin(pi_walk*random.random())*(random.random()*10+10)**2
    #nodes[:, 1] += np.sin(pi_walk*random.random())*(random.random()*100+100)
    #nodes[:, 2] += np.cos(pi_walk*random.random())*(random.random()*100+100)
    nodes [:, :] *= (random.random()*2+0.5)
    
    # Dummy data for key and value properties, play with HairInfo.Key and HairInfo.Value in your shader!
    keys = np.arange(len_nodes) + np.random.rand(len_nodes)
    values = np.random.rand(len_nodes)

    pts.foreach_set('co', nodes.ravel())
    pts.foreach_set('radius', radii.ravel())
    pts.foreach_set('key', keys.ravel())
    pts.foreach_set('value', values.ravel())
    
    if s_type == 'BEZIER':
        handle_fac = 100
        lefts = nodes.copy()
        lefts[:, 0] += np.random.rand(len_nodes)* handle_fac - handle_fac/2
        lefts[:, 1] += np.random.rand(len_nodes)* handle_fac - handle_fac/2
        lefts[:, 2] += np.random.rand(len_nodes)* handle_fac - handle_fac/2
        rights = nodes.copy()
        rights[:, 0] += np.random.rand(len_nodes)* handle_fac - handle_fac/2
        rights[:, 1] += np.random.rand(len_nodes)* handle_fac - handle_fac/2
        rights[:, 2] += np.random.rand(len_nodes)* handle_fac - handle_fac/2
        pts.foreach_set('handle_left', lefts.ravel())
        pts.foreach_set('handle_right', rights.ravel())

    spline.use_endpoint_u = True
    # Spline resolution defaults to 12 but is too much for this use-case
    spline.resolution_u = 3
    return spline

def create_random_splines_within_curve(num_splines=100, num_nodes_per_spline=100, mesh_data=None, name='Curve > hair'):
    """
    Create n splines within a single curve object
    """
    curve_data = bpy.data.curves.new(name=name, type='CURVE')
    # Tell Cycles to render all splines within this curve as hair 
    curve_data.cycles_curves.render_as_hair = True
    curve_data.dimensions = '3D'
    curve_data.resolution_u = 3
    curve = bpy.data.objects.new(name=name, object_data=curve_data)
    
    bud_position = None

    # Alternate creation of bezier vs nurbs splines
    bezier_or_not = np.random.rand(num_splines)
    for spline_id in range(num_splines):
        len_nodes = int(num_nodes_per_spline + random.random() * 10)
        s_type = 'NURBS' if bezier_or_not[spline_id] < 0.5 else 'BEZIER'
        if mesh_data is not None:
            random_vert_id = int(random.random()*len(mesh_data.vertices))
            bud_position = mesh_data.vertices[random_vert_id].co
        spline = create_spline(curve.data, s_type, len_nodes, spline_id, num_splines, bud_position)

    # Used to be bpy.data.scenes['Scene'].objects.link(curve)
    # Collection 1 or any existing collection on scene
    # bpy.data.collections['Collection 1'].objects.link(curve)
    bpy.context.scene.collection.objects.link(curve)
    return curve

# Main call to draw all the splines in a single curve object
# If you give an object, like a Suzanne, then the splines will
# be distributed using mesh vertices in a loose manner
ref_ob = bpy.data.objects.get('Suzanne')
num_splines = 10000
num_nodes_per_spline = 50
mesh_data = None
if ref_ob is not None:
    mesh_data = ref_ob.data
curve = create_random_splines_within_curve(num_splines, num_nodes_per_spline, mesh_data)