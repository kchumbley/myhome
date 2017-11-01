#!/usr/bin/python

import bpy

infile_loc = '/home/kevin/Documents/my_models/my_model/obj/map.obj'
textures_loc = '/home/kevin/Documents/OSM2World/build/textures/'
outfile_loc = '/home/kevin/Documents/my_models/my_model/dae/my_model.dae'

imported_object = bpy.ops.import_scene.obj(filepath=infile_loc)
obj_object = bpy.context.selected_objects[0] ####<--Fix
print('Imported name: ', obj_object.name)
bpy.context.scene.objects.active = obj_object
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.join()
bpy.ops.file.find_missing_files(directory=textures_loc)
bpy.ops.wm.collada_export(filepath=outfile_loc)



