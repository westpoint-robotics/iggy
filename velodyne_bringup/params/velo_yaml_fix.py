#!/usr/bin/env python

import yaml

with open("64E_Factory_Mod.yaml", 'r') as ymlfile:
    cfg = yaml.load_all(ymlfile)

    with open("64E_Factory_Mod2.yaml", 'w') as outfile:

        for section in cfg:
            for item in section["lasers"]:
                print type(item),
                item['dist_correction']=0
                item['dist_correction_x']=0
                item['dist_correction_y']=0
                item['focal_distance']=0
                item['focal_slope']=0
                item['vert_offset_correction']=0

                item['two_pt_correction_available']=False
       
        outfile.write( yaml.dump(section, default_flow_style=False) )
'''
- {dist_correction: 0.100000001490116, dist_correction_x: 0, dist_correction_y: 0,
  focal_distance: 0, focal_slope: 0, laser_id: 0, max_intensity: 255, min_intensity: 0,
  rot_correction: -0.0698131695389748, two_pt_correction_available: false, vert_correction: -0.124932751059532,
  vert_offset_correction: 0}

- dist_correction: 1.2011153
  dist_correction_x: 1.2366819
  dist_correction_y: 1.2224358
  focal_distance: 10.0
  focal_slope: 1.0
  horiz_offset_correction: 0.025999999
  laser_id: 0
  max_intensity: 255
  min_intensity: 0
  rot_correction: -0.1005212643748908
  two_pt_correction_available: false
  vert_correction: -0.12376696608832838
  vert_offset_correction: 0.19762794

'''
