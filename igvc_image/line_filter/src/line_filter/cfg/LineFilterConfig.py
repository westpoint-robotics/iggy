## *********************************************************
## 
## File autogenerated for the line_filter package 
## by the dynamic_reconfigure package.
## Please do not edit.
## 
## ********************************************************/

##**********************************************************
## Software License Agreement (BSD License)
##
##  Copyright (c) 2008, Willow Garage, Inc.
##  All rights reserved.
##
##  Redistribution and use in source and binary forms, with or without
##  modification, are permitted provided that the following conditions
##  are met:
##
##   * Redistributions of source code must retain the above copyright
##     notice, this list of conditions and the following disclaimer.
##   * Redistributions in binary form must reproduce the above
##     copyright notice, this list of conditions and the following
##     disclaimer in the documentation and/or other materials provided
##     with the distribution.
##   * Neither the name of the Willow Garage nor the names of its
##     contributors may be used to endorse or promote products derived
##     from this software without specific prior written permission.
##
##  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
##  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
##  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
##  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
##  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
##  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
##  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
##  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
##  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
##  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
##  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
##  POSSIBILITY OF SUCH DAMAGE.
##**********************************************************/

from dynamic_reconfigure.encoding import extract_params

inf = float('inf')

config_description = {'upper': 'DEFAULT', 'lower': 'groups', 'srcline': 233, 'name': 'Default', 'parent': 0, 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT', 'field': 'default', 'state': True, 'parentclass': '', 'groups': [], 'parameters': [{'srcline': 259, 'description': 'Threshold - hue lower', 'max': 180, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'hue_lower', 'edit_method': '', 'default': 85, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 259, 'description': 'Threshold - hue upper', 'max': 180, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'hue_upper', 'edit_method': '', 'default': 120, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 259, 'description': 'Threshold - saturation lower', 'max': 255, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'saturation_lower', 'edit_method': '', 'default': 0, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 259, 'description': 'Threshold - saturation upper', 'max': 255, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'saturation_upper', 'edit_method': '', 'default': 25, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 259, 'description': 'Threshold - value lower', 'max': 255, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'value_lower', 'edit_method': '', 'default': 240, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 259, 'description': 'Threshold - value upper', 'max': 255, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'value_upper', 'edit_method': '', 'default': 255, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 259, 'description': 'Threshold image diplay window enable', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'threshold_image_debug', 'edit_method': '', 'default': False, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 259, 'description': 'Kernel size for the bilateral filter', 'max': 20, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'blur_kernel_size', 'edit_method': '', 'default': 7, 'level': 0, 'min': 2, 'type': 'int'}, {'srcline': 259, 'description': 'Erosion morphological filter enable', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'erosion_bool', 'edit_method': '', 'default': True, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 259, 'description': 'Erosion morphological filter kernel type', 'max': 2, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'erosion_element', 'edit_method': '', 'default': 2, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 259, 'description': 'Erosion filter size', 'max': 25, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'erosion_size', 'edit_method': '', 'default': 2, 'level': 0, 'min': 1, 'type': 'int'}, {'srcline': 259, 'description': 'Dilation morphological filter enable', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'dilation_bool', 'edit_method': '', 'default': True, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 259, 'description': 'Dilation morphological filter kernel type', 'max': 2, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'dilation_element', 'edit_method': '', 'default': 2, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 259, 'description': 'Dialtion filter size', 'max': 25, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'dilation_size', 'edit_method': '', 'default': 7, 'level': 0, 'min': 1, 'type': 'int'}, {'srcline': 259, 'description': 'Other morphological filter enable', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'morphology_bool', 'edit_method': '', 'default': True, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 259, 'description': 'Other morphological filter operation type', 'max': 4, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'morphology_operation', 'edit_method': '', 'default': 1, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 259, 'description': 'Other morphological filter kernel type', 'max': 2, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'morphology_element', 'edit_method': '', 'default': 2, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 259, 'description': 'Other morphological filter size', 'max': 25, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'morphology_size', 'edit_method': '', 'default': 6, 'level': 0, 'min': 1, 'type': 'int'}], 'type': '', 'id': 0}

min = {}
max = {}
defaults = {}
level = {}
type = {}
all_level = 0

#def extract_params(config):
#    params = []
#    params.extend(config['parameters'])    
#    for group in config['groups']:
#        params.extend(extract_params(group))
#    return params

for param in extract_params(config_description):
    min[param['name']] = param['min']
    max[param['name']] = param['max']
    defaults[param['name']] = param['default']
    level[param['name']] = param['level']
    type[param['name']] = param['type']
    all_level = all_level | param['level']
