#!/usr/bin/env python3

import jinja2
import argparse
import os
import shutil
import fnmatch
import rospkg
import numpy as np
import json

def get_file_contents(filepath):
    with open(filepath, 'rb') as f:
        return f.read()

def str2bool(v):
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('filepath', help="full path to the jinja file that the sdf file should be generated from")
    parser.add_argument('--mavlink_addr', default="INADDR_ANY", help="IP address for PX4 SITL")
    parser.add_argument('--mavlink_udp_port', default=14560, help="Mavlink UDP port for mavlink access")
    parser.add_argument('--mavlink_tcp_port', default=4560, help="TCP port for PX4 SITL")
    parser.add_argument('--serial_enabled', default=0, help="Enable Serial device for HITL")
    parser.add_argument('--serial_device', default="/dev/ttyACM0", help="Serial device for FMU")
    parser.add_argument('--serial_baudrate', default=921600, help="Baudrate of Serial device for FMU")
    parser.add_argument('--qgc_addr', default="INADDR_ANY", help="IP address for QGC")
    parser.add_argument('--qgc_udp_port', default=14550, help="UDP port for QGC")
    parser.add_argument('--sdk_addr', default="INADDR_ANY", help="IP address for MAVSDK")
    parser.add_argument('--sdk_udp_port', default=14540, help="UDP port for MAVSDK")
    parser.add_argument('--hil_mode', default=0, help="Enable HIL mode for HITL simulation")
    parser.add_argument('--hil_state_level', default=0, help="HIL state level HITL simulation")
    parser.add_argument('--send_vision_estimation', default=0)
    parser.add_argument('--send_odometry', default=1)
    parser.add_argument('--use_lockstep', default=1, help="Enable simulation lockstep for syncing physics&sensors")
    parser.add_argument('--use_tcp', default=1, help="Use TCP instead of UDP for PX4 SITL")
    parser.add_argument('--visual_material', default="DarkGrey", help="Default texture for 3D models")
    parser.add_argument('--gps_indoor_jamming', default=0, help="Trigger bad GPS when the vehicle has obstacles above it")
    parser.add_argument('--output-file', help="sdf output file")
    parser.add_argument('--model_config_file', help="config file with list of sensors for particular drone")
    parser.add_argument('--stdout', action='store_true', default=False, help="dump to stdout instead of file")
    parser.add_argument('--namespace', default="uav", help="drone namespace")
    args, _ = parser.parse_known_args()

    print('Generating a templated model using jinja')

    filename = os.path.basename(args.filepath)
    model_name = filename.split('.')[0]
    model_dir = os.path.realpath(os.path.dirname(args.filepath))

    # find the path to the mrs_robots_description/sdf directory which should always be included
    rospack = rospkg.RosPack()
    mrs_uav_gazebo_simulation_path = os.path.realpath(rospack.get_path('mrs_uav_gazebo_simulation'))
    mrs_models_dir = os.path.join(mrs_uav_gazebo_simulation_path, 'models')

    print('[AAAAAAAAAAAAAAA]: ', model_dir)
    print('[AAAAAAAAAAAAAAA]: ', mrs_models_dir)
    print('[AAAAAAAAAAAAAAA]: ', filename)

    env = jinja2.Environment(loader=jinja2.FileSystemLoader([model_dir, mrs_models_dir]))
    # print("Jinja env. directories:\n\t{}\n\t{}".format(model_dir, mrs_models_dir))
    
    template = env.get_template(filename)

    d = {'name' : model_name, \
         'mavlink_addr': args.mavlink_addr, \
         'mavlink_udp_port': args.mavlink_udp_port, \
         'mavlink_tcp_port': args.mavlink_tcp_port, \
         'serial_enabled': args.serial_enabled, \
         'serial_device': args.serial_device, \
         'serial_baudrate': args.serial_baudrate, \
         'qgc_addr': args.qgc_addr, \
         'qgc_udp_port': args.qgc_udp_port, \
         'sdk_addr': args.sdk_addr, \
         'sdk_udp_port': args.sdk_udp_port, \
         'hil_mode': args.hil_mode, \
         'hil_state_level': args.hil_state_level, \
         'send_vision_estimation': args.send_vision_estimation, \
         'send_odometry': args.send_odometry, \
         'use_lockstep': args.use_lockstep, \
         'use_tcp': args.use_tcp, \
         'visual_material': args.visual_material, \
         'gps_indoor_jamming': args.gps_indoor_jamming, \
         'namespace': args.namespace}
    

    # reading the data from the file
    with open(args.model_config_file) as f:
        data = f.read()
  
    # reconstructing the data as a dictionary
    js = json.loads(data)
    d.update(js)

    result = template.render(d)

    if args.output_file:
        filename_out = args.output_file
    else:
        if not filename.endswith('.sdf.jinja'):
            raise Exception("ERROR: Output file can only be determined automatically for " + \
                            "input files with the .sdf.jinja extension")
        filename_out = filename.replace('.sdf.jinja', '.sdf')
        assert filename_out != filename, "Not allowed to overwrite template"

    # Overwrite protection mechanism: after generation, the file will be copied to a "last_generated" file.
    # In the next run, we can check whether the target file is still unmodified.
    filename_out_last_generated = filename_out + '.last_generated'

    if os.path.exists(filename_out) and os.path.exists(filename_out_last_generated):
        # Check whether the target file is still unmodified.
        if get_file_contents(filename_out).strip() != get_file_contents(filename_out_last_generated).strip():
            raise Exception('Generation would overwrite changes to "' + str(filename_out) +\
                            '".\nChanges should only be made to the template file "' + str(filename) + \
                            '"\nRemove "' + str(filename_out) + '"\n(after extracting your changes) to disable this overwrite protection.')

    with open(filename_out, 'w') as f_out:
        # print('Output generated "%s"' % filename_out)
        f_out.write(result)

    # Copy the contents to a "last_generated" file for overwrite protection check next time.
    shutil.copy(filename_out, filename_out_last_generated)

    if args.stdout:
        print(result)
