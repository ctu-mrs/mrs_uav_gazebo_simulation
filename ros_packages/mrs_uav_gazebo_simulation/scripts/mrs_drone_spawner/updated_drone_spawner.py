#!/usr/bin/python3
import atexit
import sys
import rospkg
import jinja2
from jinja2 import meta

glob_running_processes = []

# #{ exit_handler
def exit_handler():
    print('[INFO] [MrsDroneSpawner]: Exit requested')

    if len(glob_running_processes) > 0:
        print(f'[INFO] [MrsDroneSpawner]: Shutting down {len(glob_running_processes)} subprocesses')

        num_zombies = 0
        for p, pid in glob_running_processes:
            try:
                p.shutdown()
                print(f'[INFO] [MrsDroneSpawner]: Process {pid} shutdown')
            except:
                num_zombies += 1

        if num_zombies > 0:
            print(f'\033[91m[ERROR] [MrsDroneSpawner]: Could not stop {num_zombies} subprocesses\033[91m')
            exit(1)

    print('[INFO] [MrsDroneSpawner]: Exited gracefully')
    exit(0)
# #}

def filter_jinja_templates(template_name):
    return template_name.endswith('.sdf.jinja')

class MrsDroneSpawner:

    def __init__(self, show_help=False, verbose=False):

        self.verbose = verbose
        self.rospack = rospkg.RosPack()


        self.resource_paths = []
        self.resource_paths.append(self.rospack.get_path('mrs_uav_gazebo_simulation'))
        self.resource_paths.append(self.rospack.get_path('external_gazebo_models'))


        self.jinja_env = jinja2.Environment(loader=jinja2.FileSystemLoader(self.resource_paths))

        print('Jinja templates available:')
        print(self.jinja_env.list_templates(filter_func=filter_jinja_templates))

        # load all available models
            # from current package
            # from external packages (multiple)

        # compose all available params

        # display help (params) for individual models

    def extract_jinja_params(self, model_name):
        for n in self.jinja_env.list_templates(filter_func=filter_jinja_templates):
            if model_name in n:
                template_filepath = self.jinja_env.get_template(n).filename
                with open(template_filepath, 'r') as f:
                    content = f.read()
                    parsed_content = self.jinja_env.parse(content)
                    variable_names = meta.find_undeclared_variables(parsed_content)
                    return sorted(variable_names)

if __name__ == '__main__':

    print('[INFO] [MrsDroneSpawner]: Starting')
    
    show_help = 'no_help' not in sys.argv
    verbose = 'verbose' in sys.argv

    atexit.register(exit_handler)

    try:
        spawner = MrsDroneSpawner(show_help, verbose)
        params = spawner.extract_jinja_params('x400')
        print(params)
    except rospy.ROSInterruptException:
        pass
