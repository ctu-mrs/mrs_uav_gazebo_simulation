#!/usr/bin/python3
import ast
import atexit
import copy
import csv
import datetime
import jinja2
import jinja2.meta
import math
import os
import random
import re
import roslaunch
import rospkg
import rospy
import sys
import tempfile
import threading
import xml.dom.minidom
import yaml

from inspect import getmembers, isfunction

from component_wrapper import ComponentWrapper
from template_wrapper import TemplateWrapper

from mrs_msgs.srv import String as StringSrv
from mrs_msgs.srv import StringResponse as StringSrvResponse
from mrs_msgs.srv import StringRequest as StringSrvRequest
from mrs_msgs.msg import GazeboSpawnerDiagnostics
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SpawnModel as SpawnModelSrv
from gazebo_msgs.srv import DeleteModel as DeleteModelSrv
from geometry_msgs.msg import Pose

glob_running_processes = []

# #{ Exceptions and Errors

# #{ NoFreeIDAvailable(RuntimeError)
class NoFreeIDAvailable(RuntimeError):
    '''Indicate that the vehicle limit imposed by px4 sitl has been reached'''
    pass
# #}

# #{ NoValidIDGiven(RuntimeError)
class NoValidIDGiven(RuntimeError):
    '''Indicate that the user has provided only invalid IDs (non-integers, or all are already assigned)'''
    pass
# #}

# #{ CouldNotLaunch(RuntimeError)
class CouldNotLaunch(RuntimeError):
    '''Indicate that a subprocess could not be launched'''
    pass
# #}

# #{ FormattingError(ValueError)
class FormattingError(ValueError):
    '''Indicate that the input is not properly formatted'''
    pass
# #}

# #{ WrongNumberOfArguments(ValueError)
class WrongNumberOfArguments(ValueError):
    '''Indicate that the expected number of arguments is different'''
    pass
# #}

# #{ SuffixError(NameError)
class SuffixError(NameError):
    '''Indicate that the file has an unexpected suffix'''
    pass
# #}

# #}

# #{ dummy_function()
def dummy_function():
    '''Empty function to temporarily replace rospy signal handlers'''
    pass
# #}

# #{ filter_templates(template_name)
def filter_templates(template_name, suffix):
    '''Comparator used to load files with given suffix'''
    return template_name.endswith(suffix)
# #}

# #{ get_ros_package_name(filepath)
def get_ros_package_name(filepath):
    '''Return the name of a ros package that contains a given filepath'''
    tmp_filepath = copy.copy(filepath)
    while rospkg.get_package_name(tmp_filepath) is None:
        tmp_filepath = os.path.dirname(tmp_filepath)
    return rospkg.get_package_name(tmp_filepath)
# #}

# #{ exit_handler()
def exit_handler():
    '''
    Kill all subprocesses started by the spawner to prevent orphaned processes (mainly px4 and mavros)
    '''
    print('[INFO] [MrsDroneSpawner]: Exit requested')

    if len(glob_running_processes) > 0:
        print(f'[INFO] [MrsDroneSpawner]: Shutting down {len(glob_running_processes)} subprocesses')

        num_zombies = 0
        for p in glob_running_processes:
            try:
                p.shutdown()
                print(f'[INFO] [MrsDroneSpawner]: Process {p.run_id} shutdown')
            except:
                num_zombies += 1

        if num_zombies > 0:
            print(f'\033[91m[ERROR] [MrsDroneSpawner]: Could not stop {num_zombies} subprocesses\033[91m')
            exit(1)

    print('[INFO] [MrsDroneSpawner]: Exited gracefully')
    exit(0)
# #}

class MrsDroneSpawner:

    # #{ __init__(self, verbose=False)
    def __init__(self, verbose=False):

        self.is_initialized = False

        self.verbose = verbose

        rospy.init_node('mrs_drone_spawner', anonymous=True)

        resource_paths = []
        self.rospack = rospkg.RosPack()
        resource_paths.append(os.path.join(self.rospack.get_path('mrs_uav_gazebo_simulation'), 'models'))

        # # #{ load required params
        try:
            self.vehicle_base_port = rospy.get_param('~mavlink_config/vehicle_base_port')
            self.mavlink_tcp_base_port = rospy.get_param('~mavlink_config/mavlink_tcp_base_port')
            self.mavlink_udp_base_port = rospy.get_param('~mavlink_config/mavlink_udp_base_port')
            self.qgc_udp_port = rospy.get_param('~mavlink_config/qgc_udp_port')
            self.sdk_udp_port = rospy.get_param('~mavlink_config/sdk_udp_port')
            self.send_vision_estimation = rospy.get_param('~mavlink_config/send_vision_estimation')
            self.send_odometry = rospy.get_param('~mavlink_config/send_odometry')
            self.enable_lockstep = rospy.get_param('~mavlink_config/enable_lockstep')
            self.use_tcp = rospy.get_param('~mavlink_config/use_tcp')
            self.template_suffix = rospy.get_param('~jinja_templates/suffix')
            self.save_sdf_files = rospy.get_param('~jinja_templates/save_rendered_sdf')
            self.default_robot_name = rospy.get_param('~gazebo_models/default_robot_name')
            self.model_spacing = rospy.get_param('~gazebo_models/spacing')
        except KeyError as err:
            rospy.logerr(f'[MrsDroneSpawner]: Could not load required param {err}')
            raise rospy.ROSInterruptException
        # # #}

        # # #{ handle extra resource paths
        extra_resource_paths = rospy.get_param('~extra_resource_paths', [])
        for elem in extra_resource_paths:
            if os.path.exists(elem):
                rpath = elem
            else:
                rpath = self.rospack.get_path(elem)
            rospy.loginfo(f'[MrsDroneSpawner]: Adding extra resources from {rpath}')
            resource_paths.append(rpath)

        self.jinja_env = self.configure_jinja2_environment(resource_paths)
        # # #}

        # # #{ find launchfiles for mavros and px4_firmware
        gazebo_simulation_path = self.rospack.get_path('mrs_uav_gazebo_simulation')
        px4_api_path = self.rospack.get_path('mrs_uav_px4_api')
        self.mavros_launch_path = px4_api_path + os.sep + 'launch' + os.sep + 'mavros_gazebo_simulation.launch'
        self.px4_fimrware_launch_path = gazebo_simulation_path + os.sep + 'launch' + os.sep + 'run_simulation_firmware.launch'
        # # #}

        try:
            self.jinja_templates = self.build_template_database()
        except RecursionError as err:
            rospy.logerr(f'[MrsDroneSpawner]: {err}')
            raise rospy.ROSInterruptException

        rospy.loginfo(f'[MrsDroneSpawner]: ------------------------')
        rospy.loginfo(f'[MrsDroneSpawner]: Jinja templates loaded:')
        for name in self.jinja_templates.keys():
            rospy.loginfo(f'[MrsDroneSpawner]: "{name}" from "{self.jinja_templates[name].jinja_template.filename}"')
        rospy.loginfo(f'[MrsDroneSpawner]: ------------------------')

        # # #{ setup communication
        self.spawn_server = rospy.Service('~spawn', StringSrv, self.callback_spawn, buff_size=20)
        self.diagnostics_pub = rospy.Publisher('~diagnostics', GazeboSpawnerDiagnostics, queue_size=1)
        self.diagnostics_timer = rospy.Timer(rospy.Duration(0.1), self.callback_diagnostics_timer)
        self.action_timer = rospy.Timer(rospy.Duration(0.1), self.callback_action_timer)

        self.gazebo_spawn_proxy = rospy.ServiceProxy('~gazebo_spawn_model', SpawnModelSrv)
        self.gazebo_delete_proxy = rospy.ServiceProxy('~gazebo_delete_model', DeleteModelSrv)
        # # #}

        # #{ setup system variables
        self.spawn_called = False
        self.processing = False
        self.vehicle_queue = []
        self.queue_mutex = threading.Lock()
        self.active_vehicles = []
        self.assigned_ids = set()
        # #}

        self.is_initialized = True

        rospy.spin()

        # load all available models
        # from current package
        # from external packages (multiple)

        # compose all available params

        # display help (params) for individual models

    # #} end init

    # --------------------------------------------------------------
    # |                    jinja template utils                    |
    # --------------------------------------------------------------

    # #{ get_all_templates(self)
    def get_all_templates(self):
        '''
        Get all templates loaded by the given jinja environment
        :returns a list of tuples, consisting of (str_name, jinja2.Template)
        '''
        template_names = self.jinja_env.list_templates(filter_func=lambda template_name: filter_templates(template_name, self.template_suffix))
        templates = []
        for i, full_name in enumerate(template_names):
            rospy.loginfo(f'[MrsDroneSpawner]: \t({i+1}/{len(template_names)}): {full_name}')
            template_name = full_name.split(os.path.sep)[-1][:-(len(self.template_suffix))]
            templates.append((template_name, self.jinja_env.get_template(full_name)))
        return templates
    # #}

    # #{ get_template_imports(self, jinja_template)
    def get_template_imports(self, jinja_template):
        '''Returns a list of sub-templates imported by a given jinja_template'''
        with open(jinja_template.filename, 'r') as f:
            template_source = f.read()
            preprocessed_template = template_source.replace('\n', '')
            parsed_template = self.jinja_env.parse(preprocessed_template)
            import_names = [node.template.value for node in parsed_template.find_all(jinja2.nodes.Import)]
            imported_templates = []
            for i in import_names:
                template = self.jinja_env.get_template(i)
                imported_templates.append(template)
            return imported_templates
    # #}

    # #{ get_spawner_components_from_template(self, template)
    def get_spawner_components_from_template(self, template):
        '''
        Builds a dict of spawner-compatible macros in a given template and their corresponding ComponentWrapper objects
        Does NOT check for macros imported from other templates
        :return a dict in format {macro name: component_wrapper.ComponentWrapper}
        '''
        with open(template.filename, 'r') as f:
            template_source = f.read()
            preprocessed_template = template_source.replace('\n', '')
            parsed_template = self.jinja_env.parse(preprocessed_template)
            macro_nodes = [node for node in parsed_template.find_all(jinja2.nodes.Macro)]
            spawner_components = {}
            for node in macro_nodes:
                spawner_keyword = None
                spawner_description = None
                spawner_default_args = None
                for elem in node.body:
                    if isinstance(elem, jinja2.nodes.Assign) and elem.target.name == 'spawner_description':
                        spawner_description = elem.node.value
                    if isinstance(elem, jinja2.nodes.Assign) and elem.target.name == 'spawner_default_args':
                        if isinstance(elem.node, jinja2.nodes.Const):
                            spawner_default_args = elem.node.value
                        elif isinstance(elem.node, jinja2.nodes.List):
                            spawner_default_args = []
                            for e in elem.node.items:
                                spawner_default_args.append(e.value)
                        elif isinstance(elem.node, jinja2.nodes.Dict):
                            spawner_default_args = {}
                            for pair in elem.node.items:
                                spawner_default_args[pair.key.value] = pair.value.value
                        else:
                            rospy.logwarn(f'[MrsDroneSpawner]: Unsupported param type "{type(elem.node)}" in template {template.filename}')
                    if isinstance(elem, jinja2.nodes.Assign) and elem.target.name == 'spawner_keyword':
                        spawner_keyword = elem.node.value
                if spawner_keyword is not None:
                    spawner_components[node.name] = ComponentWrapper(spawner_keyword, spawner_description, spawner_default_args)
            return spawner_components
    # #}

    # #{ get_accessible_components(self, template_wrapper, all_components)
    def get_accessible_components(self, template_wrapper, all_components):
        '''
        Recursive function to get all spawner-compatible components accessible from template_wrapper
        Includes components in imported sub-templates
        :param template_wrapper: template_wrapper.TemplateWrapper for which we want to load components
        :param all_components: a dict to which all found ComponentWrappers will be added
        :returns a dict of objects {macro name: component_wrapper.ComponentWrapper}
        '''
        all_components.update(template_wrapper.components)
        for i in template_wrapper.imported_templates:
            try:
                all_components.update(self.get_accessible_components(i, all_components))
            except RecursionError as err:
                raise RecursionError(f'Cyclic import detected in file {template_wrapper.jinja_template.filename}. Fix your templates')
        return all_components
    # #}

    # #{ get_callable_components(self, template)
    def get_callable_components(self, template, accessible_components):
        '''
        Get all components that are actually called from a template
        :param template: a jinja template file
        :param accessible_components: a dict of macros accessible from this template (including imported modules)
        :returns a dictionary of callable components {macro_name: component_wrapper.ComponentWrapper}
        sorted alphabetically by keywords
        '''
        callable_components = {}
        with open(template.filename, 'r') as f:
            template_source = f.read()
            preprocessed_template = template_source.replace('\n', '')
            parsed_template = self.jinja_env.parse(preprocessed_template)
            call_nodes = [node for node in parsed_template.find_all(jinja2.nodes.Call)]
            callable_components = {}
            for node in call_nodes:
                if isinstance(node.node, jinja2.nodes.Getattr):
                    if node.node.attr in accessible_components.keys():
                        callable_components[node.node.attr] = accessible_components[node.node.attr]
        return dict(sorted(callable_components.items(), key=lambda item: item[1].keyword))
    # #}

    # #{ build_template_database(self)
    def build_template_database(self):
        '''
        Generate a database of jinja2 templates available to the spawner
        Scans through all folders provided into the jinja2 environment for files with matching target suffix
        Recursively checks templates imported by templates, prevents recursion loops
        Returns a dictionary of template_wrapper.TemplateWrapper objects in format {template_name: template_wrapper.TemplateWrapper}
        '''

        template_wrappers = {}

        rospy.loginfo('[MrsDroneSpawner]: Loading all templates')
        all_templates = self.get_all_templates()
        for name, template in all_templates:
            imports = self.get_template_imports(template)
            components = self.get_spawner_components_from_template(template)
            package_name = get_ros_package_name(template.filename)
            wrapper = TemplateWrapper(template, imports, components, package_name)
            template_wrappers[name] = wrapper

        rospy.loginfo('[MrsDroneSpawner]: Reindexing imported templates')
        for name, wrapper in template_wrappers.items():
            for i, it in enumerate(wrapper.imported_templates):
                if not isinstance(it, TemplateWrapper):
                    for ww in template_wrappers.values():
                        if ww.jinja_template == it:
                            wrapper.imported_templates[i] = ww

        rospy.loginfo('[MrsDroneSpawner]: Adding available components from dependencies')
        for _, wrapper in template_wrappers.items():
            prev_limit = sys.getrecursionlimit()
            sys.setrecursionlimit(len(template_wrappers) + 1)
            wrapper.components = self.get_accessible_components(wrapper, {})
            sys.setrecursionlimit(prev_limit)

        rospy.loginfo('[MrsDroneSpawner]: Pruning components to only include callables')
        callable_components = {}
        for name, template in all_templates:
            callable_components[name] = self.get_callable_components(template, template_wrappers[name].components)

        for name, wrapper in template_wrappers.items():
            wrapper.components = callable_components[name]

        rospy.loginfo('[MrsDroneSpawner]: Template database built')

        return template_wrappers
    # #}

    # #{ configure_jinja2_environment(self, resource_paths)
    def configure_jinja2_environment(self, resource_paths):
        '''Create a jinja2 environment and setup its variables'''
        env = jinja2.Environment(
            loader=jinja2.FileSystemLoader(resource_paths),
            autoescape=None
        )
        # Allows use of math module directly in the templates
        env.globals['math'] = math

        return env
    # #}

    # #{ render(self, spawner_args)
    def render(self, spawner_args):
        '''
        Renders a jinja template into a sdf, creates a formatted xml
        Input has to specify the template name in spawner_args["model"]
        :param spawner_args: a dict to be passed into the template as variables, format {component_name (string): args (list or dict)}
        :return: content of the xml file as a string or None
        '''

        params = {
            "spawner_args": spawner_args
        }

        try:
            model_name = spawner_args['model']
        except KeyError:
            rospy.logerr(f'[MrsDroneSpawner]: Cannot render template, model not specified')
            return

        try:
            template_wrapper = self.jinja_templates[model_name]
        except KeyError:
            rospy.logerr(f'[MrsDroneSpawner]: Cannot render model "{model_name}". Template not found!')
            return

        rospy.loginfo(f'[MrsDroneSpawner]: Rendering model "{model_name}" using template {template_wrapper.jinja_template.filename}')

        context = template_wrapper.jinja_template.new_context(params)
        rendered_template = template_wrapper.jinja_template.render(context)
        try:
            root = xml.dom.minidom.parseString(rendered_template)
        except Exception as e:
            rospy.logerr(f'[MrsDroneSpawner]: XML error: "{e}"')
            fd, filepath = tempfile.mkstemp(prefix='mrs_drone_spawner_' + datetime.datetime.now().strftime("%Y_%m_%d__%H_%M_%S_"), suffix='_DUMP_' + str(model_name) + '.sdf')
            with os.fdopen(fd, 'w') as output_file:
                output_file.write(rendered_template)
                rospy.loginfo(f'[MrsDroneSpawner]: Malformed XML for model {model_name} dumped to {filepath}')
            return

        ugly_xml = root.toprettyxml(indent='  ')

        # Remove empty lines
        pretty_xml = '\n'.join(line for line in ugly_xml.split('\n') if line.strip())

        return pretty_xml

    # #} end render

    # --------------------------------------------------------------
    # |                     user input parsing                     |
    # --------------------------------------------------------------

    # #{ parse_user_input(self, input_str)
    def parse_user_input(self, input_str):
        '''
        Extract params from an input string, create spawner args
        expected input:
            device ids (integers separated by spaces)
            keywords (specified in jinja components starting with '--')
            component args following a keyword (values separated by spaces)
        :param input_str: string containing all args in the format specified above
        :return: a dict in format {keyword: component_args}, always contains keys "help", "model", "ids", "names", "spawn_poses"
        NOTE: arguments of a component/keyword will always be parsed as a list/dict, even for a single value

        Raises:
        AssertionError in case of unexpected data in mandatory values under keys "model", "ids", "names", "spawn_poses"
        '''

        input_dict = {
            'help': False,
            'model': None,
            'ids': [],
            'names': [],
            'spawn_poses': {},
        }

        # parse out the keywords starting with '--'
        pattern = re.compile(r'(--\S*)')
        substrings = [m.strip() for m in re.split(pattern, input_str) if len(m.strip()) > 0]

        if len(substrings) < 1:
            input_dict['help'] = True
            return input_dict

        # before the first keyword, there should only be device IDs
        first_keyword_index = 0
        if '--' not in substrings[0]:
            input_dict['ids'] = self.parse_string_to_objects(substrings[0])
            first_keyword_index = 1
        else:
            input_dict['ids'].append(self.assign_free_id())

        # pair up keywords with args
        for i in range(first_keyword_index, len(substrings)):

            if substrings[i].startswith('--'):
                input_dict[substrings[i][2:]] = None
                continue
            else:
                input_keys = [*input_dict.keys()]
                if len(input_keys) > 1:
                    input_dict[input_keys[-1]] = self.parse_string_to_objects(substrings[i])

        # attempt to match model to available templates
        for k in input_dict.keys():
            if k in self.jinja_templates.keys():
                input_dict['model'] = str(k)
                del input_dict[k]
                break


        valid_ids = []

        for ID in input_dict['ids']:
            if not isinstance(ID, int):
                if ID in self.jinja_templates.keys() and input_dict['model'] is None:
                    rospy.loginfo(f'[MrsDroneSpawner]: Using {ID} as model template')
                    input_dict['model'] = ID
                else:
                    rospy.logwarn(f'[MrsDroneSpawner]: Ignored ID {ID}: Not an integer')
                continue
            if ID < 0 or ID > 255:
                rospy.logwarn(f'[MrsDroneSpawner]: Ignored ID {ID}: Must be in range(0, 256)')
                continue
            if ID in self.assigned_ids:
                rospy.logwarn(f'[MrsDroneSpawner]: Ignored ID {ID}: Already assigned')
                continue
            valid_ids.append(ID)

        input_dict['ids'].clear()

        if '--help' in substrings:
            input_dict['help'] = True
            return input_dict

        if len(valid_ids) > 0:
            rospy.loginfo(f'[MrsDroneSpawner]: Valid robot IDs: {valid_ids}')
            input_dict['ids'] = valid_ids
            self.assigned_ids.update(input_dict['ids'])
        else:
            raise NoValidIDGiven('No valid ID given. Check your input')

        if 'pos' in input_dict.keys():
            try:
                input_dict['spawn_poses'] = self.get_spawn_poses_from_args(input_dict['pos'], input_dict['ids'])
            except (WrongNumberOfArguments, ValueError) as err:
                rospy.logerr(f'[MrsDroneSpawner]: While parsing args for "--pos": {err}')
                rospy.logwarn(f'[MrsDroneSpawner]: Assigning random spawn poses instead')
                input_dict['spawn_poses'] = self.get_randomized_spawn_poses(input_dict['ids'])
            finally:
                del input_dict['pos']

        elif 'pos-file' in input_dict.keys():
            try:
                input_dict['spawn_poses'] = self.get_spawn_poses_from_file(input_dict['pos-file'][0], input_dict['ids'])
            except (FileNotFoundError, SuffixError, FormattingError, WrongNumberOfArguments, ValueError) as err:
                rospy.logerr(f'[MrsDroneSpawner]: While parsing args for "--pos-file": {err}')
                rospy.logwarn(f'[MrsDroneSpawner]: Assigning random spawn poses instead')
                input_dict['spawn_poses'] = self.get_randomized_spawn_poses(input_dict['ids'])
            finally:
                del input_dict['pos-file']

        else:
            input_dict['spawn_poses'] = self.get_randomized_spawn_poses(input_dict['ids'])

        if 'name' in input_dict.keys():
            for ID in input_dict['ids']:
                input_dict['names'].append(str(input_dict['name'][0]) + str(ID))
            del input_dict['name']
        else:
            for ID in input_dict['ids']:
                input_dict['names'].append(str(self.default_robot_name) + str(ID))

        assert isinstance(input_dict['ids'], list) and len(input_dict['ids']) > 0, 'No vehicle ID assigned'
        assert input_dict['model'] is not None, 'Model not specified'
        assert isinstance(input_dict['names'], list) and len(input_dict['names']) == len(input_dict['ids']), f'Invalid vehicle names {input_dict["names"]}'
        assert isinstance(input_dict['spawn_poses'], dict) and len(input_dict['spawn_poses'].keys()) == len(input_dict['ids']), f'Invalid spawn poses {input_dict["spawn_poses"]}'

        return input_dict
    # #}

    # #{ parse_string_to_objects(self, input_str)
    def parse_string_to_objects(self, input_str):
        '''
        Attempt to convert input_str into a dictionary or a list
        Convert numerals into number datatypes whenever possible
        Returns None if the input cannot be interpreted as dict or list
        '''
        input_str = input_str.strip()

        params = []
        for s in input_str.split():
            if len(s) > 0:
                try:
                    # try to convert input_str to numbers
                    params.append(ast.literal_eval(s))
                except (SyntaxError, ValueError):
                    # leave non-numbers as string
                    params.append(s)


        params_dict = {}
        if isinstance(params, list):
            # try to convert named args into a dict
            for p in params:
                try:
                    if ':=' in p:
                        kw, arg = p.split(':=')
                        try:
                            # try to convert arg to number
                            params_dict[kw] = ast.literal_eval(arg)
                        except (SyntaxError, ValueError):
                            # leave non-numbers as string
                            params_dict[kw] = arg
                except TypeError:
                    pass

        if len(params_dict.keys()) > 0 and len(params_dict.keys()) == len(params):
            # whole input converted to a dict
            return params_dict
        else:
            return params

        return None
    # #}

    # #{ get_help_text(self, input_dict):
    def get_help_text(self, input_dict):
        '''
        Used to construct the help text (string) for a given dict of input args
        Returns:
            generic spawner help
            or
            help for a specific model
            or
            None (if the input does not contain "help")
        '''
        if not input_dict['help']:
            return None

        if input_dict['model'] is None:
            display_text = self.get_spawner_help_text()
        else:
            display_text = self.get_model_help_text(input_dict['model'])

        return display_text
    # #}

    # #{ get_model_help_text(self, model_name)
    def get_model_help_text(self, model_name):
        '''
        Create a help string by loading all callable components from a given template in the following format
        Component name
            Description:
            Default args:
        '''
        rospy.loginfo(f'[MrsDroneSpawner]: Getting help for model {model_name}')
        try:
            template_wrapper = self.jinja_templates[model_name]
            response = f'[MrsDroneSpawner]: Components used in template "{template_wrapper.jinja_template.filename}":\n'
        except ValueError:
            return f'Template for model {model_name} not found'

        for name, component in template_wrapper.components.items():
            response += f'{component.keyword}\n\tDescription: {component.description}\n\tDefault args: {component.default_args}\n\n'

        return response
    # #}

    # #{ get_spawner_help_text(self)
    def get_spawner_help_text(self):
        '''Create a generic help string for the spawner basic use'''

        rospy.loginfo(f'[MrsDroneSpawner]: Getting generic spawner help')
        response = 'The spawn service expects the following input (as a string):\n'
        response += '\tdevice ids (integers separated by spaces, auto-assigned if no ID is specified),\n'
        response += '\tmodel (use \'--\' with a model name to select a specific model),\n'
        response += '\tkeywords (specified inside jinja macros as "spawner_keyword". Add \'--\' before each keyword when calling spawn),\n'
        response += '\tcomponent args following a keyword (values separated by spaces or a python dict, overrides "spawner_default_args" in jinja macros),\n'
        response += '\n'
        response += '\tModels available: '

        for model_name in sorted(self.jinja_templates.keys()):
            response += f'{model_name}, '

        return response
    # #}

    # --------------------------------------------------------------
    # |                        ROS callbacks                       |
    # --------------------------------------------------------------

    # #{ callback_spawn(self, req)
    def callback_spawn(self, req):

        self.spawn_called = True

        rospy.loginfo(f'[MrsDroneSpawner]: Spawn called with args "{req.value}"')
        res = StringSrvResponse()
        res.success = False

        # #{ input parsing
        params_dict = None
        already_assigned_ids = copy.deepcopy(self.assigned_ids) # backup in case of mid-parse failure
        try:
            params_dict = self.parse_user_input(req.value)
        except Exception as e:
            rospy.logwarn(f'[MrsDroneSpawner]: While parsing user input: {e}')
            res.message = str(e.args[0])
            self.assigned_ids = already_assigned_ids
            return res
        # #}

        # #{ display help if needed
        help_text = self.get_help_text(params_dict)
        if help_text is not None:
            rospy.loginfo(help_text)
            inline_help_text = help_text
            inline_help_text = inline_help_text.replace('\n', ' ')
            inline_help_text = inline_help_text.replace('\t', ' ')
            res.message = inline_help_text
            res.success = True
            return res
        # #}

        rospy.loginfo(f'[MrsDroneSpawner]: Spawner params assigned "{params_dict}"')

        # #{ check gazebo running
        try:
            rospy.loginfo('[MrsDroneSpawner]: Waiting for /gazebo/model_states')
            rospy.wait_for_message('/gazebo/model_states', ModelStates, 60)
        except rospy.exceptions.ROSException:
            res.message = str('Gazebo model state topic not found. Is Gazebo running?')
            return res
        # #}

        rospy.loginfo('[MrsDroneSpawner]: Adding vehicles to a spawn queue')
        self.processing = True
        self.queue_mutex.acquire()
        for i, ID in enumerate(params_dict['ids']):
            robot_params = self.get_jinja_params_for_one_robot(params_dict, i, ID)
            name = robot_params['name']
            self.vehicle_queue.append(robot_params)
        self.queue_mutex.release()

        num_added = len(params_dict['ids'])

        res = StringSrvResponse()
        res.success = True
        res.message = f'Launch sequence queued for {num_added} robots'
        return res

    # #}

    # #{ callback_diagnostics_timer
    def callback_diagnostics_timer(self, timer):
        diagnostics = GazeboSpawnerDiagnostics()
        diagnostics.spawn_called = self.spawn_called
        diagnostics.processing = self.processing
        diagnostics.active_vehicles = self.active_vehicles
        self.queue_mutex.acquire()
        diagnostics.queued_vehicles = [params['name'] for params in self.vehicle_queue]
        diagnostics.queued_processes = len(self.vehicle_queue)
        self.queue_mutex.release()
        self.diagnostics_pub.publish(diagnostics)
    # #}

    # #{ callback_action_timer
    def callback_action_timer(self, timer):

        self.queue_mutex.acquire()

        if len(self.vehicle_queue) > 0:

            robot_params = self.vehicle_queue[0]
            del self.vehicle_queue[0]
            self.queue_mutex.release()

            orig_signal_handler = roslaunch.pmon._init_signal_handlers
            roslaunch.pmon._init_signal_handlers = dummy_function

            model_spawned = False
            firmware_process = None
            mavros_process = None

            try:
                model_spawned = self.spawn_gazebo_model(robot_params)
                firmware_process = self.launch_px4_firmware(robot_params)
                mavros_process  = self.launch_mavros(robot_params)

            except:
                # one of the subprocesses failed, perform cleanup
                if model_spawned:
                    self.delete_gazebo_model(robot_params['name'])
                if firmware_process is not None:
                    try:
                        firmware_process.shutdown()
                    except:
                        pass
                if mavros_process is not None:
                    try:
                        mavros_process.shutdown()
                    except:
                        pass
                self.assigned_ids.remove(robot_params['ID'])
                roslaunch.pmon._init_signal_handlers = orig_signal_handler
                return

            roslaunch.pmon._init_signal_handlers = orig_signal_handler
            glob_running_processes.append(firmware_process)
            glob_running_processes.append(mavros_process)

            rospy.loginfo(f'[MrsDroneSpawner]: Vehicle {robot_params["name"]} successfully spawned')
            self.active_vehicles.append(robot_params['name'])

        else:
            self.processing = False
            self.queue_mutex.release()
            # rinfo('Nothing to do')
    # #}

    # --------------------------------------------------------------
    # |                        Spawner utils                       |
    # --------------------------------------------------------------

    # #{ assign_free_id(self)
    def assign_free_id(self):
        '''
        Assign an unused ID in range <0, 255>
        :return: unused ID for a robot (int)
        :raise NoFreeIDAvailable: if max vehicle count has been reached
        '''
        for i in range(0, 256): # 255 is a hard limit of px4 sitl
            if i not in self.assigned_ids:
                rospy.loginfo(f'[MrsDroneSpawner]: Assigned free ID "{i}" to a robot')
                return i
        raise NoFreeIDAvailable('Cannot assign a free ID')
    # #}

    # #{ get_spawn_poses_from_file(self, filename, ids)
    def get_spawn_poses_from_file(self, filename, ids):
        '''
        Parses an input file and extracts spawn poses for vehicles. The file must be either ".csv" or ".yaml"

        CSV files have to include one line per robot, formatting: X, Y, Z, HEADING
        YAML files have to include one block per robot, formatting:
        block_header: # not used
            id: int
            x: float
            y: float
            z: float
            heading: float


        The file must contain spawn poses for all vehicles
        :param fileame: full path to a file
        :param ids: a list of ints containing unique vehicle IDs
        :return: a dict in format {id: {'x': pos_x, 'y', pos_y, 'z': pos_z, 'heading': pos_heading}}

        Raises:
        FileNotFoundError - if filename does not exist
        FormattingError - if the csv or yaml file does not match the expected structure
        SuffixError - filename has other suffix than ".csv" or ".yaml"
        WrongNumberOfArguments - number of poses defined in the file does not match the number of ids
        ValueError - spawn poses are not numbers
        '''

        rospy.loginfo(f'[MrsDroneSpawner]: Loading spawn poses from file "{filename}"')
        if not os.path.isfile(filename):
            raise FileNotFoundError(f'File "{filename}" does not exist!')

        spawn_poses = {}

        # #{ csv
        if filename.endswith('.csv'):
            array_string = list(csv.reader(open(filename)))
            for row in array_string:
                if (len(row)!=5):
                    raise FormattingError(f'Incorrect data in file "{filename}"! Data in ".csv" file type should be in format [id, x, y, z, heading] (types: int, float, float, float, float)')
                if int(row[0]) in ids:
                    spawn_poses[int(row[0])] = {'x' : float(row[1]), 'y' : float(row[2]), 'z' : float(row[3]), 'heading' : float(row[4])}
        # #}

        # #{ yaml
        elif filename.endswith('.yaml'):
            dict_vehicle_info = yaml.safe_load(open(filename, 'r'))
            for item, data in dict_vehicle_info.items():
                if (len(data.keys())!=5):
                    raise FormattingError(f'Incorrect data in file "{filename}"! Data  in ".yaml" file type should be in format \n uav_name: \n\t id: (int) \n\t x: (float) \n\t y: (float) \n\t z: (float) \n\t heading: (float)')

                if int(data['id']) in ids:
                    spawn_poses[data['id']] = {'x' : float(data['x']), 'y' : float(data['y']), 'z' : float(data['z']), 'heading' : float(data['heading'])}
        # #}

        else:
            raise SuffixError(f'Incorrect file type! Suffix must be either ".csv" or ".yaml"')

        if len(spawn_poses.keys()) != len(ids) or set(spawn_poses.keys()) != set(ids):
            raise WrongNumberOfArguments(f'File "{filename}" does not specify poses for all robots!')

        rospy.loginfo(f'[MrsDroneSpawner]: Spawn poses returned: {spawn_poses}')
        return spawn_poses
    # #}

    # #{ get_spawn_poses_from_args(self, pos_args, ids)
    def get_spawn_poses_from_args(self, pos_args, ids):
        '''
        Parses the input args extracts spawn poses for vehicles.
        If more vehicles are spawned at the same time, the given pose is used for the first vehicle.
        Additional vehicles are spawned with an offset of {config param: gazebo_models/spacing} meters in X

        :param pos_args: a list of 4 numbers [x,y,z,heading]
        :param ids: a list of ints containing unique vehicle IDs
        :return: a dict in format {id: {'x': pos_x, 'y', pos_y, 'z': pos_z, 'heading': pos_heading}}

        Raises:
        WrongNumberOfArguments - pos_args does not contain exactly 4 values
        ValueError - input cannot be converted into numbers
        '''
        spawn_poses = {}
        if len(pos_args) != 4:
            raise WrongNumberOfArguments(f'Expected exactly 4 args after keyword "--pos", got {len(pos_args)}')

        x = float(pos_args[0])
        y = float(pos_args[1])
        z = float(pos_args[2])
        heading = float(pos_args[3])

        spawn_poses[ids[0]] = {'x': x, 'y': y, 'z': z, 'heading': heading}

        if len(ids) > 1:
            rospy.logwarn(f'[MrsDroneSpawner]: Spawning more than one vehicle with "--pos". Each additional vehicle will be offset by {self.model_spacing} meters in X')
            for i in range(len(ids)):
                x += self.model_spacing
                spawn_poses[ids[i]] = {'x': x, 'y': y, 'z': z, 'heading': heading}

        rospy.loginfo(f'[MrsDroneSpawner]: Spawn poses returned: {spawn_poses}')
        return spawn_poses
    # #}

    # #{ get_randomized_spawn_poses(self, ids)
    def get_randomized_spawn_poses(self, ids):
        '''
        Creates randomized spawn poses for all vehicles.
        The poses are generated with spacing defined by config param: gazebo_models/spacing
        Height is always set to 0.3

        :param ids: a list of ints containing unique vehicle IDs
        :return: a dict in format {id: {'x': pos_x, 'y', pos_y, 'z': pos_z, 'heading': pos_heading}}
        '''
        spawn_poses = {}

        circle_diameter = 0.0
        total_positions_in_current_circle = 0;
        angle_increment = 0;
        remaining_positions_in_current_circle = 1;
        circle_perimeter= math.pi * circle_diameter
        random_angle_offset = 0
        random_x_offset = round(random.uniform(-self.model_spacing, self.model_spacing), 2)
        random_y_offset = round(random.uniform(-self.model_spacing, self.model_spacing), 2)

        for ID in ids:
            if remaining_positions_in_current_circle == 0:
                circle_diameter = circle_diameter + self.model_spacing
                circle_perimeter= math.pi*circle_diameter
                total_positions_in_current_circle = math.floor(circle_perimeter / self.model_spacing)
                remaining_positions_in_current_circle = total_positions_in_current_circle
                angle_increment = (math.pi * 2) / total_positions_in_current_circle
                random_angle_offset = round(random.uniform(-math.pi,math.pi), 2)

            x = round(math.sin(angle_increment * remaining_positions_in_current_circle + random_angle_offset) * circle_diameter, 2) + random_x_offset
            y = round(math.cos(angle_increment * remaining_positions_in_current_circle + random_angle_offset) * circle_diameter, 2) + random_y_offset
            z = 0.3
            heading = round(random.uniform(-math.pi,math.pi), 2)
            remaining_positions_in_current_circle = remaining_positions_in_current_circle - 1
            spawn_poses[ID] = {'x': x, 'y': y, 'z': z, 'heading': heading}

        rospy.loginfo(f'[MrsDroneSpawner]: Spawn poses returned: {spawn_poses}')
        return spawn_poses
    # #}

    # #{ get_jinja_params_for_one_robot(self, params_dict, index, ID)
    def get_jinja_params_for_one_robot(self, params_dict, index, ID):
        '''Makes a deep copy of params dict, removes entries of other robots, assigns mavlink ports
        :param index: index of the robot in the input sequence
        :param ID: ID of the robot, should match the value in params_dict['ids'][index]
        :return: a dict of params to be used in rendering the jinja template
        '''

        robot_params = copy.deepcopy(params_dict)
        robot_params['ID'] = ID
        robot_params['name'] = params_dict['names'][index]
        robot_params['spawn_pose'] = params_dict['spawn_poses'][ID]

        del robot_params['names']
        del robot_params['help']
        del robot_params['ids']
        del robot_params['spawn_poses']

        robot_params['mavlink_config'] = self.get_mavlink_config_for_robot(ID)

        return robot_params
    # #}

    # #{ get_mavlink_config_for_robot(self, ID)
    def get_mavlink_config_for_robot(self, ID):
        '''Creates a mavlink port configuration based on default values offset by ID

        NOTE: The offsets have to match values assigned in px4-rc.* files located in package_root/ROMFS/px4fmu_common/init.d-posix!!

        '''
        mavlink_config = {}
        udp_offboard_port_remote = self.vehicle_base_port + (4 * ID) + 2
        udp_offboard_port_local = self.vehicle_base_port + (4 * ID) + 1
        mavlink_config['udp_offboard_port_remote'] = udp_offboard_port_remote
        mavlink_config['udp_offboard_port_local'] = udp_offboard_port_local
        mavlink_config['mavlink_tcp_port'] = self.mavlink_tcp_base_port + ID
        mavlink_config['mavlink_udp_port'] = self.mavlink_udp_base_port + ID
        mavlink_config['qgc_udp_port'] = self.qgc_udp_port
        mavlink_config['sdk_udp_port'] = self.sdk_udp_port
        mavlink_config['send_vision_estimation'] = int(self.send_vision_estimation)
        mavlink_config['send_odometry'] = int(self.send_odometry)
        mavlink_config['enable_lockstep'] = int(self.enable_lockstep)
        mavlink_config['use_tcp'] = int(self.use_tcp)
        mavlink_config['fcu_url'] = f'udp://127.0.0.1:{udp_offboard_port_remote}@127.0.0.1:{udp_offboard_port_local}'

        return mavlink_config
    # #}

    # --------------------------------------------------------------
    # |                   Launching subprocesses                   |
    # --------------------------------------------------------------

    # #{ launch_px4_firmware(self, robot_params)
    def launch_px4_firmware(self, robot_params):
        name = robot_params['name']
        rospy.loginfo(f'[MrsDroneSpawner]: Launching PX4 firmware for {name}')
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        # find PX4 ROMFS path
        package_name = self.jinja_templates[robot_params['model']].package_name
        package_path = self.rospack.get_path(package_name)
        romfs_path = os.path.join(str(package_path), 'ROMFS')
        if not os.path.exists(romfs_path) or not os.path.isdir(romfs_path):
            rospy.logerr(f'[MrsDroneSpawner]: Could not start PX4 firmware for {name}. ROMFS folder not found')
            raise CouldNotLaunch('ROMFS folder not found')

        roslaunch_args = [
            'ID:=' + str(robot_params['ID']),
            'PX4_SIM_MODEL:=' + str(robot_params['model']),
            'ROMFS_PATH:=' + str(romfs_path)
        ]
        roslaunch_sequence = [(self.px4_fimrware_launch_path, roslaunch_args)]

        firmware_process = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_sequence)

        try:
            firmware_process.start()
        except:
            rospy.logerr(f'[MrsDroneSpawner]: Could not start PX4 firmware for {name}. Node failed to launch')
            raise CouldNotLaunch('PX4 failed to launch')

        rospy.loginfo(f'[MrsDroneSpawner]: PX4 firmware for {name} launched')

        return firmware_process
    # #}

    # #{ launch_mavros(self, robot_params)
    def launch_mavros(self, robot_params):
        name = robot_params['name']
        rospy.loginfo(f'[MrsDroneSpawner]: Launching mavros for {name}')

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        roslaunch_args = [
                'ID:=' + str(robot_params['ID']),
                'fcu_url:=' + str(robot_params['mavlink_config']['fcu_url']),
                'vehicle:=' + str(robot_params['model']) # TODO: rename this to PX4_SIM_MODEL in the mavros launch file?
        ]

        roslaunch_sequence = [(self.mavros_launch_path, roslaunch_args)]
        mavros_process = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_sequence)

        try:
            mavros_process.start()
        except:
            rospy.logerr(f'[MrsDroneSpawner]: Could not start mavros for {name}. Node failed to launch')
            raise CouldNotLaunch('Mavros failed to launch')

        rospy.loginfo(f'[MrsDroneSpawner]: Mavros for {name} launched')

        return mavros_process
    # #}

    # #{ spawn_gazebo_model(self, robot_params)
    def spawn_gazebo_model(self, robot_params):
        name = robot_params['name']
        sdf_content = self.render(robot_params)

        if sdf_content is None:
            raise CouldNotLaunch('Template did not render')

        if self.save_sdf_files:
            fd, filepath = tempfile.mkstemp(prefix='mrs_drone_spawner_' + datetime.datetime.now().strftime("%Y_%m_%d__%H_%M_%S_"), suffix='_' + str(robot_params['model']) + '_' + str(name) + '.sdf')
            with os.fdopen(fd, 'w') as output_file:
                output_file.write(sdf_content)
                rospy.loginfo(f'[MrsDroneSpawner]: Model for {name} written to {filepath}')

        spawn_pose = Pose()
        spawn_pose.position.x = robot_params['spawn_pose']['x']
        spawn_pose.position.y = robot_params['spawn_pose']['y']
        spawn_pose.position.z = robot_params['spawn_pose']['z']
        spawn_pose.orientation.w = math.cos(robot_params['spawn_pose']['heading'] / 2.0)
        spawn_pose.orientation.x = 0
        spawn_pose.orientation.y = 0
        spawn_pose.orientation.z = math.sin(robot_params['spawn_pose']['heading'] / 2.0)

        rospy.loginfo(f'[MrsDroneSpawner]: Spawning gazebo model for {name}')
        try:
            response = self.gazebo_spawn_proxy(name, sdf_content, name, spawn_pose, "")
            if not response.success:
                rospy.logerr(f'MrsDroneSpawner]: While calling {self.gazebo_spawn_proxy.resolved_name}: {response.status_message}')
                raise CouldNotLaunch(response.status_message)
        except rospy.service.ServiceException as e:
                rospy.logerr(f'[MrsDroneSpawner]: While calling {self.gazebo_spawn_proxy.resolved_name}: {e}')
                raise CouldNotLaunch(e)

        return True
    # #}

    # #{ delete_gazebo_model(self, name)
    def delete_gazebo_model(self, name):
        rospy.loginfo(f'[MrsDroneSpawner]: Deleting gazebo model {name}')
        try:
            response = self.gazebo_delete_proxy(name)
            if not response.success:
                rospy.logerr(f'MrsDroneSpawner]: While calling {self.gazebo_delete_proxy.resolved_name}: {response.status_message}')
                return
        except rospy.service.ServiceException as e:
                rospy.logerr(f'[MrsDroneSpawner]: While calling {self.gazebo_delete_proxy.resolved_name}: {e}')
                return

        rospy.loginfo(f'[MrsDroneSpawner]: Model {name} deleted')
    # #}

if __name__ == '__main__':

    print('[INFO] [MrsDroneSpawner]: Starting')

    verbose = 'verbose' in sys.argv

    atexit.register(exit_handler)

    try:
        spawner = MrsDroneSpawner(verbose)
    except rospy.ROSInterruptException:
        pass
