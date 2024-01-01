#!/usr/bin/python3
import ast
import atexit
import jinja2
import jinja2.meta
import math
import os
import re
import rospkg
import rospy
import sys
import xml.dom.minidom

from inspect import getmembers, isfunction

from component_wrapper import ComponentWrapper
from template_wrapper import TemplateWrapper


glob_running_processes = []

# #{ filter_templates(template_name)
def filter_templates(template_name, suffix):
    '''Comparator used to load files with given suffix'''
    return template_name.endswith(suffix)
# #}

def custom_decoder(obj):
    if obj == 'None':
        return None
    return obj

# #{ exit_handler()
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

class MrsDroneSpawner:

    # #{ __init__(self, show_help=False, verbose=False)
    def __init__(self, show_help=False, verbose=False):

        self.verbose = verbose

        rospy.init_node('mrs_drone_spawner', anonymous=True)

        resource_paths = []
        rospack = rospkg.RosPack()
        resource_paths.append(rospack.get_path('mrs_uav_gazebo_simulation'))

        # # #{ load required params
        try:
            self.vehicle_base_port = rospy.get_param('~vehicle_base_port')
            self.mavlink_tcp_base_port = rospy.get_param('~mavlink_tcp_base_port')
            self.mavlink_udp_base_port = rospy.get_param('~mavlink_udp_base_port')
            self.launch_base_port = rospy.get_param('~launch_base_port')
            self.template_suffix = rospy.get_param('~template_suffix')
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
                rpath = rospack.get_path(elem)
            rospy.loginfo(f'[MrsDroneSpawner]: Adding extra resources from {rpath}')
            resource_paths.append(rpath)

        self.jinja_env = self.configure_jinja2_environment(resource_paths)
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
            spawner_keyword = None
            spawner_description= None
            spawner_default_args = None
            spawner_components = {}
            for node in macro_nodes:
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

    # #{ get_all_available_components(self, template_wrapper, all_components)
    def get_all_available_components(self, template_wrapper, all_components):
        '''
        Recursive function to get all spawner-compatible components accessible from template_wrapper
        Includes components in imported sub-templates
        '''
        all_components.update(template_wrapper.components)
        for i in template_wrapper.imported_templates:
            try:
                all_components.update(self.get_all_available_components(i, all_components))
            except RecursionError as err:
                raise RecursionError(f'Cyclic import detected in file {template_wrapper.jinja_template.filename}. Fix your templates')
        return all_components
    # #}

    # #{ build_template_database(self)
    def build_template_database(self):
        '''
        Generate a database of jinja2 templates available to the spawner
        Scans through all folders provided into the jinja2 environment for files with matching target suffix
        Recursively checks templates imported by templates, prevents recursion loops
        Returns a dictionary of TemplateWrapper objects in format {template_name: template_wrapper}
        '''

        template_wrappers = {}

        rospy.loginfo('[MrsDroneSpawner]: Loading all templates')
        all_templates = self.get_all_templates()
        for name, template in all_templates:
            imports = self.get_template_imports(template)
            components = self.get_spawner_components_from_template(template)
            wrapper = TemplateWrapper(template, imports, components)
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
            wrapper.components = self.get_all_available_components(wrapper, {})
            sys.setrecursionlimit(prev_limit)

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
            component args following a keyword (values separated by spaces or a python dictionary)
        '''

        input_dict = {}

        # look for --help
        if '--help' in input_str:
            input_dict['help'] = True
            input_str = input_str.replace('--help', '')
        else:
            input_dict['help'] = False

        # parse out the keywords starting with '--'
        pattern = re.compile(r'(--\S*)')
        substrings = [m.strip() for m in re.split(pattern, input_str) if len(m.strip()) > 0]

        # before the first keyword, there should be only device IDs
        if '--' not in substrings[0]:
            input_dict['ids'] = self.parse_string_to_objects(substrings[0])
        else:
            raise ValueError(f'Expected device IDs, got "{substrings[0]}" instead')

        # pair up keywords with args
        for i in range(1, len(substrings)):

            if substrings[i].startswith('--'):
                input_dict[substrings[i][2:]] = None
                continue
            else:
                input_keys = [*input_dict.keys()]
                if len(input_keys) > 1:
                    input_dict[input_keys[-1]] = self.parse_string_to_objects(substrings[i])

        input_dict['model'] = None
        # attempt to match model to available templates
        for k in input_dict.keys():
            if k in self.jinja_templates.keys():
                input_dict['model'] = str(k)
                del input_dict[k]
                break

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
        try:
            params = ast.literal_eval(input_str)
            if isinstance(params, dict):
                return params

        except (SyntaxError, ValueError) as e:
            # cannot convert input_str to a dict
            pass

        params = []
        for s in input_str.split():
            if len(s) > 0:
                try:
                    # try to convert input_str to numbers
                    params.append(ast.literal_eval(s))
                except ValueError:
                    # leave non-numbers as string
                    params.append(s)

        if isinstance(params, list):
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
        Create a help string by loading all components from a given template in the following format
        Component name
            Description:
            Default args:
        '''
        rospy.loginfo(f'[MrsDroneSpawner]: Getting help for model {model_name}')
        response = f'[MrsDroneSpawner]: Components available for model {model_name}:\n'
        try:
            template_wrapper = self.jinja_templates[model_name]
            rospy.loginfo(f'[MrsDroneSpawner]: Template {template_wrapper.jinja_template.name} loaded with {len(template_wrapper.components)} components')
        except ValueError:
            return f'Template for model {model_name} not found'

        for name, component in template_wrapper.components.items():
            response += f'{component.keyword}\n\tDescription: {component.description}\n\tDefault args: {component.default_args}\n\n'

        return response
    # #} end get_model_help_text

    # #{ get_spawner_help_text(self)
    def get_spawner_help_text(self):
        '''Create a generic help string for the spawner basic use'''

        rospy.loginfo(f'[MrsDroneSpawner]: Getting generic spawner help')
        response = 'Expected input:\n'
        response += '\tdevice ids (integers separated by spaces)\n'
        response += '\tkeywords (specified in jinja components starting with \'--\')\n'
        response += '\tcomponent args following a keyword (values separated by spaces or a python dictionary)\n'
        response += '\tavailable model templates: '
        for model_name in self.jinja_templates.keys():
            response += f'{model_name}, '
        response += '\n'
        return response
    # #}


    # --------------------------------------------------------------
    # |                           Testing                          |
    # --------------------------------------------------------------

    # #{ render(self, model_name, output)
    def render(self, model_name, output, spawner_args):
        '''
        Renders a jinja template into a sdf file
        :param model_name: name of the model, has to match a key in the self.jinja_templates database
        :param output: full filepath to write the output into
        :param spawner_args: spawner args specified by the user, will be passed into the template
        '''
        
        params = {
            "spawner_args": spawner_args
            #     "name": "uav1",
            #     "namespace": "uav1",
            #     "enable_rangefinder": True,
            #     "enable_component": True,
            #     "spawner_args": {'roll': 0.1, 'pitch': 0.1}
        }

        try:
            template_wrapper = self.jinja_templates[model_name]
        except KeyError:
            rospy.logerr(f'[MrsDroneSpawner]: Cannot render model "{model_name}". Template not found!')
            return
 
        rospy.loginfo(f'[MrsDroneSpawner]: Rendering model "{model_name}" using template {template_wrapper.jinja_template.filename}')

        context = template_wrapper.jinja_template.new_context(params)
        rendered_template = template_wrapper.jinja_template.render(context)
        root = xml.dom.minidom.parseString(rendered_template)
        ugly_xml = root.toprettyxml(indent='  ')
        # Remove empty lines
        pretty_xml = '\n'.join(line for line in ugly_xml.split('\n') if line.strip())
        
        with open(output, 'w') as f:
            f.write(pretty_xml)
            rospy.loginfo(f'[MrsDroneSpawner]: Rendered model written to {output}')
            return

    # #} end render

if __name__ == '__main__':

    print('[INFO] [MrsDroneSpawner]: Starting')

    show_help = 'no_help' not in sys.argv
    verbose = 'verbose' in sys.argv

    atexit.register(exit_handler)

    try:
        spawner = MrsDroneSpawner(show_help, verbose)
        spawner_args = spawner.parse_user_input('2 10 --enable_component_with_args 0.01 0.38 1E10 --enable_component_with_args_as_dict {"roll": 0, "pitch": 22, "aaa": "bbbb"} --enable-noargs --dummy')

        if spawner.get_help_text(spawner_args) is not None:
            rospy.loginfo(help_text)
        elif spawner_args['model'] is not None:
            spawner.render(spawner_args['model'], '/home/mrs/devel_workspace/src/external_gazebo_models/dummy.sdf', spawner_args)
        else:
            rospy.logerr('[MrsDroneSpawner]: Model not specified')
            raise rospy.ROSInterruptException
    except rospy.ROSInterruptException:
        pass
