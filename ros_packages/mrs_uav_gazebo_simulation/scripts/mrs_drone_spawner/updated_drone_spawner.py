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


TEMPLATE_SUFFIX = '.sdf.jinja'
glob_running_processes = []

# #{ filter_templates(template_name)
def filter_templates(template_name):
    '''Comparator used to load files with given suffix'''
    return template_name.endswith(TEMPLATE_SUFFIX)
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
        rospack = rospkg.RosPack()

        resource_paths = []
        resource_paths.append(rospack.get_path('mrs_uav_gazebo_simulation'))
        resource_paths.append(rospack.get_path('external_gazebo_models'))

        self.jinja_env = self.configure_jinja2_environment(resource_paths)

        try:
            self.jinja_templates = self.build_template_database()
        except RecursionError as err:
            print(f'[ERROR] [MrsDroneSpawner]: {err}')
            raise rospy.ROSInterruptException

        print('Jinja templates available:')
        print([name for name in self.jinja_templates.keys()])
        print('------------------------')

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
        template_names = self.jinja_env.list_templates(filter_func=filter_templates)
        templates = []
        for i, full_name in enumerate(template_names):
            print(f'\t({i+1}/{len(template_names)}): {full_name}')
            template_name = full_name.split(os.path.sep)[-1][:-(len(TEMPLATE_SUFFIX))]
            templates.append((template_name, self.jinja_env.get_template(full_name)))
        return templates
    # #}

    # #{ get_template_imports(self, template)
    def get_template_imports(self, template):
        with open(template.filename, 'r') as f:
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
                            print(f'Unsupported param type {type(elem.node)}')
                    if isinstance(elem, jinja2.nodes.Assign) and elem.target.name == 'spawner_keyword':
                        spawner_keyword = elem.node.value
                if spawner_keyword is not None:
                    spawner_components[node.name] = ComponentWrapper(spawner_keyword, spawner_description, spawner_default_args)
            return spawner_components
    # #}

    # #{ get_all_available_components(self, template_wrapper, all_components)
    def get_all_available_components(self, template_wrapper, all_components):
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

        template_wrappers = {}

        print('Loading all templates')
        all_templates = self.get_all_templates()
        for name, template in all_templates:
            imports = self.get_template_imports(template)
            components = self.get_spawner_components_from_template(template)
            wrapper = TemplateWrapper(template, imports, components)
            template_wrappers[name] = wrapper

        print('Reindexing imported templates')
        for name, wrapper in template_wrappers.items():
            for i, it in enumerate(wrapper.imported_templates):
                if not isinstance(it, TemplateWrapper):
                    for ww in template_wrappers.values():
                        if ww.jinja_template == it:
                            wrapper.imported_templates[i] = ww

        print('Adding available components from dependencies')
        for _, wrapper in template_wrappers.items():
            prev_limit = sys.getrecursionlimit()
            sys.setrecursionlimit(len(template_wrappers) + 1)
            wrapper.components = self.get_all_available_components(wrapper, {})
            sys.setrecursionlimit(prev_limit)

        print('Template database built')

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

        # parse out the keywords starting with '--'
        pattern = re.compile(r"(--\S*)")
        substrings = [m.strip() for m in re.split(pattern, input_str) if len(m.strip()) > 0]

        input_dict = {}

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

        # extract model name
        for k in input_dict.keys():
            if k in self.jinja_templates.keys():
                input_dict['model'] = str(k)
                del input_dict[k]
                break

        if 'model' not in input_dict.keys():
            raise ValueError('Model name not defined. Specify a model with a keyword (e.g. "--x500")')
        print('Model name not defined!')

        # look for --help
        ## TODO implement help

        return input_dict
    # #}

    # #{ parse_string_to_objects(self, input_str)
    def parse_string_to_objects(self, input_str):
        '''
        Attempt to convert input_str into a dictionary or a list
        Convert numerals into number datatypes whenever possible
        '''
        input_str = input_str.strip()
        try:
            params = ast.literal_eval(input_str)
            if isinstance(params, dict):
                return params
        except (SyntaxError, ValueError) as e:
            # print(f"Error converting string {input_str} to dictionary: {e}")
            pass

        params = []
        for s in input_str.split():
            if len(s) > 0:
                try:
                    # try converting string to numbers
                    params.append(ast.literal_eval(s))
                except ValueError:
                    # leave non-numbers as string
                    params.append(s)

        if isinstance(params, list):
            return params

        return None
    # #}

    # --------------------------------------------------------------
    # |                           Testing                          |
    # --------------------------------------------------------------

    # #{ get_help_for_model(self, model_name)
    def get_help_for_model(self, model_name):
        print(f'Getting help for model {model_name}')
        response = ''
        try:
            template_wrapper = self.jinja_templates[model_name]
            print(f'Template loaded with {len(template_wrapper.components)} components')
        except ValueError:
            return f"Template for model {model_name} not found"

        for name, component in template_wrapper.components.items():
            response += f'{component.keyword}\n\tDescription: {component.description}\n\tDefault args: {component.default_args}\n\n'

        return response
    # #} end get_help_for_model

    # #{ render(self, model_name, output)
    def render(self, model_name, output, spawner_args):
        print('Rendering model', model_name)
        for template_name, template_wrapper in self.jinja_templates.items():
            if model_name == template_name:
                print('Using template', template_name)
                # spawner_args = {
                #     "enable_component_with_args": [0.01, 0.38],
                #     "enable_component_with_args_as_dict": {'roll': 0},
                #     # "enable_second_order_component": None,
                #     # "enable_third_order_component": None,
                # }

                params = {
                    "spawner_args": spawner_args
                    #     "name": "uav1",
                    #     "namespace": "uav1",
                    #     "enable_rangefinder": True,
                    #     "enable_component": True,
                    #     "spawner_args": {'roll': 0.1, 'pitch': 0.1}
                }
                context = template_wrapper.jinja_template.new_context(params)
                rendered_template = template_wrapper.jinja_template.render(context)
                root = xml.dom.minidom.parseString(rendered_template)
                ugly_xml = root.toprettyxml(indent='  ')
                # Remove empty lines
                pretty_xml = "\n".join(line for line in ugly_xml.split("\n") if line.strip())
                with open(output, 'w') as f:
                    f.write(pretty_xml)
                    print('Rendered model written to', output)
                    return
    # #} end render

if __name__ == '__main__':

    print('[INFO] [MrsDroneSpawner]: Starting')

    show_help = 'no_help' not in sys.argv
    verbose = 'verbose' in sys.argv

    atexit.register(exit_handler)

    try:
        spawner = MrsDroneSpawner(show_help, verbose)

        # help_response = spawner.get_help_for_model('dummy')
        # print(help_response)

        # spawner_args = spawner.parse_input_params('--enable_component --enable_component_with_args 14')
        # print(spawner_args)
        # spawner_args = spawner.parse_user_input('--enable_component')
        # spawner.parse_as_dict('{"aaa": "bbb", "ccc": 22, "ddd": {"u":"v", "w":1}, "eee": None}')

        # spawner_args = spawner.parse_user_input('2 10 --enable_component_with_args 0.01 0.38 1E10 --enable_component_with_args_as_dict {"roll": 0, "pitch": 0, "aaa": "bbbb"} --enable-noargs')
        spawner_args = spawner.parse_user_input('2 10 --enable_component_with_args 0.01 0.38 1E10 --enable_component_with_args_as_dict {"roll": 0, "pitch": 0, "aaa": "bbbb"} --enable-noargs --f450')

        for kw, args in spawner_args.items():
            print(kw)
            print('\t', args, type(args))
            if isinstance(args, dict):
                for e in args.values():
                    print('\t\t', e, type(e))
            elif isinstance(args, list):
                for e in args:
                    print('\t\t', e, type(e))
            else:
                print('\t\t', args)

        # # params = jinja_utils.get_all_params('x555', spawner.jinja_env)
        # # params = jinja_utils.get_all_params('f400', spawner.jinja_env)
        # # print(params)
        # # spawner.render('x555', '/home/mrs/devel_workspace/src/external_gazebo_models/models/x555/sdf/x555.sdf')
        # # spawner.render('f400', '/home/mrs/devel_workspace/src/external_gazebo_models/models/f400/sdf/f400.sdf')

        spawner.render('dummy', '/home/mrs/devel_workspace/src/external_gazebo_models/dummy.sdf', spawner_args)
    except rospy.ROSInterruptException:
        pass
