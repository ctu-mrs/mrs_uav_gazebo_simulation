import math
import jinja2
import jinja2.meta
from datatypes import SpawnerComponent

from inspect import getmembers, isclass

TEMPLATE_SUFFIX = '.sdf.jinja'

# #{ configure_jinja2_environment
def configure_jinja2_environment(resource_paths):
    '''Create a jinja2 environment and setup its variables'''
    env = jinja2.Environment(
        loader=jinja2.FileSystemLoader(resource_paths),
        autoescape=None
    )
    # Allows use of math module directly in the templates
    env.globals['math'] = math

    return env
# #}

# #{ filter_templates
def filter_templates(template_name):
    '''Comparator used to load files with given suffix'''
    return template_name.endswith(TEMPLATE_SUFFIX)
# #}

# #{ get_template_filepath
def get_template_filepath(model_name, jinja_env):
    '''Return a filepath to a template with given name (without suffix)
    or None if a matching template is not found
    '''
    for n in jinja_env.list_templates(filter_func=filter_templates):
        if model_name in n:
            template_filepath = jinja_env.get_template(n).filename
            return template_filepath
# #}

# #{ get_macros_from_template
def get_macros_from_template(jinja_env, template_path):
    # TODO
    # this needs to recursively check all imported sub-templates, and load macros from them as well
    # TODO
    template_source = jinja_env.loader.get_source(jinja_env, template_path)
    preprocessed_template = template_source[0].replace('\n', '')
    parsed_template = jinja_env.parse(preprocessed_template)
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
                else:
                    print(f'Unsupported param type {type(elem.node)}')
            if isinstance(elem, jinja2.nodes.Assign) and elem.target.name == 'spawner_keyword':
                spawner_keyword = elem.node.value
        if spawner_keyword is not None:
            spawner_components[node.name] = SpawnerComponent(spawner_keyword, spawner_description, spawner_default_args)
    return spawner_components
# #}

# #{ get_imported_templates
def get_imported_templates(jinja_env, template_path):
    template_source = jinja_env.loader.get_source(jinja_env, template_path)
    preprocessed_template = template_source[0].replace('\n', '')
    parsed_template = jinja_env.parse(preprocessed_template)
    imports = [node.template.value for node in parsed_template.find_all(jinja2.nodes.Import)]
    return imports
# #}

# #{ get_all_templates
def get_all_templates(jinja_env):
    '''Return all templates loaded by the given jinja environment'''
    return jinja_env.list_templates(filter_func=filter_templates)
# #}

# #{ get_all_params
def get_all_params(model_name, jinja_env, sort=True):
    '''Return a list of all params that can be used with a template of given name'''
    filepath = get_template_filepath(model_name, jinja_env)
    if filepath is None:
        return None
    with open(filepath, 'r') as f:
        content = f.read()
        parsed_content = jinja_env.parse(content)
        variable_names = list(jinja2.meta.find_undeclared_variables(parsed_content))
        if sort:
            variable_names = sorted(variable_names)
        return variable_names
# #}

