import math
import jinja2
import jinja2.meta
import sys
import os.path

from inspect import getmembers, isclass

from component_wrapper import ComponentWrapper
from template_wrapper import TemplateWrapper

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

# #{ get_spawner_components_from_template
def get_spawner_components_from_template(jinja_env, template):
    '''
    Builds a dict of spawner-compatible macros in a given template and their corresponding ComponentWrapper objects
    Does NOT check for macros imported from other templates
    :return a dict in format {macro name: component_wrapper.ComponentWrapper}
    '''
    with open(template.filename, 'r') as f:
        template_source = f.read()
        preprocessed_template = template_source.replace('\n', '')
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

# #{ get_all_templates
def get_all_templates(jinja_env):
    '''
    Get all templates loaded by the given jinja environment
    :returns a list of tuples, consisting of (str_name, jinja2.Template)
    '''
    template_names = jinja_env.list_templates(filter_func=filter_templates)
    templates = []
    for i, full_name in enumerate(template_names):
        print(f'\t({i+1}/{len(template_names)}): {full_name}')
        template_name = full_name.split(os.path.sep)[-1][:-(len(TEMPLATE_SUFFIX))]
        templates.append((template_name, jinja_env.get_template(full_name)))
    return templates
# #}

# #{ get_template_imports
def get_template_imports(jinja_env, template):
    with open(template.filename, 'r') as f:
        template_source = f.read()
        preprocessed_template = template_source.replace('\n', '')
        parsed_template = jinja_env.parse(preprocessed_template)
        import_names = [node.template.value for node in parsed_template.find_all(jinja2.nodes.Import)]
        imported_templates = []
        for i in import_names:
            template = jinja_env.get_template(i)
            imported_templates.append(template)
        return imported_templates
# #}

# #{ get_all_available_components
def get_all_available_components(template_wrapper, all_components):
    all_components.update(template_wrapper.components)
    for i in template_wrapper.imported_templates:
        try:
            all_components.update(get_all_available_components(i, all_components))
        except RecursionError as err:
            raise RecursionError(f'Cyclic import detected in file {template_wrapper.jinja_template.filename}. Fix your templates')
    return all_components
# #}

# #{ build_template_database
def build_template_database(jinja_env):

    template_wrappers = {}

    print('Loading all templates')
    all_templates = get_all_templates(jinja_env)
    for name, template in all_templates:
        imports = get_template_imports(jinja_env, template)
        components = get_spawner_components_from_template(jinja_env, template)
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
        wrapper.components = get_all_available_components(wrapper, {})
        sys.setrecursionlimit(prev_limit)

    print('Template database built')

    return template_wrappers
# #}
