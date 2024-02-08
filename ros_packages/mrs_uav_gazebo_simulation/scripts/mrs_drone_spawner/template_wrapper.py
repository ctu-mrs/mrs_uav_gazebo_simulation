class TemplateWrapper:

    def __init__(self, jinja_template, imported_templates, components, package_name):
        '''
        A wrapper for jinja templates for storing variables specific to the mrs_drone_spawner
        :param jinja_template: a jinja2.Template with a parsed content of the template file
        :param imported_templates: a list of references to other jinja2.Template files that are directly imported by this template
        :param components: a dict of modular components in format "{name (string): component (component_wrapper.ComponentWrapper)}",
        :param package_name: name of the parent ROS package that contains this template",
        where name is the name of a jinja macro. Contains all components accessible from this template, including imported ones from other template snippets
        '''
        self.jinja_template = jinja_template
        self.imported_templates = imported_templates
        self.components = components
        self.package_name = package_name

    def __eq__(self, other):
        if isinstance(other, TemplateWrapper):
            return self.jinja_template.filename == other.jinja_template.filename
        return False
