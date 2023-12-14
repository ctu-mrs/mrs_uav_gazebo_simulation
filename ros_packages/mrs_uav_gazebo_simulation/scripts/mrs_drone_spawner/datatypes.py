class SpawnerComponent:
    
    def __init__(self, keyword, description, default_args):
        self.keyword = keyword
        self.description = description
        self.default_args = default_args 

class TemplateWrapper:

    def __init__(self, jinja_template, imported_templates, components):
        self.jinja_template = jinja_template
        self.imported_templates = imported_templates
        self.components = components


    def __eq__(self, other):
        if isinstance(other, TemplateWrapper):
            return self.jinja_template.filename == other.jinja_template.filename
        return False
