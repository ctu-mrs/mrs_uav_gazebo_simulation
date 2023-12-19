class ComponentWrapper:

    def __init__(self, keyword, description, default_args):
        '''
        A wrapper for params defined in the jinja template that are recognized and used by the mrs_drone_spawner

        :param keyword: string used to activate the component by using '--keyword' as spawner argument

        :param description: string used to display help for a component in a human-readable form

        :param default_args: a list of dict of configurable internal variables, user can override those by
        adding them as args after the '--keyword' without the '--'
        '''
        self.keyword = keyword
        self.description = description
        self.default_args = default_args
