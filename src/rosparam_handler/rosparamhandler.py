import inspect

import rospy


class RosparamHandler(object):

    def __init__(self, namespace=""):
        self.namespace = namespace
        self.description = dict()
        self.defaults = dict()

    def get_attributes(self):
        attributes = inspect.getmembers(self, lambda a: not (inspect.isroutine(a)))
        return dict([a for a in attributes
                if not (a[0].startswith('__') and a[0].endswith('__') or a[0] == "description" or a[0] == "namespace" or a[0] == "defaults")])

    def from_param_server(self):
        """
        Reads and initializes parameters with values from parameter server.
        Called automatically at initialization.
        """
        attributes = self.get_attributes()
        for name, value in attributes.iteritems():
            config = next((item for item in self.description if item["name"] == name), None)
            if config['constant']:
                self.test_const_param(name)
                continue
            param_name = name
            if self.namespace:
                param_name = "/" + self.namespace + "/" + param_name
            param_value = self.get_param(param_name, config)
            setattr(self, param_name, param_value)

    def to_param_server(self):
        """
        Sets parameters with current values on the parameter server.
        """
        attributes = self.get_attributes()
        for param_name, param_value in attributes.iteritems():
            config = next((item for item in self.description if item["name"] == param_name), None)
            if not config['constant']:
                full_name = "/" + param_name if config['global_scope'] else "~" + param_name
                if self.namespace:
                    full_name = self.namespace + "/" + full_name
                rospy.set_param(full_name, param_value)

    def from_config(self, config):
        """
        Reads parameter from a dynamic_reconfigure config file.
        Should be called in the callback of dynamic_reconfigure.
        :param config: config object from dynamic reconfigure
        """
        attributes = self.get_attributes()
        for k, v in config.iteritems():
            # handle reserved name groups
            if k == "groups":
                continue
            if k not in attributes:
                raise TypeError("Element {} of config is not part of parameters.".format(k))
            setattr(self, k, v)

    @staticmethod
    def test_const_param(param_name):
        if rospy.has_param("~" + param_name):
            rospy.logwarn(
                "Parameter {} was set on the parameter server even though it was defined to be constant.".format(
                    param_name))

    @staticmethod
    def get_param(param_name, config):
        def get_type(type_string):
            if type_string == 'std::string':
                return str
            elif type_string == 'int':
                return int
            elif type_string == 'bool':
                return bool
            elif type_string == 'float' or type_string == 'double':
                return float
            else:
                raise ValueError()

        full_name = "/" + param_name if config['global_scope'] else "~" + param_name
        try:
            val = rospy.get_param(full_name)
        except KeyError:
            if config['default'] is None:
                raise KeyError("Parameter {} is neither set on the parameter server nor "
                               "has it a default value".format(param_name))
            rospy.loginfo("Parameter {} is not yet set. Setting default value".format(param_name))
            rospy.set_param(full_name, config['default'])
            val = config['default']

        # test whether type is correct
        try:
            if config['is_vector']:
                val = list(val)
                config_type = config['type']
                val_type = get_type(config_type[config_type.find("<") + 1:config_type.find(">")])
                val = [val_type(v) for v in val]
            elif config['is_map']:
                val = dict(val)
                config_type = config['type']
                key_type = get_type(config_type[config_type.find("<") + 1:config_type.find(",")])
                val_type = get_type(config_type[config_type.find(",") + 1:config_type.find(">")])
                val = {key_type(key): val_type(v) for key, v in val.items()}
            else:
                val = get_type(config['type'])(val)
        except ValueError:
            rospy.logerr(
                "Parameter {} is set, but has a different type. Using default value instead.".format(param_name))
            val = config['default']
        # test bounds
        if config['min'] is not None:
            if config['is_vector']:
                if min(val) < config['min']:
                    rospy.logwarn(
                        "Some values in {} for {} are smaller than minimal allowed value. "
                        "Correcting them to min={}".format(val, param_name, config['min']))
                    val = [v if v > config['min'] else config['min'] for v in val]
            elif config['is_map']:
                if min(val.values()) < config['min']:
                    rospy.logwarn(
                        "Some values in {} for {} are smaller than minimal allowed value. "
                        "Correcting them to min={}".format(val, param_name, config['min']))
                    val = {k: (v if v > config['min'] else config['min']) for k, v in val.items()}
            elif val < config['min']:
                rospy.logwarn(
                    "Value of {} for {} is smaller than minimal allowed value. "
                    "Correcting value to min={}".format(val, param_name, config['min']))
                val = config['min']

        if config['max'] is not None:
            if config['is_vector']:
                if max(val) > config['max']:
                    rospy.logwarn(
                        "Some values in {} for {} are greater than maximal allowed value. "
                        "Correcting them to max={}".format(val, param_name, config['max']))
                    val = [v if v < config['max'] else config['max'] for v in val]
            elif config['is_map']:
                if max(val.values()) > config['max']:
                    rospy.logwarn(
                        "Some values in {} for {} are greater than maximal allowed value. "
                        "Correcting them to max={}".format(val, param_name, config['max']))
                    val = {k: (v if v < config['max'] else config['max']) for k, v in val.items()}
            elif val > config['max']:
                rospy.logwarn(
                    "Value of {} for {} is greater than maximal allowed value. "
                    "Correcting value to max={}".format(val, param_name, config['max']))
                val = config['max']
        return val
