from morse.builder import *

def add_package(package_type, package_name, x, y, z):
    #TODO adjust blender file path to package type
    package = PassiveObject(package_type + '.blend', package_type)
    package.setgraspable()
    package.name = package_name
    package.translate(x, y, z)

def frange(start, stop, step):
    i = start
    while i <= stop:
        yield i
        i += step

def add_packages(warehouse_config, package_configs):
    number = len(warehouse_config['trays'])

    package_length = 0.28
    height = 3
    p_pos = warehouse_config['package_pool']['location']
    p_area = warehouse_config['package_pool']['relative_stacking_area']
    x_positions = list(frange(p_pos['x'] + p_area['x1'], p_pos['x'] + p_area['x2'], package_length))
    y_positions = list(frange(p_pos['y'] + p_area['y1'], p_pos['y'] + p_area['y2'], package_length))

    configs = list(package_configs.keys())
    index = 0
    package_count = 0
    prefix = 'pkg'
    for x in x_positions:
        for y in y_positions:
            #z = 0.125
            z = p_area['z']
            for i in range(height):
                if package_count >= number and index + 1 < len(configs):
                    index = index + 1
                    package_count = 0
                if package_count < number:
                    type_str = prefix + configs[index]
                    add_package(type_str, type_str + '_' + str(package_count + 1), x, y, z)
                    package_count = package_count + 1
                    z = z + package_length
