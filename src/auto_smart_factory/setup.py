from distutils.core import setup

setup(
        version='0.0.1',
        #scripts=['bin/myscript'],
        packages=['package_generator', 'roadmap_generator'],
        package_dir={'': 'src'}
)
