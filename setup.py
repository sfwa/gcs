from setuptools import setup

setup(
    name='uav',
    version='1.0dev1',
    packages=['gcs', 'gcs.radio', 'gcs.plog', 'gcs.vectors', 'gcs.mission'],
    scripts=['bin/gcs-server'],
    package_data={
        'gcs': [
            'static/css/*.css', 'static/js/*.js', 'static/favicon.ico',
            'templates/*.html'
        ]
    },
    install_requires=['tornado>=4.0', 'cobs', 'pyserial', 'enum34'],

    # meta
    author='Ben Dyer',
    author_email='ben_dyer@mac.com',
    url='https://github.com/sfwa/gcs',
    license='LICENSE',
    description='UAV software',
    long_description=open('README.md').read(),
)
