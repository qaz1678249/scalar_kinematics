from setuptools import setup, find_packages
import codecs
import os.path


def read(rel_path):
    here = os.path.abspath(os.path.dirname(__file__))
    with codecs.open(os.path.join(here, rel_path), 'r') as fp:
        return fp.read()


with open('requirements.txt') as f:
    requirements = f.read().splitlines()

setup(
    name='scaler_kin',
    version='0.5.0',
    description='Scaler kinematics for 3DoF, 4DoF and 6DoF and camera arm kinematics',
    author='Yusuke Tanaka, Feng Xu,',
    license='LGPLv3',
    packages=find_packages(include=['scaler_kin', 'scaler_kin.*', 'camera_arm_kin', 'camera_arm_kin.*']),
    install_requires=requirements,
    classifiers=[
        'Development Status :: 4 - Beta',
        'Framework :: Robot Framework'
        'Intended Audience :: Developers',
        'Intended Audience :: Education',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: GNU Lesser General Public License v3 (LGPLv3)',
        'Programming Language :: Python :: 3',
    ],
)

