from setuptools import setup

#RUN pip install -e .  

setup(name='EnvCreator',
      version='0.1.0',
      author='Kyle Morgenstein',
      author_email='kylem@utexas.edu',
      install_requires=[
            'numpy',
            'matplotlib',
            'Image',
            ] #And any other dependencies required
)