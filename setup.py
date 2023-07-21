from setuptools import setup, find_packages

setup(name='compatible_clf_cbf',
      version='0.0',
      description='Traffic control based on CLF and CBFs',
      url='https://github.com/CaipirUltron/CompatibleCLFCBF',
      author='Matheus Reis',
      license='GPLv3',
      packages=find_packages(include=['spline_cbfs.*']),
      zip_safe=False)