#!/usr/bin/env python
#
#   Copyright (C) 2021 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.


# This script checks for dependencies of pypilot, and installs them.
# Hardened for Bookworm / Python 3.12+ (PEP 668 externally-managed-environment).

import os, sys

success = True

# Detect PEP 668 "externally managed" environment (Bookworm / Python 3.12+).
# When true we must pass --break-system-packages to pip as a last resort.
def _pip_extra_flags():
    marker = os.path.join(sys.prefix, 'lib',
                          'python%d.%d' % sys.version_info[:2],
                          'EXTERNALLY-MANAGED')
    if os.path.exists(marker):
        return '--break-system-packages'
    return ''

class dep(object):
    def __init__(self, name):
        self.name = name

class py_dep(dep):
    def __init__(self, name):
        super(py_dep, self).__init__(name)

    def test(self):
        # Maps logical dependency names to actual importable module names.
        import_remap = {'pil': 'PIL',
                        'gevent-websocket': 'geventwebsocket',
                        'flask-socketio': 'flask_socketio',
                        'flask-babel': 'flask_babel',
                        'python-socketio': 'socketio',
                        'opengl': 'OpenGL',
                        'importlib_metadata': 'importlib.metadata'}

        name = import_remap.get(self.name, self.name)
        try:
            __import__(name)
        except Exception:
            print('failed to import', self.name)
            return False
        return True

    def install(self):
        # Maps logical names to correct apt package names where they differ
        # from the default python3-{name} pattern.
        apt_remap = {'pil':              'python3-pil',
                     'gevent-websocket': 'python3-gevent',
                     'flask-socketio':   'python3-flask-socketio',
                     'python-socketio':  'python3-socketio',
                     'opengl':           'python3-opengl',
                     'importlib_metadata': 'python3-importlib-metadata',
                     'websocket':        'python3-websocket'}

        # pip package names that differ from the logical name
        pip_remap = {'pil':           'pillow',
                     'flask-socketio': 'flask-socketio',
                     'python-socketio': 'python-socketio'}

        apt_name = apt_remap.get(self.name, 'python3-' + self.name)
        ret = os.system('sudo apt install -y ' + apt_name)

        if ret:
            pip_name = pip_remap.get(self.name, self.name)
            extra = _pip_extra_flags()
            print('failed to install via apt, trying with pip', pip_name)
            ret = os.system('sudo python3 -m pip install %s %s' % (extra, pip_name))

            if ret:
                print('failed to install dependency', pip_name)
                return False
        return True

class sys_dep(dep):
    def __init__(self, name):
        super(sys_dep, self).__init__(name)

    def test(self):
        try:
            import apt
            return apt.Cache()[self.name].is_installed
        except Exception as e:
            print("Failed to detect package", self.name, i)
        return False

    def install(self):
        ret = os.system('sudo apt install -y ' + self.name)
        if ret:
            return False
        return True

class wiringpi_dep(dep):
    def __init__(self, name):
        super(wiringpi_dep, self).__init__(name)

    def install(self):
        os.system('git clone https://github.com/wiringPi/wiringPi')
        if os.system('cd wiringPi; ./build'):
            return False
        return True
        
    def test(self, check=False):
        try:
            f = open('/sys/firmware/devicetree/base/model')
            pi = 'Raspberry Pi' in f.readline()
            f.close()
            if pi:
                print('detected', pi)
                try:
                    import subprocess
                    output = subprocess.check_output(['gpio', '-v'])
                    version = output.split(b'\n')[0].strip(b'gpio version: ')
                    return float(version) >= 2.6
                except Exception as e:
                    print('failed to run gpio command!', e)
                    return False
                return super(wiringpi_dep, self).test()
        except:
            pass
        return True

class RTIMULIB2_dep(dep):
    def __init__(self):
        super(RTIMULIB2_dep, self).__init__('RTIMULIB2')

    def test(self, check=False):
        try:
            import RTIMU
        except Exception:
            print('failed to import', self.name)
            return False

        # Use stdlib importlib.metadata (Python 3.8+) with fallback to pip package.
        try:
            from importlib.metadata import version as _ver
        except ImportError:
            from importlib_metadata import version as _ver
        try:
            v = _ver('RTIMULib').split('.')
            n = (int(v[0])*1000 + int(v[1]))*1000 + int(v[2])
            if n < 8001000:
                print('RTIMULib version out of date')
                return False
        except Exception:
            pass  # version check non-fatal if metadata unavailable

        return True

    def install(self):
        return os.system('. scripts/install_rtimulib.sh') == 0

class data_dep(dep):
    def __init__(self):
        super(data_dep, self).__init__('data')

    def test(self, check=False):
        return os.path.exists('ui/compass.png')

    def install(self):
        return os.system('. scripts/install_data.sh') == 0

class subsystem(object):
    def __init__(self, name, info, deps):
        self.name = name
        self.info = info
        self.deps = deps
        self.summary = 'SUCCESS'

    def install(self):
        allok = True
        for dep in self.deps:
            sys.stdout.write('checking for ' + dep.name + '... ')
            if dep.test():
                print('done')
            else:
                global success
                print(dep.name, 'not found')
                if not dep.install():
                    print('failed to install', dep.name)
                    self.summary = 'failed to install ' + dep.name
                    success = False
                elif dep.test():
                    print('install dependency', dep.name, 'success')
                else:
                    print('dependency failed to install', dep.name)
                    self.summary = 'failed to detect after installing ' + dep.name
                    success = False

    def result(self):
        return self.name + ' ' + self.info + ':' + self.summary

subsystems = []
def ss(*cargs):
    subsystems.append(subsystem(*cargs))
    
# autopilot dependencies (required): RTIMULIB2 python3-serial libpython3-dev python3-numpy python3-scipy swig
#                        (recommended): python3-ujson python3-pyudev python3-zeroconf

ss('autopilot', 'core autopilot or imu-only mode',
   [RTIMULIB2_dep(), py_dep('serial'), py_dep('numpy'), py_dep('scipy'), sys_dep('libpython3-dev'), sys_dep('swig')])

# dependencies not required but reduce cpu usage considerably
ss('optimize', '(recommended) core autopilot operations',
   [py_dep('ujson'), py_dep('pyudev'), py_dep('inotify')])

# signalk dependencies: python3-zerconf python3-requests python3-websocket
ss('signalk', 'communicate with signalk-node-server distributed with openploter',
   [py_dep('zeroconf'), py_dep('requests'), py_dep('websocket')])
        
# hat dependencies: python3-pil (or pillow)
ss('hat', 'SPI lcd keypad, and remote control interface',
   [py_dep('pil'), wiringpi_dep('wiringpi')])

# web dependencies: python3-flask python3-gevent-websocket
ss('web', 'web browser control',
   [py_dep('flask'), py_dep('gevent-websocket'), py_dep('python-socketio'), py_dep('flask-socketio'), py_dep('flask-babel')])

# client dependencies (viewers control applications): python3-wxgtk4.0
# optional: python3-opengl python3-pyglet pywavefront
ss('python_gui', 'python scripts for control and configuration',
   [sys_dep('python3-wxgtk4.0'), py_dep('opengl'), py_dep('pyglet'), py_dep('pywavefront')])

ss('data', 'data files used by various pypilot components',
   [data_dep(), sys_dep('gettext')])


if os.path.basename(os.path.abspath(os.curdir)) != 'pypilot':
    print('please run this script from the pypilot directory')
    exit(1)

for s in subsystems:
    s.install()
print('')
print('')
print('summary of pypilot dependencies')
for s in subsystems:
    r = s.result()
    print(r)
print('')
print('')

if success:
    f = open('deps', 'w')
    for s in subsystems:
        r = s.result()
        f.write(r + '\n')
    f.close()
