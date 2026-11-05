import argparse
import os
import subprocess
import sys

def run_shell(cmd, allow_non_zero=False, stderr=None, real_shell=False):
  """Get var_name either from env, or user or default.
  Args:
    cmd: copy of the os.environ.
    allow_non_zero: string for name of environment variable, e.g. "TF_NEED
    stderr: string for how to ask for user input.
  Returns:
    string value output of the command executed.
  """
  if stderr is None:
    stderr = sys.stdout
  if allow_non_zero:
    try:
      output = subprocess.check_output(cmd, stderr=stderr, shell=real_shell)
    except subprocess.CalledProcessError as e:
      output = e.output
  else:
    output = subprocess.check_output(cmd, stderr=stderr, shell=real_shell)
  return output.decode('UTF-8').strip()

def setup_python_environment(environ_cp):
   #this change make the scripts can only be executed at current directory not the root directory
   #cmd = ["virtualenv --no-site-packages ./python/venv && . python/venv/bin/activate && pip3 install --upgrade pip && pip3 install --index-url=https://pypi.python.org/simple/ -r requirements.txt && deactivate"]
    cmd = ["virtualenv --no-site-packages ./python/venv && . python/venv/bin/activate && pip3 install --upgrade pip && pip3 install -r requirements.txt && deactivate"]
    proc = subprocess.Popen(cmd, shell=True)
    try:
      outs, errs = proc.communicate(timeout=15)
    except subprocess.TimeoutExpired:
      proc.kill()
      outs, errs = proc.communicate()

def main():
     # Make a copy of os.environ to be clear when functions and getting and setting
     # environment vaiables.
    environ_cp = dict(os.environ, PATH="path")
    setup_python_environment(environ_cp)

if __name__ == '__main__':
  main()
