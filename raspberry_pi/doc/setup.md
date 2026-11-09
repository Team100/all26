# Setup

How to set up your dev environment.

## Windows

Windows 11 seems to come with python 3.11, which is antique.

* Uninstall 3.11 using the Windows "programs" control panel.
* Install 3.14 from https://www.python.org/downloads/.

## Linux

* Install Ubuntu 26.04
  * This is the only option for WPILib.
  * At the moment, the 26.04 release is not actually an LTS release
    (because there's no 26.04.1 release),
    so you have to get there using the non-LTS path.

## Vscode

On both Windows and Linux, you need these vscode extensions:

* python (microsoft)
* mypy type checker (microsoft)
* pylint (microsoft)
* black formatter (microsoft)

## Python

* Create a virtual environment for the build
```
python -m venv ./env
```
* Install required libraries (on windows, use 'py' not 'python')
```
source ./env/bin/activate
python -m ensurepip --default-pip
python -m pip install --upgrade pip
python -m pip install numpy
python -m pip install opencv-python
python -m pip install robotpy
python -m pip install robotpy-apriltag
python -m pip install robotpy-cscore
python -m pip install --index-url https://download.pytorch.org/whl/cpu torch torchvision
python -m pip install ultralytics==8.4.187
python -m pip install ncnn pnnx==20260526
```
* On Windows, install one more library
```
python -m pip install aiohttp
```
* Choose this venv as the interpreter in vscode.
    * type shift-ctrl-P
    * choose "Python: select interpreter"
    * select ./env


## Testing in vscode

Use the "Python" extension in vscode, which should populate the "test" pane once it
is set up.

## Testing in Github Actions

There's a Github workflow file that runs the raspberry pi
tests, using pytest.  It's in
`all26/.github/workflows`