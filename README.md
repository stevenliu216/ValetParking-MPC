## MPC for Automated Valet Parking [![Build Status](https://travis-ci.com/stevenliu216/EECS561-final-project.svg?token=nsybZNLKNQxgq5LiqTqf&branch=master)](https://travis-ci.com/stevenliu216/EECS561-final-project)
Our EECS 561 (Digital Controls) final project.

## Setup
Make a python virtual environment (to exit the venv, `deactivate`):
```
cd ${your virtual environment directory}
python3 -m venv eecs561
source eecs561/bin/activate
```

Install dependencies and run:
```
cd ${your desired directory for this project}
git clone https://github.com/stevenliu216/EECS561-final-project.git
pip install -r requirements.txt

# Run the code
python main.py
```

## Diagram
![Diagram](doc/diagram.png)

## References
We referenced Atsushi Sakai's PythonRobotics repository for example codes for implementing MPC control in python.
```
@misc{PythonRobotics,
Author = {Atsushi Sakai and Daniel Ingram and Joseph Dinius and Karan Chawla and Antonin Raffin and Alexis Paques},
Title = {PythonRobotics: a Python code collection of robotics algorithms},
Year = {2018},
Eprint = {arXiv:1808.10703},
}
```
