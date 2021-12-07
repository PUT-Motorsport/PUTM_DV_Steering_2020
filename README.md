# PUTM_DV_CONTROL_METHODS
Repository for Formula Student Driverless Simulator lateral steering methods.  
The program uses two models for simulations:
- [Dynamic Bycylse Model](https://thef1clan.com/2020/12/23/vehicle-dynamics-the-dynamic-bicycle-model/)
- [Kinematic Bycylse Model](https://thef1clan.com/2020/09/21/vehicle-dynamics-the-kinematic-bicycle-model/)


## Instalation

0. Clone repository

```bash
git clone 
cd lateral_control
```

1. Install requirements

```bash
pip3 install -r requirements.txt
```

2. To run a simulation

```bash
python3 main.py
```

3. To create a new path
```bash
python3 path_maker.py
```

## Lateral Control Methods
In this repository, we compare performance of two lateral control methods:

- Pure Pursuit
- Stanley


So far, Pure Pursuit method is looking more promising, but more testes are required. Additionally, path_planner needs to be rewritten, because different path points distances affect controllers performance   

### To do:
- change path planer to polynomial based
- implement MPC controller for comparison