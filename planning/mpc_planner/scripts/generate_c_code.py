from acados_template import *
import acados_template as at
from export_ode_model import *
import numpy as np
import scipy.linalg
from ctypes import *
from math import sqrt

ACADOS_PATH = "/home/rohan/tools/acados"

# create render arguments
ocp = AcadosOcp()

# export model
model = export_ode_model()
ocp.model = model

Tf = 4.0
N = 40
nx = model.x.size()[0]
nu = model.u.size()[0]
ny = nx + nu
ny_e = nx

# set ocp_nlp_dimensions
ocp.dims.nx  = nx
ocp.dims.ny  = ny
ocp.dims.ny_e = ny_e
ocp.dims.nbx = 0
ocp.dims.nbu = nu
ocp.dims.nbx_e = 0
ocp.dims.nu  = model.u.size()[0]
ocp.dims.N   = N

# set cost
Q_mat = np.diag([120, 100, 0.0, 0.0])  # [x,y,x_d,th]
R_mat = np.diag([30, 800])
Q_mat_e = np.diag([0, 0, 0.0, 200])
P_mat = np.diag([200, 200, 0, 0])

ocp.cost.cost_type = "EXTERNAL"
ocp.cost.cost_type_e = "EXTERNAL"

ocp.model.cost_expr_ext_cost = (model.x - model.p[:4]).T @ Q_mat @ (model.x - model.p[:4]) + model.u.T @ R_mat @ model.u + \
                                (200 * sqrt((model.x[0] - model.p[4])^2 + (model.x[1] - model.p[5])^2))
ocp.model.cost_expr_ext_cost_e = (model.x - model.p).T @ Q_mat_e @ (model.x - model.p)

# set constraints
ocp.constraints.lbu = np.array([-3.0, -0.5])
ocp.constraints.ubu = np.array([+0.75, +0.5])
ocp.constraints.idxbu = np.array([0, 1])
ocp.constraints.x0 = np.array([0, 0, 0, 0])

## set QP solver
#ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_QPOASES'
ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
ocp.solver_options.integrator_type = 'ERK'
ocp.solver_options.nlp_solver_max_iter = 100

# set prediction horizon
ocp.solver_options.tf = Tf
#ocp.solver_options.nlp_solver_type = 'SQP_RTI'
ocp.solver_options.nlp_solver_type = 'SQP'

# set header path
ocp.acados_include_path  = f'{ACADOS_PATH}/include'
ocp.acados_lib_path      = f'{ACADOS_PATH}/lib'

acados_ocp_solver = AcadosOcpSolver(
        ocp, json_file="acados_ocp_" + ocp.model.name + ".json"
    )

print('>> NMPC exported')